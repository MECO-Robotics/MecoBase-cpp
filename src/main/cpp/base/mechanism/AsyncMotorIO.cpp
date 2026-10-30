#include "base/Mechanism.h"
#include "base/detail/BackgroundPriority.h"
#include <cstdio>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <frc/DriverStation.h>

namespace base
{
namespace
{
// Only the worker touches the child during configuration. The scheduler never
// waits for that lock, including Stop and Read; configuration starts from zero.
class AsyncMotorIO final : public MotorIO
{
  public:
    explicit AsyncMotorIO(std::unique_ptr<MotorIO> child) : child_(std::move(child)), worker_([this] { Work(); })
    {
    }
    ~AsyncMotorIO() override
    {
        {
            std::lock_guard lock(jobMutex_);
            shutdown_ = true;
        }
        wake_.notify_one();
        worker_.join(); // Lifecycle only, never from a scheduler callback.
    }
    bool UsesAsyncConfiguration() const override
    {
        return true;
    }
    MotorConfigurationState ConfigurationState() const override
    {
        return state_.load();
    }
    MotorInputs Read() override
    {
        const auto state = state_.load();
        if (state == MotorConfigurationState::Pending && neutralJob_ && frc::DriverStation::IsEnabled())
            cancelled_ = true;
        std::unique_lock lock(deviceMutex_, std::try_to_lock);
        MotorInputs input;
        if (lock.owns_lock() && state == MotorConfigurationState::Ready)
            input = child_->Read();
        input.configuration = state;
        input.configurationFailure = failure_.load();
        return input;
    }
    void SetVoltage(double volts) override
    {
        std::unique_lock lock(deviceMutex_, std::try_to_lock);
        if (lock.owns_lock())
            child_->SetVoltage(state_ == MotorConfigurationState::Ready ? volts : 0);
    }
    bool SetNative(const NativeControlRequest &request) override
    {
        std::unique_lock lock(deviceMutex_, std::try_to_lock);
        return lock.owns_lock() && state_ == MotorConfigurationState::Ready && child_->SetNative(request);
    }
    bool ConfigureNative(const Gains &gains, double scale, Gravity gravity, bool joint) override
    {
        if (!initialized_)
        {
            // Mechanism construction precedes scheduler execution and worker jobs.
            const bool native = child_->ConfigureNative(gains, scale, gravity, joint);
            if (!native)
                throw std::logic_error("Async motor requires firmware control");
            initialized_ = true;
            return true;
        }
        if (state_ == MotorConfigurationState::Fault && failure_ != MotorConfigurationFailure::Gains)
            throw std::logic_error("Retry the failed neutral configuration first");
        Submit({false, false, gains, scale, gravity, joint});
        return true; // Firmware ownership, not an acknowledgement of completion.
    }
    void SetBrake(bool brake) override
    {
        if (brake == brake_ && state_ == MotorConfigurationState::Ready)
            return;
        if (state_ == MotorConfigurationState::Fault &&
            failure_ != (brake ? MotorConfigurationFailure::Brake : MotorConfigurationFailure::Coast))
            throw std::logic_error("Retry the failed configuration operation and neutral target");
        Submit({true, brake, {}});
    }

  private:
    struct Job
    {
        bool neutral = false, brake = true;
        Gains gains;
        double scale = 1;
        Gravity gravity = Gravity::Constant;
        bool joint = false;
    };
    void Submit(Job job)
    {
        if (job.neutral && frc::DriverStation::IsEnabled())
            throw std::logic_error("Neutral configuration requires disabled state");
        std::unique_lock jobs(jobMutex_, std::try_to_lock);
        std::unique_lock device(deviceMutex_, std::try_to_lock);
        if (!jobs.owns_lock() || !device.owns_lock() || state_ == MotorConfigurationState::Pending)
            throw std::logic_error("Motor configuration is busy");
        child_->SetVoltage(0);
        cancelled_ = false;
        neutralJob_ = job.neutral;
        job_ = job;
        state_ = MotorConfigurationState::Pending;
        ready_ = true;
        wake_.notify_one();
    }
    void Work()
    {
        if (!detail::BackgroundPriority("Motor-Config"))
            std::fputs("Motor configuration worker priority could not be lowered\n", stderr);
        for (;;)
        {
            Job job;
            {
                std::unique_lock lock(jobMutex_);
                wake_.wait(lock, [this] { return ready_ || shutdown_; });
                if (shutdown_)
                    return;
                job = job_;
                ready_ = false;
            }
            std::lock_guard device(deviceMutex_);
            bool ok = false;
            try
            {
                if (!job.neutral || !frc::DriverStation::IsEnabled())
                {
                    if (job.neutral)
                        child_->SetBrake(job.brake);
                    else if (!child_->ConfigureNative(job.gains, job.scale, job.gravity, job.joint))
                        throw std::runtime_error("Firmware control capability lost");
                    ok = !job.neutral || (!frc::DriverStation::IsEnabled() && !cancelled_);
                }
            }
            catch (const std::exception &)
            { /* The latched state is observed and logged. */
            }
            try
            {
                child_->SetVoltage(0);
            }
            catch (const std::exception &)
            {
                ok = false;
            }
            failure_ = ok            ? MotorConfigurationFailure::None
                       : job.neutral ? job.brake ? MotorConfigurationFailure::Brake : MotorConfigurationFailure::Coast
                                     : MotorConfigurationFailure::Gains;
            if (ok && job.neutral)
                brake_ = job.brake;
            state_ = ok ? MotorConfigurationState::Ready : MotorConfigurationState::Fault;
        }
    }
    std::unique_ptr<MotorIO> child_;
    std::mutex deviceMutex_, jobMutex_;
    std::condition_variable wake_;
    Job job_;
    bool ready_ = false, shutdown_ = false, initialized_ = false;
    std::atomic<bool> brake_{true};
    std::atomic<bool> cancelled_{false}, neutralJob_{false};
    std::atomic<MotorConfigurationFailure> failure_{MotorConfigurationFailure::None};
    std::atomic<MotorConfigurationState> state_{MotorConfigurationState::Ready};
    std::thread worker_;
};
} // namespace
std::unique_ptr<MotorIO> MakeAsyncMotorIO(std::unique_ptr<MotorIO> child)
{
    if (!child)
        throw std::invalid_argument("Missing motor adapter");
    return std::make_unique<AsyncMotorIO>(std::move(child));
}
} // namespace base
