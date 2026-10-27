#include "base/Mechanism.h"
#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace
{
class StalledMotor final : public base::MotorIO
{
  public:
    base::MotorInputs Read() override
    {
        base::MotorInputs value;
        value.connected = true;
        return value;
    }
    void SetVoltage(double value) override
    {
        voltage = value;
    }
    void SetBrake(bool) override
    {
        ConfigureNative({}, 1, base::Gravity::Constant, false);
    }
    bool SetNative(const base::NativeControlRequest &) override
    {
        ++requests;
        return true;
    }
    bool ConfigureNative(const base::Gains &, double, base::Gravity, bool) override
    {
        if (first)
        {
            first = false;
            return true;
        }
        std::unique_lock lock(mutex);
        entered = true;
        wake.notify_all();
        if (!wake.wait_for(lock, std::chrono::seconds(5), [this] { return released; }))
            throw std::runtime_error("Fixture stall timed out");
        released = false;
        entered = false;
        if (fail)
            throw std::runtime_error("deliberate configuration failure");
        return true;
    }
    bool Await()
    {
        std::unique_lock lock(mutex);
        return wake.wait_for(lock, std::chrono::seconds(2), [this] { return entered; });
    }
    void Release(bool failure = false)
    {
        std::lock_guard lock(mutex);
        fail = failure;
        released = true;
        wake.notify_all();
    }
    bool first = true, entered = false, released = false, fail = false;
    double voltage = 0;
    int requests = 0;
    std::mutex mutex;
    std::condition_variable wake;
};
void Enable(bool enabled)
{
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
}
bool Complete(base::MotorIO &io)
{
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (io.ConfigurationState() == base::MotorConfigurationState::Pending && std::chrono::steady_clock::now() < end)
        std::this_thread::yield();
    return io.ConfigurationState() != base::MotorConfigurationState::Pending;
}
} // namespace
TEST(AsyncMotorConfiguration, StalledWorkerCannotBlockControlOrResumeOldIntent)
{
    HAL_Initialize(500, 0);
    Enable(false);
    auto child = std::make_unique<StalledMotor>();
    auto *observed = child.get();
    auto io = base::MakeAsyncMotorIO(std::move(child));
    auto *adapter = io.get();
    base::Flywheel wheel("AsyncConfiguration", std::move(io), {});
    Enable(true);
    wheel.SetVelocity(1);
    wheel.Periodic();
    ASSERT_EQ(observed->requests, 1);
    Enable(false);
    wheel.SetGains({});
    ASSERT_TRUE(observed->Await());
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i)
    {
        wheel.Periodic();
        wheel.SetVelocity(2);
        wheel.Stop();
    }
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
    EXPECT_EQ(wheel.ConfigurationState(), base::MotorConfigurationState::Pending);
    EXPECT_FALSE(wheel.Inputs().connected);
    EXPECT_THROW(wheel.SetGains({}), std::logic_error);
    observed->Release();
    ASSERT_TRUE(Complete(*adapter));
    wheel.Periodic();
    Enable(true);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 1);
    EXPECT_DOUBLE_EQ(observed->voltage, 0);
    wheel.SetVelocity(3);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 2);
    Enable(false);
    wheel.SetBrake(false);
    ASSERT_TRUE(observed->Await());
    wheel.Periodic();
    observed->Release();
    ASSERT_TRUE(Complete(*adapter));
    wheel.Periodic();
    Enable(true);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 2);
    wheel.SetVelocity(3);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 3);
    Enable(false);
    wheel.SetGains({});
    ASSERT_TRUE(observed->Await());
    Enable(true);
    wheel.Periodic();
    Enable(false);
    observed->Release();
    ASSERT_TRUE(Complete(*adapter));
    EXPECT_EQ(adapter->ConfigurationState(), base::MotorConfigurationState::Fault);
    wheel.Periodic();
    wheel.SetVelocity(3);
    Enable(true);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 3);
    Enable(false);
    wheel.SetGains({});
    ASSERT_TRUE(observed->Await());
    observed->Release(true);
    ASSERT_TRUE(Complete(*adapter));
    EXPECT_EQ(adapter->ConfigurationState(), base::MotorConfigurationState::Fault);
    wheel.Periodic();
    Enable(false);
    EXPECT_THROW(wheel.SetBrake(true), std::logic_error);
    EXPECT_EQ(adapter->ConfigurationState(), base::MotorConfigurationState::Fault);
    wheel.SetGains({});
    ASSERT_TRUE(observed->Await());
    observed->Release();
    ASSERT_TRUE(Complete(*adapter));
    wheel.Periodic();
    Enable(true);
    wheel.SetVelocity(4);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 4);
    Enable(false);
    wheel.SetBrake(true);
    ASSERT_TRUE(observed->Await());
    observed->Release(true);
    ASSERT_TRUE(Complete(*adapter));
    wheel.Periodic();
    EXPECT_THROW(wheel.SetGains({}), std::logic_error);
    EXPECT_EQ(adapter->ConfigurationState(), base::MotorConfigurationState::Fault);
    EXPECT_THROW(wheel.SetBrake(false), std::logic_error);
    wheel.SetBrake(true);
    EXPECT_THROW(wheel.SetBrake(true), std::logic_error);
    ASSERT_TRUE(observed->Await());
    observed->Release();
    ASSERT_TRUE(Complete(*adapter));
    wheel.Periodic();
    Enable(true);
    wheel.SetVelocity(4);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 5);
    Enable(false);
    base::telemetry::Flush();
}
