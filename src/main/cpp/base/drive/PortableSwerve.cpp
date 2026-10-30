#include "base/PortableSwerve.h"
#include "base/Drive.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/RobotController.h>
#include <frc/DriverStation.h>
#include <frc/system/plant/DCMotor.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cstdint>
namespace base
{
struct PortableSwerve::Impl
{
    DriveConfig config;
    SwerveIO io;
    frc::SwerveDriveKinematics<4> kinematics;
    frc::SwerveDrivePoseEstimator<4> estimator;
    mutable std::mutex mutex;
    std::mutex ioMutex;
    std::condition_variable wake;
    bool quit = false, validRequest = false, voltageMode = false, estimatorInitialized = false;
    double requestedAt{}, voltage{};
    uint64_t requestGeneration = 0, resetGeneration = 0;
    std::array<double, 4> forceVolts{};
    Snapshot state;
    std::array<Snapshot, 64> queue{};
    size_t head = 0, count = 0;
    std::thread thread;
    static frc::Translation2d Location(const DriveConfig &c, int i)
    {
        return {units::meter_t{c.modules[i].xMeters}, units::meter_t{c.modules[i].yMeters}};
    }
    Impl(const DriveConfig &c, SwerveIO devices)
        : config(c), io(std::move(devices)), kinematics(Location(c, 0), Location(c, 1), Location(c, 2), Location(c, 3)),
          estimator(kinematics, frc::Rotation2d{}, std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d{})
    {
        if (!io.gyro)
            throw std::invalid_argument("Portable swerve requires gyro IO");
        for (const auto &module : io.modules)
            if (!module)
                throw std::invalid_argument("Portable swerve requires four module IO objects");
    }
    void Sample(double timestamp, bool enabled)
    {
        // Configuration may hold IO on a worker. Never block the scheduler's
        // simulation sample or the periodic hardware sampler behind it.
        std::unique_lock ioLock(ioMutex, std::try_to_lock);
        if (!ioLock.owns_lock()) return;
        Snapshot next;
        bool requested, requestVoltage;
        double requestTime, volts;
        std::array<double, 4> forces;
        uint64_t requestEpoch, resetEpoch;
        {
            std::scoped_lock lock(mutex);
            next = state;
            resetEpoch = resetGeneration;
        }
        next.timestamp = timestamp;
        const auto gyro = io.gyro->Read();
        next.rawYaw = gyro.yaw;
        next.healthy = gyro.healthy && std::isfinite(timestamp) && std::isfinite(gyro.yaw);
        for (size_t i = 0; i < 4; ++i)
        {
            const auto m = io.modules[i]->Read();
            next.healthy = next.healthy && m.healthy && std::isfinite(m.distance) && std::isfinite(m.velocity) &&
                           std::isfinite(m.angle) && std::isfinite(m.angularVelocity) && std::isfinite(m.voltage);
            next.positions[i] = {units::meter_t{m.distance}, frc::Rotation2d{units::radian_t{m.angle}}};
            next.states[i] = {units::meters_per_second_t{m.velocity}, frc::Rotation2d{units::radian_t{m.angle}}};
            next.voltages[i] = m.voltage;
        }
        {
            std::scoped_lock lock(mutex);
            requested = validRequest;
            requestVoltage = voltageMode;
            requestTime = requestedAt;
            volts = voltage;
            forces = forceVolts;
            requestEpoch = requestGeneration;
            next.targets = state.targets;
        }
        const double now = frc::RobotController::GetFPGATime() * 1e-6;
        const bool disarm = !enabled || !next.healthy || now - requestTime > .1 || now < requestTime;
        if (disarm)
            requested = false;
        for (size_t i = 0; i < 4; ++i)
        {
            if (!requested)
                io.modules[i]->Stop();
            else if (requestVoltage)
                io.modules[i]->SetVoltage(volts);
            else
                io.modules[i]->Set(next.targets[i], forces[i]);
        }
        {
            std::scoped_lock lock(mutex);
            if (disarm && requestEpoch == requestGeneration)
                validRequest = false;
            if (resetEpoch != resetGeneration)
                return;
            if (!next.healthy)
            {
                head = count = 0;
            }
            if (next.healthy)
            {
                if (count == queue.size())
                {
                    head = (head + 1) % queue.size();
                    --count;
                }
                queue[(head + count) % queue.size()] = next;
                ++count;
            }
            next.targets = state.targets;
            next.pose = state.pose;
            state = next;
        }
    }
};
PortableSwerve::PortableSwerve(const DriveConfig &c, SwerveIO io, bool threaded)
    : impl_(std::make_unique<Impl>(c, std::move(io)))
{
    if (threaded)
        impl_->thread = std::thread([this] {
            auto &p = *impl_;
            std::unique_lock lock(p.mutex);
            const auto period =
                std::chrono::duration<double>(1 / (p.config.odometryHz > 0 ? p.config.odometryHz : 100));
            while (!p.quit)
            {
                lock.unlock();
                p.Sample(frc::RobotController::GetFPGATime() * 1e-6, frc::DriverStation::IsEnabled());
                lock.lock();
                p.wake.wait_for(lock, period, [&] { return p.quit; });
            }
        });
}
PortableSwerve::~PortableSwerve()
{
    {
        std::scoped_lock lock(impl_->mutex);
        impl_->quit = true;
        impl_->wake.notify_all();
    }
    if (impl_->thread.joinable())
        impl_->thread.join();
    for (auto &module : impl_->io.modules)
        module->Stop();
}
void PortableSwerve::Velocity(const frc::ChassisSpeeds &speeds, const std::array<double, 4> &fx,
                              const std::array<double, 4> &fy)
{
    std::scoped_lock lock(impl_->mutex);
    auto &p = *impl_;
    if (!std::isfinite(speeds.vx.value()) || !std::isfinite(speeds.vy.value()) || !std::isfinite(speeds.omega.value()))
    {
        p.validRequest = false;
        throw std::invalid_argument("Nonfinite swerve chassis request");
    }
    auto targets = p.kinematics.ToSwerveModuleStates(speeds);
    p.kinematics.DesaturateWheelSpeeds(&targets, units::meters_per_second_t{p.config.maxSpeed});
    for (size_t i = 0; i < 4; ++i)
    {
        if (!std::isfinite(fx[i]) || !std::isfinite(fy[i]) || !std::isfinite(targets[i].speed.value()) ||
            !std::isfinite(targets[i].angle.Radians().value()))
        {
            p.validRequest = false;
            throw std::invalid_argument("Nonfinite swerve request");
        }
        targets[i].Optimize(p.state.states[i].angle);
        const double angle = targets[i].angle.Radians().value();
        // Feedforward is only the winding drop for projected wheel torque. Back EMF
        // is already provided by velocity FF and must not be counted a second time.
        const auto motor = frc::DCMotor::NEO(1);
        p.forceVolts[i] = (fx[i] * std::cos(angle) + fy[i] * std::sin(angle)) * p.config.wheelRadius /
                          p.config.driveRatio * motor.R.value() / motor.Kt.value();
        if (!std::isfinite(p.forceVolts[i]))
        {
            p.validRequest = false;
            throw std::invalid_argument("Overflow in swerve force feedforward");
        }
    }
    p.state.targets = targets;
    ++p.requestGeneration;
    p.voltageMode = false;
    p.validRequest = true;
    p.requestedAt = frc::RobotController::GetFPGATime() * 1e-6;
}
void PortableSwerve::Voltage(double volts)
{
    std::scoped_lock lock(impl_->mutex);
    if (!std::isfinite(volts))
    {
        impl_->validRequest = false;
        throw std::invalid_argument("Nonfinite swerve voltage");
    }
    ++impl_->requestGeneration;
    impl_->voltage = volts;
    impl_->voltageMode = true;
    impl_->validRequest = true;
    impl_->requestedAt = frc::RobotController::GetFPGATime() * 1e-6;
}
void PortableSwerve::Brake()
{
    std::scoped_lock lock(impl_->mutex);
    auto &p = *impl_;
    for (size_t i = 0; i < 4; ++i)
        p.state.targets[i] = {
            units::meters_per_second_t{0},
            frc::Rotation2d{units::radian_t{std::atan2(p.config.modules[i].yMeters, p.config.modules[i].xMeters)}}};
    p.forceVolts = {};
    ++p.requestGeneration;
    p.voltageMode = false;
    p.validRequest = true;
    p.requestedAt = frc::RobotController::GetFPGATime() * 1e-6;
}
bool PortableSwerve::InhibitForConfiguration()
{
    std::unique_lock ioLock(impl_->ioMutex, std::try_to_lock);
    if (!ioLock.owns_lock()) return false;
    std::unique_lock lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return false;
    impl_->validRequest=false;
    ++impl_->requestGeneration;
    bool stopped=true;
    for (auto &module : impl_->io.modules) {
        try { module->Stop(); } catch (...) { stopped=false; }
    }
    if (!stopped) throw std::runtime_error("Failed to stop swerve modules for configuration");
    return true;
}
bool PortableSwerve::ConfigureGains(const SparkSwerveGains &gains)
{
    std::scoped_lock ioLock(impl_->ioMutex);
    bool success=true;
    for (auto &module : impl_->io.modules)
        success=module->ConfigureGains(gains) && success;
    return success;
}
void PortableSwerve::Sample(double time, bool enabled)
{
    impl_->Sample(time, enabled);
}
void PortableSwerve::Update()
{
    std::scoped_lock lock(impl_->mutex);
    auto &p = *impl_;
    while (p.count)
    {
        const auto &s = p.queue[p.head];
        if (!p.estimatorInitialized)
        {
            p.estimator.ResetPosition(frc::Rotation2d{units::radian_t{s.rawYaw}}, s.positions, p.state.pose);
            p.estimatorInitialized = true;
        }
        p.estimator.UpdateWithTime(units::second_t{s.timestamp}, frc::Rotation2d{units::radian_t{s.rawYaw}},
                                   s.positions);
        p.head = (p.head + 1) % p.queue.size();
        --p.count;
    }
    p.state.pose = p.estimator.GetEstimatedPosition();
    p.state.speeds = p.kinematics.ToChassisSpeeds(p.state.states);
}
PortableSwerve::Snapshot PortableSwerve::State() const
{
    std::scoped_lock lock(impl_->mutex);
    auto state = impl_->state;
    const double age = frc::RobotController::GetFPGATime() * 1e-6 - state.timestamp;
    state.healthy = state.healthy && age >= -.025 && age <= .1;
    return state;
}
void PortableSwerve::ResetPose(const frc::Pose2d &pose)
{
    std::scoped_lock lock(impl_->mutex);
    auto &p = *impl_;
    ++p.resetGeneration;
    p.head = p.count = 0;
    p.estimatorInitialized = p.estimatorInitialized || p.state.healthy;
    p.estimator.ResetPosition(frc::Rotation2d{units::radian_t{p.state.rawYaw}}, p.state.positions, pose);
    p.state.pose = pose;
}
void PortableSwerve::AddVision(const frc::Pose2d &pose, units::second_t time, std::array<double, 3> noise)
{
    std::scoped_lock lock(impl_->mutex);
    impl_->estimator.AddVisionMeasurement(pose, time, noise);
    impl_->state.pose = impl_->estimator.GetEstimatedPosition();
}
std::array<double, 2> PortableSwerve::SimVoltages(size_t i, double dt, double bus)
{
    std::scoped_lock lock(impl_->ioMutex);
    return impl_->io.modules.at(i)->SimVoltages(dt, bus);
}
void PortableSwerve::SetSimInputs(size_t i, SwerveModuleInputs value)
{
    std::scoped_lock lock(impl_->ioMutex);
    impl_->io.modules.at(i)->SetSimInputs(value);
}
void PortableSwerve::SetSimYaw(double yaw)
{
    std::scoped_lock lock(impl_->ioMutex);
    impl_->io.gyro->SetSimYaw(yaw);
}
} // namespace base
