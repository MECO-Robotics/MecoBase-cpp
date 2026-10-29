#include "base/PortableSwerve.h"
#include "base/Drive.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <studica/AHRS.h>
#include <frc/RobotController.h>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <cstdint>
#include <chrono>
#include <thread>
#include <frc/DriverStation.h>
namespace base
{
namespace
{
constexpr double tau = 2 * std::numbers::pi;
class SparkModule final : public SwerveModuleIO
{
    rev::spark::SparkMax drive_, steer_;
    rev::spark::SparkRelativeEncoder driveEncoder_, steerEncoder_;
    rev::spark::SparkAbsoluteEncoder absolute_;
    rev::spark::SparkClosedLoopController driveControl_, steerControl_;
    DriveConfig config_;
    bool commandOK_ = true, resetLatched_ = false;
    double steerRate_{};

  public:
    SparkModule(const DriveConfig &c, size_t i)
        : drive_(c.modules[i].drive, rev::spark::SparkLowLevel::MotorType::kBrushless),
          steer_(c.modules[i].steer, rev::spark::SparkLowLevel::MotorType::kBrushless),
          driveEncoder_(drive_.GetEncoder()), steerEncoder_(steer_.GetEncoder()),
          absolute_(steer_.GetAbsoluteEncoder()), driveControl_(drive_.GetClosedLoopController()),
          steerControl_(steer_.GetClosedLoopController()), config_(c)
    {
        rev::spark::SparkMaxConfig drive, steer;
        const auto &m = c.modules[i];
        drive.Inverted(m.driveInverted)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
            .SmartCurrentLimit(static_cast<int>(c.driveCurrent))
            .VoltageCompensation(12);
        steer.Inverted(m.steerInverted)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
            .SmartCurrentLimit(static_cast<int>(c.steerCurrent))
            .VoltageCompensation(12);
        drive.encoder.PositionConversionFactor(tau * c.wheelRadius / c.driveRatio)
            .VelocityConversionFactor(tau * c.wheelRadius / (60 * c.driveRatio));
        steer.encoder.PositionConversionFactor(tau / c.steerRatio).VelocityConversionFactor(tau / (60 * c.steerRatio));
        steer.absoluteEncoder.Inverted(m.encoderInverted)
            .ZeroOffset(m.offsetTurns)
            .PositionConversionFactor(tau)
            .VelocityConversionFactor(tau / 60);
        // Public derivative gains use seconds; REV D uses milliseconds.
        drive.closedLoop.P(c.spark.driveP)
            .D(c.spark.driveD * 1000)
            .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder);
        steer.closedLoop.P(c.spark.steerP)
            .D(c.spark.steerD * 1000)
            .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
            .PositionWrappingEnabled(true)
            .PositionWrappingInputRange(0, tau);
        drive.signals.FaultsPeriodMs(20).WarningsPeriodMs(20).FaultsAlwaysOn(true).WarningsAlwaysOn(true);
        steer.signals.FaultsPeriodMs(20).WarningsPeriodMs(20).FaultsAlwaysOn(true).WarningsAlwaysOn(true);
        drive.signals.PrimaryEncoderPositionPeriodMs(10).PrimaryEncoderVelocityPeriodMs(10).AppliedOutputPeriodMs(20);
        steer.signals.AbsoluteEncoderPositionPeriodMs(10)
            .AbsoluteEncoderVelocityPeriodMs(10)
            .PrimaryEncoderPositionPeriodMs(10)
            .PrimaryEncoderVelocityPeriodMs(10);
        if (frc::DriverStation::IsEnabled())
            throw std::runtime_error("Spark swerve configuration requires disabled state");
        rev::spark::SparkMaxConfig startupSignals;
        startupSignals.signals.FaultsPeriodMs(20).WarningsPeriodMs(20).FaultsAlwaysOn(true).WarningsAlwaysOn(true);
        if (drive_.Configure(startupSignals, rev::ResetMode::kNoResetSafeParameters,
                             rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk ||
            steer_.Configure(startupSignals, rev::ResetMode::kNoResetSafeParameters,
                             rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
            throw std::runtime_error("Spark swerve startup status configuration failed");
        if (drive_.ClearFaults() != rev::REVLibError::kOk || steer_.ClearFaults() != rev::REVLibError::kOk)
            throw std::runtime_error("Spark swerve reset-warning initialization failed");
        // ClearFaults acknowledges a command, not arrival of the refreshed status
        // cache. Establish a cleared baseline before enabling the persistent latch.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        bool cleared = false;
        do
        {
            if (frc::DriverStation::IsEnabled())
                throw std::runtime_error("Spark swerve configuration requires disabled state");
            const auto d = drive_.GetStickyWarnings();
            const bool dOK = drive_.GetLastError() == rev::REVLibError::kOk;
            const auto s = steer_.GetStickyWarnings();
            const bool sOK = steer_.GetLastError() == rev::REVLibError::kOk;
            cleared = dOK && sOK && !d.hasReset && !s.hasReset;
            if (cleared)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (std::chrono::steady_clock::now() < deadline);
        if (!cleared)
            throw std::runtime_error("Spark swerve cleared reset-warning status did not arrive within 250 ms");
        if (frc::DriverStation::IsEnabled())
            throw std::runtime_error("Spark swerve configuration requires disabled state");
        const auto reset = rev::ResetMode::kResetSafeParameters;
        const auto persist = rev::PersistMode::kNoPersistParameters;
        if (drive_.Configure(drive, reset, persist) != rev::REVLibError::kOk ||
            steer_.Configure(steer, reset, persist) != rev::REVLibError::kOk)
            throw std::runtime_error("Spark swerve configuration failed");
        drive_.SetCANTimeout(0);
        steer_.SetCANTimeout(0);
    }
    SwerveModuleInputs Read() override
    {
        const double coupling = config_.couplingRatio * config_.wheelRadius / config_.driveRatio;
        SwerveModuleInputs out;
        bool healthy = commandOK_;
        commandOK_ = true;
        auto driveOK = [&] { healthy = healthy && drive_.GetLastError() == rev::REVLibError::kOk; };
        auto steerOK = [&] { healthy = healthy && steer_.GetLastError() == rev::REVLibError::kOk; };
        const double distance = driveEncoder_.GetPosition();
        driveOK();
        const double azimuth = steerEncoder_.GetPosition();
        steerOK();
        out.distance = distance - coupling * azimuth;
        const double velocity = driveEncoder_.GetVelocity();
        driveOK();
        const double steerVelocity = steerEncoder_.GetVelocity();
        steerOK();
        out.velocity = velocity - coupling * steerVelocity;
        out.angle = absolute_.GetPosition();
        steerOK();
        out.angularVelocity = absolute_.GetVelocity();
        steerOK();
        steerRate_ = out.angularVelocity;
        const double output = drive_.GetAppliedOutput();
        driveOK();
        out.voltage = output * drive_.GetBusVoltage();
        driveOK();
        const auto driveFaults = drive_.GetFaults();
        driveOK();
        const auto steerFaults = steer_.GetFaults();
        steerOK();
        const auto driveWarnings = drive_.GetStickyWarnings();
        driveOK();
        const auto steerWarnings = steer_.GetStickyWarnings();
        steerOK();
        resetLatched_ = resetLatched_ || driveWarnings.hasReset || steerWarnings.hasReset;
        out.healthy = healthy && !resetLatched_ && driveFaults.rawBits == 0 && steerFaults.rawBits == 0;
        return out;
    }
    void Set(const frc::SwerveModuleState &state, double force) override
    {
        if (resetLatched_)
        {
            Stop();
            return;
        }
        double angle = std::fmod(state.angle.Radians().value() + tau, tau);
        const double velocity = state.speed.value();
        const double coupled = velocity + config_.couplingRatio * config_.wheelRadius / config_.driveRatio * steerRate_;
        const double ff = config_.spark.driveKV * velocity +
                          (velocity == 0 ? 0 : std::copysign(config_.spark.driveKS, velocity)) + force;
        if (!std::isfinite(coupled) || !std::isfinite(ff) || !std::isfinite(angle))
        {
            commandOK_ = false;
            Stop();
            return;
        }
        const auto a = driveControl_.SetSetpoint(coupled, rev::spark::SparkLowLevel::ControlType::kVelocity,
                                                 rev::spark::ClosedLoopSlot::kSlot0, ff);
        const auto b = steerControl_.SetSetpoint(angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        commandOK_ = a == rev::REVLibError::kOk && b == rev::REVLibError::kOk;
        if (!commandOK_)
            Stop();
    }
    void SetVoltage(double volts) override
    {
        if (resetLatched_)
        {
            Stop();
            return;
        }
        drive_.SetVoltage(units::volt_t{volts});
        commandOK_ =
            steerControl_.SetSetpoint(0, rev::spark::SparkLowLevel::ControlType::kPosition) == rev::REVLibError::kOk;
    }
    void Stop() override
    {
        drive_.StopMotor();
        steer_.StopMotor();
    }
    bool ConfigureGains(const SparkSwerveGains &gains) override
    {
        rev::spark::SparkMaxConfig drive, steer;
        drive.closedLoop.P(gains.driveP).D(gains.driveD * 1000);
        steer.closedLoop.P(gains.steerP).D(gains.steerD * 1000);
        drive_.SetCANTimeout(100);
        steer_.SetCANTimeout(100);
        struct ResetTimeouts {
            rev::spark::SparkMax &drive, &steer;
            ~ResetTimeouts() {drive.SetCANTimeout(0);steer.SetCANTimeout(0);}
        } reset{drive_,steer_};
        const auto d=drive_.Configure(drive,rev::ResetMode::kNoResetSafeParameters,rev::PersistMode::kNoPersistParameters);
        const auto s=steer_.Configure(steer,rev::ResetMode::kNoResetSafeParameters,rev::PersistMode::kNoPersistParameters);
        if (d!=rev::REVLibError::kOk || s!=rev::REVLibError::kOk) return false;
        config_.spark=gains;
        return true;
    }
};
class NavX final : public SwerveGyroIO
{
    studica::AHRS sensor_{studica::AHRS::NavXComType::kMXP_SPI, studica::AHRS::NavXUpdateRate::k100Hz};
    std::int64_t last_ = -1;
    double changedAt_{};

  public:
    SwerveGyroInputs Read() override
    {
        const auto stamp = sensor_.GetLastSensorTimestamp();
        const double now = frc::RobotController::GetFPGATime() * 1e-6;
        if (stamp != last_)
        {
            last_ = stamp;
            changedAt_ = now;
        }
        return {sensor_.GetRotation2d().Radians().value(),
                sensor_.IsConnected() && !sensor_.IsCalibrating() && stamp > 0 && now - changedAt_ <= .1};
    }
};
class SimModule final : public SwerveModuleIO
{
    SparkSwerveGains gains_;
    SwerveModuleInputs inputs_{0, 0, 0, 0, 0, true};
    frc::SwerveModuleState target_{};
    bool active_ = false, voltage_ = false;
    double volts_{}, force_{}, lastDriveError_{}, lastSteerError_{};

  public:
    explicit SimModule(SparkSwerveGains gains) : gains_(gains)
    {
    }
    SwerveModuleInputs Read() override
    {
        return inputs_;
    }
    void Set(const frc::SwerveModuleState &state, double force) override
    {
        target_ = state;
        force_ = force;
        active_ = true;
        voltage_ = false;
    }
    void SetVoltage(double volts) override
    {
        volts_ = volts;
        active_ = true;
        voltage_ = true;
        target_.angle = frc::Rotation2d{};
    }
    void Stop() override
    {
        active_ = false;
        lastDriveError_ = lastSteerError_ = 0;
    }
    bool ConfigureGains(const SparkSwerveGains &gains) override { gains_=gains; return true; }
    std::array<double, 2> SimVoltages(double dt, double bus) override
    {
        if (!active_)
        {
            inputs_.voltage = 0;
            return {};
        }
        const double error = target_.speed.value() - inputs_.velocity;
        const double steering = std::remainder(target_.angle.Radians().value() - inputs_.angle, tau);
        const double d =
            voltage_
                ? volts_
                : 12 * (gains_.driveP * error + gains_.driveD * (error - lastDriveError_) / dt) +
                      gains_.driveKV * target_.speed.value() +
                      (target_.speed.value() == 0 ? 0 : std::copysign(gains_.driveKS, target_.speed.value())) + force_;
        const double s = 12 * (gains_.steerP * steering + gains_.steerD * (steering - lastSteerError_) / dt);
        lastDriveError_ = error;
        lastSteerError_ = steering;
        inputs_.voltage = std::clamp(d, -bus, bus);
        return {inputs_.voltage, std::clamp(s, -bus, bus)};
    }
    void SetSimInputs(SwerveModuleInputs value) override
    {
        value.voltage = inputs_.voltage;
        inputs_ = value;
    }
};
class SimGyro final : public SwerveGyroIO
{
    double yaw_{};

  public:
    SwerveGyroInputs Read() override
    {
        return {yaw_, true};
    }
    void SetSimYaw(double yaw) override
    {
        yaw_ = yaw;
    }
};
} // namespace
SwerveIO MakeSparkNavXSwerveIO(const DriveConfig &c, bool simulation)
{
    if (!simulation && frc::DriverStation::IsEnabled())
        throw std::runtime_error("Spark swerve must be constructed while disabled");
    SwerveIO out;
    for (size_t i = 0; i < 4; ++i)
    {
        if (simulation)
            out.modules[i] = std::make_unique<SimModule>(c.spark);
        else
            out.modules[i] = std::make_unique<SparkModule>(c, i);
    }
    if (simulation)
        out.gyro = std::make_unique<SimGyro>();
    else
        out.gyro = std::make_unique<NavX>();
    return out;
}
} // namespace base
