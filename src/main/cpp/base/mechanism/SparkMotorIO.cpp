#include "MechanismHardwareInternal.h"
#include "base/detail/SparkControl.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <frc/DriverStation.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/voltage.h>

namespace base
{
namespace detail
{
bool SparkDeviceConfigured(rev::spark::SparkMax &motor, bool &resetLatched)
{
    const auto warnings = motor.GetStickyWarnings();
    const bool warningStatus = motor.GetLastError() == rev::REVLibError::kOk;
    resetLatched = resetLatched || warnings.hasReset;
    const auto faults = motor.GetFaults();
    return warningStatus && motor.GetLastError() == rev::REVLibError::kOk && faults.rawBits == 0 && !resetLatched;
}
void ConfigureSparkPid(rev::spark::ClosedLoopConfig &settings, const Gains &gains, double scale)
{
    const double p = gains.p * scale / 12;
    const double i = gains.i * scale / 12 / 1000;
    const double d = gains.d * scale / 12 * 1000;
    for (double value : {p, i, d})
        if (!std::isfinite(value))
            throw std::invalid_argument("Spark PID conversion overflow");
    settings.Pid(p, i, d);
}
void ConfigureSparkGravity(rev::spark::FeedForwardConfig &settings, double volts, double scale, Gravity gravity,
                           bool joint)
{
    // Each setter selects a mode and overwrites the shared gravity parameter.
    // Clear the prior cosine ratio explicitly when returning to constant.
    if (joint && gravity == Gravity::Cosine)
        settings.kCos(volts).kCosRatio(scale);
    else
        settings.kG(joint && gravity == Gravity::Constant ? volts : 0).kCosRatio(0);
}
} // namespace detail

namespace
{
class SparkConfigurationTimeout
{
  public:
    explicit SparkConfigurationTimeout(rev::spark::SparkMax &motor) : motor_(motor)
    {
        if (motor_.SetCANTimeout(250) != rev::REVLibError::kOk)
        {
            motor_.SetCANTimeout(0);
            throw std::runtime_error("Spark configuration timeout setup failed");
        }
    }
    ~SparkConfigurationTimeout()
    {
        if (!restored_)
            motor_.SetCANTimeout(0);
    }
    void Restore()
    {
        if (motor_.SetCANTimeout(0) != rev::REVLibError::kOk)
            throw std::runtime_error("Spark runtime timeout restoration failed");
        restored_ = true;
    }

  private:
    rev::spark::SparkMax &motor_;
    bool restored_ = false;
};
class SparkMotorIO final : public MotorIO
{
  public:
    explicit SparkMotorIO(MotorConfig config) : config_(std::move(config))
    {
        if (frc::DriverStation::IsEnabled())
            throw std::logic_error("Motor construction requires disabled state");
        if (!config_.canBus.empty() && config_.canBus != "rio")
            throw std::invalid_argument("SparkMax requires roboRIO CAN bus");
        for (size_t i = 0; i < config_.ids.size(); ++i)
        {
            if (frc::DriverStation::IsEnabled())
                throw std::logic_error("Spark startup configuration requires disabled state");
            auto motor = std::make_unique<rev::spark::SparkMax>(config_.ids[i],
                                                                rev::spark::SparkLowLevel::MotorType::kBrushless);
            // Persisted signal periods may be slow or disabled. Establish bounded
            // health-frame delivery before waiting for cleared boot history.
            rev::spark::SparkMaxConfig healthSignals;
            healthSignals.signals.FaultsPeriodMs(20).FaultsAlwaysOn(true).WarningsPeriodMs(20).WarningsAlwaysOn(true);
            if (motor->Configure(healthSignals, rev::ResetMode::kNoResetSafeParameters,
                                 rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
                throw std::runtime_error("Spark startup health signal configuration failed");
            // Clear and confirm boot history before writing configuration. A reset
            // during or after configuration must remain visible to runtime checks.
            if (motor->ClearFaults() != rev::REVLibError::kOk)
                throw std::runtime_error("Spark startup reset acknowledgement failed");
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{250};
            while (true)
            {
                if (frc::DriverStation::IsEnabled())
                    throw std::logic_error("Spark startup acknowledgement requires disabled state");
                const auto warnings = motor->GetStickyWarnings();
                if (motor->GetLastError() != rev::REVLibError::kOk)
                    throw std::runtime_error("Spark startup warning read failed");
                if (!warnings.hasReset)
                    break;
                if (std::chrono::steady_clock::now() >= deadline)
                    throw std::runtime_error("Spark startup reset acknowledgement timed out");
                std::this_thread::sleep_for(std::chrono::milliseconds{5});
            }
            if (frc::DriverStation::IsEnabled())
                throw std::logic_error("Spark full configuration requires disabled state");
            rev::spark::SparkMaxConfig settings;
            settings.signals.FaultsPeriodMs(20).FaultsAlwaysOn(true).WarningsPeriodMs(20).WarningsAlwaysOn(true);
            settings.SmartCurrentLimit(config_.currentLimit).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
            settings.VoltageCompensation(12);
            if (i == 0)
                settings.Inverted(config_.reversed.front());
            else
                settings.Follow(config_.ids.front(), config_.reversed[i]);
            settings.encoder.PositionConversionFactor(1 / config_.gearRatio)
                .VelocityConversionFactor(1 / (60 * config_.gearRatio));
            if (config_.spark.feedback == SparkControlConfig::Feedback::AbsoluteEncoder)
                settings.absoluteEncoder.PositionConversionFactor(1 / config_.spark.sensorToMechanismRatio)
                    .VelocityConversionFactor(1 / (60 * config_.spark.sensorToMechanismRatio))
                    .Inverted(config_.spark.sensorReversed)
                    .ZeroOffset(config_.spark.zeroOffset);
            settings.closedLoop.SetFeedbackSensor(config_.spark.feedback ==
                                                          SparkControlConfig::Feedback::AbsoluteEncoder
                                                      ? rev::spark::FeedbackSensor::kAbsoluteEncoder
                                                      : rev::spark::FeedbackSensor::kPrimaryEncoder);
            if (motor->Configure(settings, rev::ResetMode::kResetSafeParameters,
                                 rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
                throw std::runtime_error("SparkMax configuration failed");
            if (motor->SetCANTimeout(0) != rev::REVLibError::kOk)
                throw std::runtime_error("Spark runtime timeout configuration failed");
            motors_.push_back(std::move(motor));
        }
    }
    MotorInputs Read() override
    {
        const bool configured = CheckConfigured();
        auto &motor = *motors_.front();
        const bool absolute = config_.spark.feedback == SparkControlConfig::Feedback::AbsoluteEncoder;
        double position = absolute ? motor.GetAbsoluteEncoder().GetPosition() : motor.GetEncoder().GetPosition();
        bool connected = motor.GetLastError() == rev::REVLibError::kOk;
        double velocity = absolute ? motor.GetAbsoluteEncoder().GetVelocity() : motor.GetEncoder().GetVelocity();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double appliedOutput = motor.GetAppliedOutput();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double supplyVoltage = motor.GetBusVoltage();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double current = motor.GetOutputCurrent();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        lastPosition_ = position;
        return {position,      velocity, (config_.reversed.front() ? -1 : 1) * appliedOutput * supplyVoltage,
                supplyVoltage, current,  connected && configured};
    }
    void SetVoltage(double volts) override
    {
        if (!CheckConfigured())
        {
            StopAll();
            return;
        }
        // Native followers remain followers even while stopping or characterizing.
        motors_.front()->SetVoltage(units::volt_t{volts});
    }
    bool ConfigureNative(const Gains &gains, double scale, Gravity gravity, bool joint) override
    {
        if (!CheckConfigured())
        {
            StopAll();
            throw std::runtime_error("Spark reset requires robot restart and full configuration");
        }
        scale_ = scale;
        gains_ = gains;
        gravity_ = gravity;
        joint_ = joint;
        for (double value : {gains.v * scale, gains.a * scale, gains.minimum / scale, gains.maximum / scale,
                             gains.maxVelocity / scale, gains.maxAcceleration / scale})
            if (!std::isfinite(value))
                throw std::invalid_argument("Spark control conversion overflow");
        rev::spark::SparkMaxConfig settings;
        // Encoders report output rotations; PID outputs duty cycle. Nominal voltage compensation maps
        // voltage gains to duty gains. REV I/D use ms; feedforward uses volts and seconds.
        detail::ConfigureSparkPid(settings.closedLoop, gains, scale);
        settings.closedLoop.feedForward.kS(gains.s).kV(gains.v * scale).kA(gains.a * scale);
        detail::ConfigureSparkGravity(settings.closedLoop.feedForward, gains.gravity, scale, gravity, joint);
        settings.closedLoop.maxMotion.CruiseVelocity(gains.maxVelocity / scale)
            .MaxAcceleration(gains.maxAcceleration / scale)
            .AllowedProfileError(gains.tolerance / scale);
        settings.softLimit.ForwardSoftLimitEnabled(joint)
            .ReverseSoftLimitEnabled(joint)
            .ForwardSoftLimit(gains.maximum / scale)
            .ReverseSoftLimit(gains.minimum / scale);
        SparkConfigurationTimeout timeout{*motors_.front()};
        if (motors_.front()->Configure(settings, rev::ResetMode::kNoResetSafeParameters,
                                       rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
            throw std::runtime_error("Spark onboard control configuration failed");
        timeout.Restore();
        return true;
    }
    bool SetNative(const NativeControlRequest &request) override
    {
        if (!CheckConfigured())
        {
            StopAll();
            return false;
        }
        const double target = request.target / scale_;
        double feedforward = 0;
        auto type = rev::spark::SparkLowLevel::ControlType::kPosition;
        if (request.kind == NativeControlRequest::Kind::MotionMagic)
            type = rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;
        else if (request.kind == NativeControlRequest::Kind::Velocity)
        {
            type = rev::spark::SparkLowLevel::ControlType::kVelocity;
            feedforward = gains_.a * request.acceleration;
            // REV gravity feedforward is inactive in velocity mode.
            if (joint_ && gravity_ != Gravity::Sine)
            {
                const double position = lastPosition_ * scale_ * 2 * std::numbers::pi;
                feedforward += gains_.gravity * (gravity_ == Gravity::Constant ? 1 : std::cos(position));
            }
        }
        else
        {
            // REV basic position mode does not apply kV/kA automatically.
            feedforward = gains_.v * request.velocity + gains_.a * request.acceleration;
        }
        if (joint_ && gravity_ == Gravity::Sine)
            feedforward += gains_.gravity * std::sin(lastPosition_ * scale_ * 2 * std::numbers::pi);
        if (!std::isfinite(target) || !std::isfinite(feedforward))
            return false;
        return motors_.front()->GetClosedLoopController().SetSetpoint(
                   target, type, rev::spark::ClosedLoopSlot::kSlot0, feedforward,
                   rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage) == rev::REVLibError::kOk;
    }
    void SetBrake(bool enabled) override
    {
        if (brake_ == enabled)
            return;
        if (frc::DriverStation::IsEnabled())
            throw std::logic_error("Neutral mode configuration requires disabled state");
        rev::spark::SparkMaxConfig settings;
        settings.SetIdleMode(enabled ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                                     : rev::spark::SparkBaseConfig::IdleMode::kCoast);
        for (auto &motor : motors_)
        {
            SparkConfigurationTimeout timeout{*motor};
            if (motor->Configure(settings, rev::ResetMode::kNoResetSafeParameters,
                                 rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
            {
                StopAll();
                throw std::runtime_error("Spark brake configuration failed");
            }
            timeout.Restore();
        }
        brake_ = enabled;
    }

  private:
    bool CheckConfigured()
    {
        bool healthy = true;
        for (auto &motor : motors_)
        {
            healthy = detail::SparkDeviceConfigured(*motor, resetLatched_) && healthy;
        }
        return healthy && !resetLatched_;
    }
    void StopAll()
    {
        for (auto &motor : motors_)
            motor->SetVoltage(units::volt_t{0});
    }
    double scale_ = 1, lastPosition_ = 0;
    Gains gains_;
    Gravity gravity_ = Gravity::Constant;
    bool joint_ = false, brake_ = true, resetLatched_ = false;
    MotorConfig config_;
    std::vector<std::unique_ptr<rev::spark::SparkMax>> motors_;
};

} // namespace

namespace detail
{
std::unique_ptr<MotorIO> MakeSparkMotorIO(const MotorConfig &config)
{
    return std::make_unique<SparkMotorIO>(config);
}
} // namespace detail

} // namespace base
