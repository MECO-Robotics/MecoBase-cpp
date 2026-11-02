#include "MechanismHardwareInternal.h"

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <frc/Errors.h>
#include <frc/DriverStation.h>
#include <algorithm>
#include <string>
#include <units/voltage.h>

namespace base::detail
{
namespace
{
void SetStatusRate(int deviceId, const char *signalName, auto &signal, double hz)
{
    if (hz == 0)
        return;
    const auto status = signal.SetUpdateFrequency(units::hertz_t{hz});
    if (!status.IsOK())
    {
        const std::string message = "Phoenix status rate configuration failed for TalonFX " +
                                    std::to_string(deviceId) + " " + signalName + " at " + std::to_string(hz) +
                                    " Hz: " + std::string{status.GetName()};
        FRC_ReportError(frc::err::Error, "{}", message);
        throw std::runtime_error(message);
    }
}

class TalonMotorIO final : public MotorIO
{
  public:
    explicit TalonMotorIO(MotorConfig config) : config_(std::move(config))
    {
        if (frc::DriverStation::IsEnabled())
            throw std::logic_error("Motor construction requires disabled state");
        for (size_t i = 0; i < config_.ids.size(); ++i)
        {
            auto motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config_.ids[i], config_.canBus);
            ctre::phoenix6::configs::TalonFXConfiguration settings;
            {
                settings.MotorOutput.Inverted = config_.reversed[i]
                                                    ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                                                    : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
                // Phoenix keeps its integrated sensor in phase with motor inversion.
                settings.Feedback.SensorToMechanismRatio = config_.gearRatio;
                if (config_.phoenix.feedback == PhoenixControlConfig::Feedback::RemoteCANcoder)
                {
                    settings.Feedback.FeedbackRemoteSensorID = config_.phoenix.remoteSensorId;
                    settings.Feedback.SensorToMechanismRatio =
                        (config_.phoenix.sensorReversed ? -1 : 1) * config_.phoenix.sensorToMechanismRatio;
                    settings.Feedback.FeedbackSensorSource =
                        ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
                    settings.Feedback.RotorToSensorRatio = config_.phoenix.rotorToSensorRatio;
                }
            }
            settings.CurrentLimits.SupplyCurrentLimit = units::ampere_t{static_cast<double>(config_.currentLimit)};
            settings.CurrentLimits.SupplyCurrentLimitEnable = true;
            settings.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            if (!motor->GetConfigurator().Apply(settings).IsOK())
                throw std::runtime_error("TalonFX configuration failed");
            motors_.push_back(std::move(motor));
        }

        // Mechanism feedback and native-controller status belong to the leader;
        // followers keep their own Phoenix defaults and receive only follower
        // control requests. Apply requests during disabled startup, never in Read.
        auto &leader = *motors_.front();
        SetStatusRate(config_.ids.front(), "position", leader.GetPosition(false), config_.rates.positionVelocityHz);
        SetStatusRate(config_.ids.front(), "velocity", leader.GetVelocity(false), config_.rates.positionVelocityHz);
        SetStatusRate(config_.ids.front(), "motor voltage", leader.GetMotorVoltage(false), config_.rates.LeaderOutputHz(motors_.size() > 1));
        SetStatusRate(config_.ids.front(), "supply voltage", leader.GetSupplyVoltage(false), config_.rates.diagnosticHz);
        SetStatusRate(config_.ids.front(), "supply current", leader.GetSupplyCurrent(false), config_.rates.diagnosticHz);
        SetStatusRate(config_.ids.front(), "temperature", leader.GetDeviceTemp(false), config_.rates.diagnosticHz);
        SetStatusRate(config_.ids.front(), "closed-loop reference", leader.GetClosedLoopReference(false),
                      config_.rates.controlHz);
        SetStatusRate(config_.ids.front(), "closed-loop reference slope", leader.GetClosedLoopReferenceSlope(false),
                      config_.rates.controlHz);
        if (config_.phoenix.feedback == PhoenixControlConfig::Feedback::RemoteCANcoder)
            SetStatusRate(config_.ids.front(), "remote sensor validity",
                          leader.GetFault_RemoteSensorDataInvalid(false),
                          std::max(config_.rates.positionVelocityHz, config_.rates.controlHz));
    }
    MotorInputs Read() override
    {
        auto &motor = *motors_.front();
        const auto &position = motor.GetPosition();
        const auto &velocity = motor.GetVelocity();
        const auto &voltage = motor.GetMotorVoltage();
        const auto &supply = motor.GetSupplyVoltage();
        const auto &current = motor.GetSupplyCurrent();
        bool feedbackHealthy = true;
        if (config_.phoenix.feedback == PhoenixControlConfig::Feedback::RemoteCANcoder)
        {
            const auto &invalid = motor.GetFault_RemoteSensorDataInvalid();
            feedbackHealthy = invalid.GetStatus().IsOK() && !invalid.GetValue();
        }
        // Phoenix getters refresh their motor-owned cached signal snapshots.
        double direction = 1;
        const double ratio = 1;
        lastPosition_ = position.GetValueAsDouble();
        return {direction * position.GetValueAsDouble() / ratio,
                direction * velocity.GetValueAsDouble() / ratio,
                (config_.reversed.front() ? -1 : 1) * voltage.GetValueAsDouble(),
                supply.GetValueAsDouble(),
                current.GetValueAsDouble(),
                position.GetStatus().IsOK() && velocity.GetStatus().IsOK() && feedbackHealthy};
    }
    void SetVoltage(double volts) override
    {
        if (volts == 0)
        {
            for (auto &motor : motors_)
                motor->SetVoltage(units::volt_t{0});
        }
        else
        {
            motors_.front()->SetVoltage(units::volt_t{volts});
            if (!Follow())
            {
                for (auto &motor : motors_)
                    motor->SetVoltage(units::volt_t{0});
            }
        }
    }
    bool ConfigureNative(const Gains &gains, double scale, Gravity gravity, bool joint) override
    {
        if (frc::DriverStation::IsEnabled())
            throw std::logic_error("Motor configuration requires disabled state");
        for (double value :
             {gains.p * scale, gains.i * scale, gains.d * scale, gains.v * scale, gains.a * scale,
              gains.minimum / scale, gains.maximum / scale, gains.maxVelocity / scale, gains.maxAcceleration / scale})
            if (!std::isfinite(value))
                throw std::invalid_argument("Native control conversion overflow");
        scale_ = scale;
        accelerationGain_ = gains.a;
        gravity_ = gravity;
        gravityGain_ = joint ? gains.gravity : 0;
        customGravity_ = gravity == Gravity::Sine || (gravity == Gravity::Cosine && scale != 1);
        ctre::phoenix6::configs::Slot0Configs slot;
        slot.kP = gains.p * scale;
        slot.kI = gains.i * scale;
        slot.kD = gains.d * scale;
        slot.kS = gains.s;
        slot.kV = gains.v * scale;
        slot.kA = gains.a * scale;
        slot.kG = customGravity_ ? 0 : gravityGain_;
        slot.GravityType = gravity == Gravity::Cosine ? ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine
                                                      : ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
        auto &configurator = motors_.front()->GetConfigurator();
        ctre::phoenix6::configs::SoftwareLimitSwitchConfigs limits;
        limits.ForwardSoftLimitEnable = joint;
        limits.ReverseSoftLimitEnable = joint;
        limits.ForwardSoftLimitThreshold = units::turn_t{gains.maximum / scale};
        limits.ReverseSoftLimitThreshold = units::turn_t{gains.minimum / scale};
        ctre::phoenix6::configs::MotionMagicConfigs profile;
        profile.MotionMagicCruiseVelocity = units::turns_per_second_t{gains.maxVelocity / scale};
        profile.MotionMagicAcceleration = units::turns_per_second_squared_t{gains.maxAcceleration / scale};
        if (!configurator.Apply(slot).IsOK() || !configurator.Apply(limits).IsOK() ||
            !configurator.Apply(profile).IsOK())
            throw std::runtime_error("Phoenix native control configuration failed");

        return true;
    }
    bool SetNative(const NativeControlRequest &request) override
    {

        for (double value : {request.target / scale_, request.velocity / scale_, request.acceleration / scale_,
                             accelerationGain_ * request.acceleration})
            if (!std::isfinite(value))
                return false;
        auto &motor = *motors_.front();
        const double angle = lastPosition_ * scale_ * 2 * std::numbers::pi;
        const double gravity =
            customGravity_ ? gravityGain_ * (gravity_ == Gravity::Sine ? std::sin(angle) : std::cos(angle)) : 0;
        ctre::phoenix::StatusCode status;
        if (request.kind == NativeControlRequest::Kind::Velocity)
            status = motor.SetControl(
                ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{request.target / scale_}}
                    .WithAcceleration(units::turns_per_second_squared_t{request.acceleration / scale_})
                    .WithFeedForward(units::volt_t{gravity})
                    .WithEnableFOC(false)
                    .WithUpdateFreqHz(units::hertz_t{config_.rates.controlHz > 0 ? config_.rates.controlHz : 100}));
        else if (request.kind == NativeControlRequest::Kind::Position)
            status =
                motor.SetControl(ctre::phoenix6::controls::PositionVoltage{units::turn_t{request.target / scale_}}
                                     .WithVelocity(units::turns_per_second_t{request.velocity / scale_})
                                     .WithFeedForward(units::volt_t{accelerationGain_ * request.acceleration + gravity})
                                     .WithEnableFOC(false)
                    .WithUpdateFreqHz(units::hertz_t{config_.rates.controlHz > 0 ? config_.rates.controlHz : 100}));
        else
        {
            status =
                motor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{units::turn_t{request.target / scale_}}
                                     .WithFeedForward(units::volt_t{gravity})
                                     .WithEnableFOC(false)
                    .WithUpdateFreqHz(units::hertz_t{config_.rates.controlHz > 0 ? config_.rates.controlHz : 100}));
        }
        const bool followers = Follow();
        return status.IsOK() && followers;
    }
    void SetBrake(bool enabled) override
    {
        if (brake_ == enabled)
            return;
        if (frc::DriverStation::IsEnabled())
            throw std::logic_error("Neutral mode configuration requires disabled state");
        for (auto &motor : motors_)
            if (!motor
                     ->SetNeutralMode(enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake
                                              : ctre::phoenix6::signals::NeutralModeValue::Coast)
                     .IsOK())
            {
                SetVoltage(0);
                throw std::runtime_error("Talon brake configuration failed");
            }
        brake_ = enabled;
    }

  private:
    bool Follow()
    {
        bool ok = true;
        for (size_t i = 1; i < motors_.size(); ++i)
            ok = motors_[i]
                     ->SetControl(ctre::phoenix6::controls::Follower{
                         config_.ids.front(), config_.reversed[i]
                                                  ? ctre::phoenix6::signals::MotorAlignmentValue::Opposed
                                                  : ctre::phoenix6::signals::MotorAlignmentValue::Aligned})
                     .IsOK() &&
                 ok;
        return ok;
    }
    double scale_ = 1, accelerationGain_ = 0, gravityGain_ = 0, lastPosition_ = 0;
    Gravity gravity_ = Gravity::Constant;
    bool customGravity_ = false, brake_ = true;
    MotorConfig config_;
    std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
};

} // namespace

std::unique_ptr<MotorIO> MakeTalonMotorIO(const MotorConfig &config)
{
    return std::make_unique<TalonMotorIO>(config);
}

} // namespace base::detail
