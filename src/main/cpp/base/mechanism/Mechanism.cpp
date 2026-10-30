#include "base/Mechanism.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <limits>
#include <algorithm>
#include <numbers>
#include <set>
#include <stdexcept>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>

namespace base
{
namespace
{
wpi::json Number(double value)
{
    return std::isfinite(value) ? wpi::json(value) : wpi::json(nullptr);
}
double Number(const wpi::json &value)
{
    return value.is_null() ? std::numeric_limits<double>::quiet_NaN() : value.get<double>();
}
void Finite(double value)
{
    if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite mechanism value");
}
} // namespace
void MotorConfig::Validate() const
{
    if (ids.empty() || ids.size() != reversed.size() || gearRatio <= 0 || currentLimit <= 0)
        throw std::invalid_argument("Invalid motor configuration");
    Finite(gearRatio);
    if ((phoenix.remoteSensorId < 0 || phoenix.remoteSensorId > 62 || !std::isfinite(phoenix.rotorToSensorRatio) ||
         phoenix.rotorToSensorRatio <= 0 || !std::isfinite(phoenix.sensorToMechanismRatio) ||
         phoenix.sensorToMechanismRatio <= 0))
        throw std::invalid_argument("Invalid Phoenix feedback configuration");
    if (!std::isfinite(spark.sensorToMechanismRatio) || spark.sensorToMechanismRatio <= 0 ||
        !std::isfinite(spark.zeroOffset) || spark.zeroOffset < 0 || spark.zeroOffset >= 1 ||
        (spark.feedback != SparkControlConfig::Feedback::PrimaryEncoder &&
         spark.feedback != SparkControlConfig::Feedback::AbsoluteEncoder))
        throw std::invalid_argument("Invalid Spark feedback configuration");
    std::set<int> seen;
    for (int id : ids)
        if (id < 0 || id > 62 || !seen.insert(id).second)
            throw std::invalid_argument("Invalid or duplicate CAN ID");
}
void Gains::Validate(bool position) const
{
    for (double value :
         {p, i, d, s, v, a, gravity, maxVelocity, maxAcceleration, tolerance, minimum, maximum, defaultPosition})
        Finite(value);
    if (maxVelocity <= 0 || maxAcceleration <= 0 || tolerance <= 0 ||
        (position && (minimum >= maximum || defaultPosition < minimum || defaultPosition > maximum)))
        throw std::invalid_argument("Invalid mechanism gains or limits");
}
void EncoderCalibration::Validate() const
{
    Finite(unitsPerRotation);
    Finite(offsetRotations);
    if (unitsPerRotation <= 0)
        throw std::invalid_argument("Invalid encoder scale");
}
double EncoderCalibration::Apply(double raw) const
{
    Validate();
    Finite(raw);
    return ((reversed ? -raw : raw) + offsetRotations) * unitsPerRotation;
}
Mechanism::Mechanism(std::string name, std::unique_ptr<MotorIO> io, Gains gains, bool positionControlled,
                     double unitsPerOutputRotation, Gravity gravity, std::function<double()> externalEncoder,
                     EncoderCalibration calibration)
    : io_(std::move(io)), gains_(gains), joint_(positionControlled), units_(unitsPerOutputRotation),
      maxVelocity_(gains.maxVelocity), maxAcceleration_(gains.maxAcceleration), gravity_(gravity),
      externalEncoder_(std::move(externalEncoder)), calibration_(calibration)
{
    if (!io_ || !std::isfinite(units_) || units_ <= 0)
        throw std::invalid_argument("Invalid mechanism IO or units");
    gains_.Validate(joint_);
    calibration_.Validate();
    native_ = io_->ConfigureControl({gains_, units_, gravity_, joint_, static_cast<bool>(externalEncoder_)});
    if (native_ && externalEncoder_)
        throw std::invalid_argument(
            "Native control requires the motor adapter feedback, not an external encoder callback");
    asynchronousConfiguration_ = io_->UsesAsyncConfiguration();
    SetName(name);
}
void Mechanism::SetGains(Gains gains)
{
    gains.Validate(joint_);
    if (configurationIntent_)
        throw std::logic_error("Motor configuration awaits a control-tick acknowledgement");
    if (inputs_.configuration == MotorConfigurationState::Fault &&
        inputs_.configurationFailure != MotorConfigurationFailure::Gains)
        throw std::logic_error("Retry the failed neutral configuration first");
    if (native_ && !asynchronousConfiguration_ && frc::DriverStation::IsEnabled())
        throw std::logic_error("Synchronous native gain configuration requires disabled state");
    if (native_)
    {
        Stop();
        const bool previousFault = nativeFault_;
        nativeFault_ = true;
        try
        {
            if (!io_->ConfigureControl({gains, units_, gravity_, joint_, static_cast<bool>(externalEncoder_)}))
                throw std::runtime_error("Native controller reconfiguration lost capability");
        }
        catch (...)
        {
            // A rejected overlapping submission does not change the accepted job.
            if (asynchronousConfiguration_)
                nativeFault_ = previousFault;
            throw;
        }
        nativeFault_ = false;
    }
    if (!native_)
        io_->ConfigureControl({gains, units_, gravity_, joint_, static_cast<bool>(externalEncoder_)});
    configurationIntent_ = native_ && asynchronousConfiguration_;
    gains_ = gains;
    maxVelocity_ = gains.maxVelocity;
    maxAcceleration_ = gains.maxAcceleration;
}
bool Mechanism::AcceptIntent()
{
    if (configurationIntent_ || inputs_.configuration != MotorConfigurationState::Ready)
    {
        Stop();
        return false;
    }
    return true;
}
void Mechanism::SetVelocity(double velocity)
{
    if (!updatingLatchedTarget_) {latchedTarget_={};latchedOwner_=nullptr;}
    Finite(velocity);
    if (!AcceptIntent())
        return;
    unprofiled_ = false;
    if (control_ != ControlMode::Velocity)
    {
        profiledVelocity_ = Velocity();
        io_->ResetControl();
    }
    maxAcceleration_ = gains_.maxAcceleration;
    target_ = joint_ ? std::clamp(velocity, -gains_.maxVelocity, gains_.maxVelocity) : velocity;
    control_ = ControlMode::Velocity;
    replay::RecordOutput("Mechanism/" + GetName() + "/Request", {{"kind", "velocity"}, {"value", target_}});
}
void Mechanism::SetLatchedVelocity(std::function<double()> velocity)
{
    if (!velocity) {Stop();throw std::invalid_argument("Live velocity requires a supplier");}
    try {SetVelocity(velocity());} catch (...) {Stop();throw;}
    if (control_==ControlMode::Velocity && ConfigurationState()==MotorConfigurationState::Ready) {
        latchedOwner_=frc2::CommandScheduler::GetInstance().Requiring(this);
        latchedTarget_=std::move(velocity);
    }
}
void Mechanism::SetLatchedPosition(std::function<double()> position)
{
    if (!position) {Stop();throw std::invalid_argument("Live position requires a supplier");}
    try {SetPosition(position());} catch (...) {Stop();throw;}
    if (control_==ControlMode::Position && ConfigurationState()==MotorConfigurationState::Ready) {
        latchedOwner_=frc2::CommandScheduler::GetInstance().Requiring(this);
        latchedTarget_=std::move(position);
    }
}
void Mechanism::SetPosition(double position)
{
    SetPositionDynamic(position, gains_.maxVelocity, gains_.maxAcceleration);
    motionMagic_ = true;
}
void Mechanism::SetPositionUnprofiled(double position)
{
    SetPosition(position);
    motionMagic_ = false;
    unprofiled_ = true;
}
void Mechanism::SetPositionDynamic(double position, double maxVelocity, double maxAcceleration)
{
    if (!updatingLatchedTarget_) {latchedTarget_={};latchedOwner_=nullptr;}
    if (!AcceptIntent())
        return;
    const bool enterSoftwareProfile = native_ && (motionMagic_ || unprofiled_);
    Finite(position);
    Finite(maxVelocity);
    Finite(maxAcceleration);
    if (!joint_ || maxVelocity <= 0 || maxAcceleration <= 0)
        throw std::invalid_argument("Invalid joint request");
    motionMagic_ = false;
    unprofiled_ = false;
    if (control_ != ControlMode::Position || enterSoftwareProfile)
    {
        profiledPosition_ = physicalPosition_;
        profiledVelocity_ = Velocity();
        io_->ResetControl();
    }
    target_ = std::clamp(position + zeroOffset_, gains_.minimum, gains_.maximum);
    maxVelocity_ = maxVelocity;
    maxAcceleration_ = maxAcceleration;
    control_ = ControlMode::Position;
    replay::RecordOutput("Mechanism/" + GetName() + "/Request", {{"kind", "position"}, {"value", target_}});
}
void Mechanism::SetVoltage(double volts)
{
    if (!updatingLatchedTarget_) {latchedTarget_={};latchedOwner_=nullptr;}
    Finite(volts);
    if (!AcceptIntent())
        return;
    voltage_ = std::clamp(volts, -12., 12.);
    control_ = ControlMode::Voltage;
    replay::RecordOutput("Mechanism/" + GetName() + "/Request", {{"kind", "voltage"}, {"value", voltage_}});
}
void Mechanism::Stop()
{
    latchedTarget_={};latchedOwner_=nullptr;
    control_ = ControlMode::Stopped;
    io_->ResetControl();
    ApplyVoltage(0);
}
void Mechanism::SetBrake(bool enabled)
{
    if (configurationIntent_)
        throw std::logic_error("Motor configuration awaits a control-tick acknowledgement");
    if (inputs_.configuration == MotorConfigurationState::Fault &&
        inputs_.configurationFailure != (enabled ? MotorConfigurationFailure::Brake : MotorConfigurationFailure::Coast))
        throw std::logic_error("Retry the failed configuration operation and neutral target");
    if (native_ && (brake_ != enabled || inputs_.configuration == MotorConfigurationState::Fault) &&
        frc::DriverStation::IsEnabled())
        throw std::logic_error("Neutral mode configuration requires disabled state");
    const bool configuring = native_ && asynchronousConfiguration_ &&
                             (brake_ != enabled || inputs_.configuration == MotorConfigurationState::Fault);
    if (configuring)
        Stop();
    io_->SetBrake(enabled);
    if (configuring)
        configurationIntent_ = true;
    brake_ = enabled;
    replay::RecordOutput("Mechanism/" + GetName() + "/Brake", enabled);
}
void Mechanism::ApplyVoltage(double volts)
{
    io_->SetVoltage(volts);
    replay::RecordOutput("Mechanism/" + GetName() + "/AppliedVoltageRequest", volts);
}
void Mechanism::PublishTelemetry() const
{
    const std::string name = GetName();
    const bool connected = inputs_.connected;
    const double position = Position(), velocity = Velocity(), target = joint_ ? DesiredPosition() : target_;
    telemetry::Post("Mechanism/" + name, [name, connected, position, velocity, target] {
        const auto prefix = name + "/";
        frc::SmartDashboard::PutBoolean(prefix + "Connected", connected);
        frc::SmartDashboard::PutNumber(prefix + "Position", position);
        frc::SmartDashboard::PutNumber(prefix + "Velocity", velocity);
        frc::SmartDashboard::PutNumber(prefix + "Target", target);
    });
}
void Mechanism::ResetPosition()
{
    if (!inputs_.connected || !std::isfinite(physicalPosition_) || !std::isfinite(Velocity()))
    {
        Stop();
        return;
    }
    zeroOffset_ = physicalPosition_;
    target_ = physicalPosition_;
    profiledPosition_ = physicalPosition_;
    profiledVelocity_ = 0;
    io_->ResetControl();
    SetBrake(true);
    Stop();
    if (joint_)
        SetPosition(0);
}
bool Mechanism::AtSetpoint() const
{
    if (!inputs_.connected || !std::isfinite(physicalPosition_) || !std::isfinite(Velocity()) ||
        (control_ != ControlMode::Position && control_ != ControlMode::Velocity))
        return false;
    return std::abs((control_ == ControlMode::Position ? physicalPosition_ : Velocity()) - target_) <= gains_.tolerance;
}
void Mechanism::Periodic()
{
    if (!externalScheduling_)
        Control(units::second_t{.02});
    PublishTelemetry();
}
void Mechanism::Control(units::second_t timestep, bool enabled)
{
    const double dt = timestep.value();
    if (!std::isfinite(dt) || dt <= 0 || dt > .1)
    {
        Stop();
        throw std::invalid_argument("Invalid mechanism control timestep");
    }
    const auto input = replay::ProcessInputs("Mechanism/" + GetName(), [this] {
        const auto value = io_->Read();
        return wpi::json{{"position", Number(value.position)},
                         {"velocity", Number(value.velocity)},
                         {"voltage", Number(value.voltage)},
                         {"supplyVoltage", Number(value.supplyVoltage)},
                         {"current", Number(value.current)},
                         {"connected", value.connected},
                         {"nativeControl", native_},
                         {"configuration", static_cast<int>(value.configuration)},
                         {"configurationFailure", static_cast<int>(value.configurationFailure)},
                         {"asynchronousConfiguration", asynchronousConfiguration_},
                         {"externalEncoder", externalEncoder_ ? Number(externalEncoder_()) : wpi::json(nullptr)}};
    });
    if (replay::Active())
    {
        native_ = input.value("nativeControl", false);
        asynchronousConfiguration_ = input.value("asynchronousConfiguration", false);
        io_->UseRecordedControl(native_);
    }
    const int configuration = input.value("configuration", 0);
    if (configuration < 0 || configuration > 2)
    {
        Stop();
        throw std::runtime_error("Invalid recorded motor configuration state");
    }
    const int failure = input.value("configurationFailure", 0);
    if (failure < 0 || failure > 3)
    {
        Stop();
        throw std::runtime_error("Invalid recorded motor configuration failure");
    }
    inputs_ = {Number(input.at("position")),
               Number(input.at("velocity")),
               Number(input.at("voltage")),
               Number(input.at("supplyVoltage")),
               Number(input.at("current")),
               input.at("connected").get<bool>(),
               static_cast<MotorConfigurationState>(configuration),
               static_cast<MotorConfigurationFailure>(failure)};
    configurationIntent_ = inputs_.configuration == MotorConfigurationState::Pending;
    if (inputs_.configuration != MotorConfigurationState::Ready)
    {
        Stop();
        return;
    }
    if (!inputs_.connected || nativeFault_)
    {
        Stop();
        return;
    }
    if (externalEncoder_)
    {
        const double raw = Number(input.at("externalEncoder"));
        if (!std::isfinite(raw))
        {
            physicalPosition_ = raw;
            Stop();
            return;
        }
        physicalPosition_ = calibration_.Apply(raw);
    }
    else
        physicalPosition_ = inputs_.position * units_;
    if (!std::isfinite(physicalPosition_) || !std::isfinite(Velocity()))
    {
        Stop();
        return;
    }
    if (latchedTarget_)
    {
        const auto owner=frc2::CommandScheduler::GetInstance().Requiring(this);
        if (owner && owner!=latchedOwner_) {latchedTarget_={};latchedOwner_=nullptr;}
        else {
            try {
                const double target=latchedTarget_();
                updatingLatchedTarget_=true;
                if (control_==ControlMode::Position) SetPosition(target);
                else if (control_==ControlMode::Velocity) SetVelocity(target);
                updatingLatchedTarget_=false;
            } catch (...) {
                updatingLatchedTarget_=false;
                Stop();
                return;
            }
        }
    }
    if (!enabled || frc::DriverStation::IsDisabled())
    {
        io_->ResetControl();
        profiledPosition_ = physicalPosition_;
        profiledVelocity_ = Velocity();
        ApplyVoltage(0);
        return;
    }
    double volts = 0, profileAcceleration = 0;
    if (control_ == ControlMode::Voltage)
        volts = voltage_;
    else if (control_ == ControlMode::Velocity || control_ == ControlMode::Position)
    {
        double previousVelocity = profiledVelocity_;
        if (control_ == ControlMode::Position)
        {
            double distance = target_ - profiledPosition_;
            double desired =
                std::copysign(std::min(maxVelocity_, std::sqrt(2 * maxAcceleration_ * std::abs(distance))), distance);
            profiledVelocity_ += std::clamp(desired - profiledVelocity_, -maxAcceleration_ * dt, maxAcceleration_ * dt);
            double step = profiledVelocity_ * dt;
            // A close goal may require overshoot: snapping a moving profile to rest
            // must not violate the acceleration bound or spike acceleration feedforward.
            if (step * distance >= 0 && std::abs(step) >= std::abs(distance) &&
                std::abs(previousVelocity) <= maxAcceleration_ * dt)
            {
                profiledPosition_ = target_;
                profiledVelocity_ = 0;
            }
            else
                profiledPosition_ += step;
        }
        else
            profiledVelocity_ += std::clamp(target_ - profiledVelocity_, -maxAcceleration_ * dt, maxAcceleration_ * dt);
        profileAcceleration = (profiledVelocity_ - previousVelocity) / dt;
    }
    if (control_ == ControlMode::Position || control_ == ControlMode::Velocity)
    {
        const bool outward = joint_ && control_ == ControlMode::Velocity &&
                             ((physicalPosition_ <= gains_.minimum && target_ < 0) ||
                              (physicalPosition_ >= gains_.maximum && target_ > 0));
        if (outward)
        {
            ApplyVoltage(0);
            return;
        }
        const NativeControlRequest request{control_ == ControlMode::Velocity ? NativeControlRequest::Kind::Velocity
                                           : motionMagic_                    ? NativeControlRequest::Kind::MotionMagic
                                                                             : NativeControlRequest::Kind::Position,
                                           control_ == ControlMode::Velocity ? profiledVelocity_
                                           : motionMagic_ || unprofiled_     ? target_
                                                                             : profiledPosition_,
                                           maxVelocity_,
                                           maxAcceleration_,
                                           unprofiled_ ? 0 : profiledVelocity_,
                                           unprofiled_ ? 0 : profileAcceleration};
        if (!std::isfinite(request.target) || !std::isfinite(request.velocity) || !std::isfinite(request.acceleration))
        {
            Stop();
            return;
        }
        if (native_)
            replay::RecordOutput("Mechanism/" + GetName() + "/NativeRequest",
                                 {{"kind", static_cast<int>(request.kind)},
                                  {"target", request.target},
                                  {"maxVelocity", request.maxVelocity},
                                  {"maxAcceleration", request.maxAcceleration},
                                  {"velocity", request.velocity},
                                  {"acceleration", request.acceleration}});
        const auto result = io_->Control({request, profiledPosition_}, inputs_, physicalPosition_, timestep);
        if (result.voltage)
            replay::RecordOutput("Mechanism/" + GetName() + "/AppliedVoltageRequest", *result.voltage);
        if (!result.ok)
        {
            nativeFault_ = native_;
            Stop();
        }
        return;
    }
    if (!std::isfinite(volts))
    {
        Stop();
        return;
    }
    if (joint_ &&
        ((physicalPosition_ <= gains_.minimum && volts < 0) || (physicalPosition_ >= gains_.maximum && volts > 0)))
        volts = 0;
    ApplyVoltage(std::clamp(volts, -12., 12.));
}
DigitalSensor::DigitalSensor(int channel, bool inverted) : channel_(channel), inverted_(inverted)
{
    if (!replay::Active())
        input_ = std::make_unique<frc::DigitalInput>(channel);
}
bool DigitalSensor::Get() const
{
    const auto input =
        replay::ProcessInputs("Digital/" + std::to_string(channel_), [this] { return wpi::json(input_->Get()); });
    return input.get<bool>() != inverted_;
}
frc2::CommandPtr Mechanism::VelocityCommand(std::function<double()> supplier)
{
    return frc2::cmd::RunEnd([this, supplier] { SetVelocity(supplier()); }, [this] { Stop(); }, {this});
}
frc2::CommandPtr Mechanism::PositionCommand(std::function<double()> supplier)
{
    return frc2::cmd::Run([this, supplier] { SetPosition(supplier()); }, {this}).Until([this] { return AtSetpoint(); });
}
frc2::CommandPtr Mechanism::VoltageCommand(std::function<double()> supplier)
{
    return frc2::cmd::RunEnd([this, supplier] { SetVoltage(supplier()); }, [this] { Stop(); }, {this});
}
} // namespace base
