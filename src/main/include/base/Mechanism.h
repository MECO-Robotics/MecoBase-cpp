#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

namespace base
{
enum class Mode
{
    Real,
    Sim,
    Replay
};
enum class Vendor
{
    TalonFX,
    SparkMax
};
enum class Gravity
{
    Constant,
    Cosine,
    Sine
};
struct PhoenixControlConfig
{
    // Legacy compatibility flag; real Talon control is always onboard.
    bool enabled = false;
    enum class Feedback
    {
        Rotor,
        RemoteCANcoder
    };
    Feedback feedback = Feedback::Rotor;
    int remoteSensorId = 0;
    double rotorToSensorRatio = 1, sensorToMechanismRatio = 1;
    bool sensorReversed = false;
};
struct SparkControlConfig
{
    enum class Feedback
    {
        PrimaryEncoder,
        AbsoluteEncoder
    };
    Feedback feedback = Feedback::PrimaryEncoder;
    // Absolute sensor rotations per output rotation, with zero in sensor rotations.
    double sensorToMechanismRatio = 1, zeroOffset = 0;
    bool sensorReversed = false;
};
struct MotorConfig
{
    std::vector<int> ids;
    // Primary inversion first; subsequent true flags oppose the primary motor.
    std::vector<bool> reversed;
    double gearRatio = 1;
    int currentLimit = 40;
    std::string canBus;
    PhoenixControlConfig phoenix;
    SparkControlConfig spark;
    void Validate() const;
};
struct Gains
{
    double p = 0, i = 0, d = 0, s = 0, v = 0, a = 0, gravity = 0;
    double maxVelocity = 1, maxAcceleration = 1, tolerance = .01;
    double minimum = 0, maximum = 1, defaultPosition = 0;
    void Validate(bool position) const;
};
struct EncoderCalibration
{
    double unitsPerRotation = 1, offsetRotations = 0;
    bool reversed = false;
    double Apply(double raw) const;
    void Validate() const;
};
enum class MotorConfigurationState
{
    Ready,
    Pending,
    Fault
};
enum class MotorConfigurationFailure
{
    None,
    Gains,
    Brake,
    Coast
};
struct MotorInputs
{
    double position = 0, velocity = 0, voltage = 0, supplyVoltage = 12, current = 0;
    // Adapters and replay injectors must affirm healthy feedback before control.
    bool connected = false;
    MotorConfigurationState configuration = MotorConfigurationState::Ready;
    MotorConfigurationFailure configurationFailure = MotorConfigurationFailure::None;
};
// All positions below are output-shaft angles; elevator travel is angle * drumRadius.
struct MotorSimulationConfig
{
    enum class Load
    {
        Free,
        Arm,
        Elevator
    };
    Load load = Load::Free;
    // Arm inertia includes its payload. Elevator adds mass * drumRadius squared.
    units::kilogram_square_meter_t outputInertia{.025};
    units::kilogram_t mass{0};
    units::meter_t armCOM{0}, drumRadius{0};
    // Arm angle zero is horizontal; offset maps the output sensor zero to that frame.
    units::radian_t armAngleOffset{0}, initialPosition{0};
    std::optional<units::radian_t> minimum, maximum;
    void Validate() const;
};
struct NativeControlRequest
{
    enum class Kind
    {
        Velocity,
        Position,
        MotionMagic
    };
    Kind kind = Kind::Velocity;
    double target = 0, maxVelocity = 1, maxAcceleration = 1;
    double velocity = 0, acceleration = 0;
};
struct MotorControlConfig
{
    Gains gains;
    double unitsPerOutputRotation = 1;
    Gravity gravity = Gravity::Constant;
    bool positionControlled = false, externalFeedback = false;
};
struct MotorControlRequest
{
    NativeControlRequest native;
    // Software trajectory reference used by sim/replay. Native profilers use the goal.
    double profiledPosition = 0;
};
struct MotorControlResult
{
    bool ok = false;
    std::optional<double> voltage; // Absent for onboard firmware control.
};
class MotorIO
{
  public:
    virtual ~MotorIO() = default;
    virtual MotorInputs Read() = 0;
    virtual void SetVoltage(double volts) = 0;
    virtual void SetBrake(bool brake) = 0;
    virtual bool UsesAsyncConfiguration() const
    {
        return false;
    }
    virtual MotorConfigurationState ConfigurationState() const
    {
        return MotorConfigurationState::Ready;
    }
    // Adapters own all PID/feedforward state and vendor conversion. Configuration
    // returns whether firmware owns the loop; false selects the IO-owned emulator.
    bool ConfigureControl(const MotorControlConfig &);
    MotorControlResult Control(const MotorControlRequest &, const MotorInputs &, double physicalPosition,
                               units::second_t dt);
    void ResetControl();
    // Only replay calls this, using the source frame rather than current hardware.
    virtual void UseRecordedControl(bool native)
    {
        controlNative_ = native;
    }
    // Native gains/requests use mechanism units; adapters own vendor conversion.
    virtual bool ConfigureNative(const Gains &, double, Gravity, bool)
    {
        return false;
    }
    virtual bool SetNative(const NativeControlRequest &)
    {
        return false;
    }
    // Explicit plant advancement (0..1 s); real/replay adapters remain observational.
    virtual void StepSimulation(units::second_t)
    {
    }
    // Offline compatibility entry point; callers must retain one physics owner.
    void AdvanceSimulation(units::second_t dt)
    {
        StepSimulation(dt);
    }

  private:
    MotorControlConfig controlConfig_;
    bool controlNative_ = false;
    double integral_ = 0, previousError_ = 0;
};
// Real adapters wrap blocking configuration in a prestarted, bounded worker.
// The first gain configuration is startup-only and synchronous.
std::unique_ptr<MotorIO> MakeAsyncMotorIO(std::unique_ptr<MotorIO> child);
// Factories are lazy: replay never constructs a controller or simulator.
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config, double outputInertia = .025);
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config,
                                     const MotorSimulationConfig &simulation);
class InjectedMotorIO final : public MotorIO
{
  public:
    MotorInputs inputs;
    double requestedVoltage = 0;
    bool brake = true, nativeEnabled = false;
    std::optional<NativeControlRequest> nativeRequest;
    bool ConfigureNative(const Gains &, double, Gravity, bool) override
    {
        return nativeEnabled;
    }
    bool SetNative(const NativeControlRequest &request) override
    {
        nativeRequest = request;
        return nativeEnabled;
    }
    void UseRecordedControl(bool native) override
    {
        nativeEnabled = native;
        MotorIO::UseRecordedControl(native);
    }
    MotorInputs Read() override
    {
        return inputs;
    }
    void SetVoltage(double volts) override
    {
        requestedVoltage = volts;
        nativeRequest.reset();
    }
    void SetBrake(bool enabled) override
    {
        brake = enabled;
    }
};
// Mechanism positions are rotations, or metres when unitsPerOutputRotation is a drum circumference.
class Mechanism : public frc2::SubsystemBase
{
  public:
    Mechanism(std::string name, std::unique_ptr<MotorIO> io, Gains gains, bool positionControlled,
              double unitsPerOutputRotation = 1, Gravity gravity = Gravity::Constant,
              std::function<double()> externalEncoder = {}, EncoderCalibration calibration = {});
    void Periodic() override;
    // Control, accessors, and telemetry must share one thread. Offline ticks require 0 < dt <= 100 ms.
    // Never enables a disabled DS; external scheduling only suppresses default control.
    // The shared world remains the sole physics owner.
    void Control(units::second_t dt, bool enabled = true);
    void UseExternalScheduling(bool enabled = true)
    {
        externalScheduling_ = enabled;
    }
    // Observational snapshot only; does not read sensors or advance control/physics.
    void PublishTelemetry() const;
    void SetVelocity(double velocity);
    void SetPosition(double position);
    // Holds a supplied target until a different request, stop, fault, or a new
    // command owner replaces it. The supplier runs on the control thread,
    // must not mutate this mechanism, and captured objects must outlive it.
    void SetLatchedVelocity(std::function<double()> velocity);
    void SetLatchedPosition(std::function<double()> position);
    // Native PositionVoltage when supported; ordinary SetPosition uses Motion Magic.
    void SetPositionUnprofiled(double position);
    void SetPositionDynamic(double position, double maxVelocity, double maxAcceleration);
    void SetVoltage(double volts);
    void Stop();
    void ResetPosition();
    void SetBrake(bool enabled);
    void SetGains(Gains gains);
    const Gains &GetGains() const
    {
        return gains_;
    }
    MotorConfigurationState ConfigurationState() const
    {
        return configurationIntent_ ? MotorConfigurationState::Pending : inputs_.configuration;
    }
    bool IsPositionControlled() const
    {
        return joint_;
    }
    bool AtSetpoint() const;
    bool VelocityControlActive() const
    {
        return control_ == ControlMode::Velocity;
    }
    double VelocitySetpoint() const
    {
        return target_;
    }
    double Position() const
    {
        return physicalPosition_ - zeroOffset_;
    }
    double Velocity() const
    {
        return inputs_.velocity * units_;
    }
    double DesiredPosition() const
    {
        return target_ - zeroOffset_;
    }
    double PositionOffset() const
    {
        return zeroOffset_;
    }
    const MotorInputs &Inputs() const
    {
        return inputs_;
    }
    frc2::CommandPtr VelocityCommand(std::function<double()> velocity);
    frc2::CommandPtr PositionCommand(std::function<double()> position);
    frc2::CommandPtr VoltageCommand(std::function<double()> voltage);

  private:
    void ApplyVoltage(double volts);
    bool AcceptIntent();
    enum class ControlMode
    {
        Stopped,
        Voltage,
        Velocity,
        Position
    };
    std::unique_ptr<MotorIO> io_;
    Gains gains_;
    bool externalScheduling_ = false, configurationIntent_ = false, brake_ = true, asynchronousConfiguration_ = false;
    bool joint_, native_ = false, nativeFault_ = false, motionMagic_ = true, unprofiled_ = false;
    double units_, physicalPosition_ = 0, zeroOffset_ = 0, target_ = 0, voltage_ = 0;
    double profiledPosition_ = 0, profiledVelocity_ = 0;
    double maxVelocity_, maxAcceleration_;
    Gravity gravity_;
    std::function<double()> externalEncoder_;
    EncoderCalibration calibration_;
    MotorInputs inputs_;
    ControlMode control_ = ControlMode::Stopped;
    std::function<double()> latchedTarget_;
    frc2::Command *latchedOwner_ = nullptr;
    bool updatingLatchedTarget_ = false;
};
class Flywheel final : public Mechanism
{
  public:
    Flywheel(std::string name, std::unique_ptr<MotorIO> io, Gains gains)
        : Mechanism(std::move(name), std::move(io), gains, false)
    {
    }
};
class PositionJoint final : public Mechanism
{
  public:
    PositionJoint(std::string name, std::unique_ptr<MotorIO> io, Gains gains, double unitsPerOutputRotation = 1,
                  Gravity gravity = Gravity::Constant, std::function<double()> encoder = {},
                  EncoderCalibration calibration = {})
        : Mechanism(std::move(name), std::move(io), gains, true, unitsPerOutputRotation, gravity, std::move(encoder),
                    calibration)
    {
    }
};
class DigitalSensor
{
  public:
    DigitalSensor(int channel, bool inverted = false);
    bool Get() const;

  private:
    std::unique_ptr<frc::DigitalInput> input_;
    int channel_;
    bool inverted_;
};
} // namespace base
