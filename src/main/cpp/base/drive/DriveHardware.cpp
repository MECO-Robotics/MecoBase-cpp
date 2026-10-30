#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include "base/detail/BackgroundPriority.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cstdio>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#ifndef __FRC_ROBORIO__
#include "base/SimulationRuntime.h"
#include <chrono>
#include <iostream>
#endif
#include <ctre/phoenix6/Utils.hpp>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
using namespace units::literals;
namespace p = ctre::phoenix6;
namespace s = p::swerve;
namespace base
{
struct Drive::ReplayKinematics
{
    static frc::Translation2d Location(const DriveConfig &config, size_t i)
    {
        return {units::meter_t{config.modules[i].xMeters}, units::meter_t{config.modules[i].yMeters}};
    }
    explicit ReplayKinematics(const DriveConfig &config)
        : value(Location(config, 0), Location(config, 1), Location(config, 2), Location(config, 3))
    {
    }
    frc::SwerveDriveKinematics<4> value;
};

struct Drive::MotorTuningWorker
{
    NativeDrive *drive = nullptr;
    PortableSwerve *portable = nullptr;
    std::mutex mutex;
    std::condition_variable wake;
    std::atomic<DriveGainState> state{DriveGainState::Ready};
    DriveMotorGains job{};
    bool ready = false, quit = false;
    std::thread thread;
    explicit MotorTuningWorker(NativeDrive &native) : drive(&native), thread([this] { Work(); }) {}
    explicit MotorTuningWorker(PortableSwerve &swerve) : portable(&swerve), thread([this] { Work(); }) {}
    ~MotorTuningWorker()
    {
        { std::lock_guard lock(mutex); quit = true; }
        wake.notify_one();
        thread.join();
    }
    bool Submit(const DriveMotorGains &next)
    {
        std::unique_lock lock(mutex, std::try_to_lock);
        if (!lock.owns_lock() || state == DriveGainState::Pending) return false;
        const auto previous=state.load();
        state = DriveGainState::Pending;
        try
        {
            if (portable)
            {
                if (!portable->InhibitForConfiguration()) {state=previous;return false;}
            }
            else
            {
                // Idle prevents Phoenix's swerve thread from reissuing old goals.
                drive->SetControl(s::requests::Idle{});
                for (size_t i=0; i<4; ++i)
                {
                    drive->GetModule(i).GetDriveMotor().SetVoltage(0_V);
                    drive->GetModule(i).GetSteerMotor().SetVoltage(0_V);
                }
            }
        }
        catch (...)
        {
            // Try every controller even if stopping one device throws.
            if (drive) for (size_t i=0; i<4; ++i)
            {
                try { drive->GetModule(i).GetDriveMotor().SetVoltage(0_V); } catch (...) {}
                try { drive->GetModule(i).GetSteerMotor().SetVoltage(0_V); } catch (...) {}
            }
            state = DriveGainState::Fault;
            throw;
        }
        job = next;
        ready = true;
        wake.notify_one();
        return true;
    }
    void Work()
    {
        if (!detail::BackgroundPriority("Swerve-Config"))
            std::fputs("Swerve configuration worker priority could not be lowered\n", stderr);
        for (;;)
        {
            DriveMotorGains next;
            {
                std::unique_lock lock(mutex);
                wake.wait(lock, [this] { return ready || quit; });
                if (quit) return;
                next = job;
                ready = false;
            }
            p::configs::Slot0Configs driveSlot, steerSlot;
            driveSlot.kP = next.driveP; driveSlot.kI = next.driveI; driveSlot.kD = next.driveD; driveSlot.kS = next.driveS; driveSlot.kV = next.driveV; driveSlot.kA = next.driveA;
            steerSlot.kP = next.steerP; steerSlot.kI = next.steerI; steerSlot.kD = next.steerD; steerSlot.kS = next.steerS; steerSlot.kV = next.steerV; steerSlot.kA = next.steerA;
            bool ok = true;
            try
            {
                if (portable)
                {
                    ok=portable->ConfigureGains({next.driveP,next.driveD,next.driveS,next.driveV,next.steerP,next.steerD});
                }
                else for (size_t i=0; i<4; ++i)
                {
                    ok = drive->GetModule(i).GetDriveMotor().GetConfigurator().Apply(driveSlot, .1_s).IsOK() && ok;
                    ok = drive->GetModule(i).GetSteerMotor().GetConfigurator().Apply(steerSlot, .1_s).IsOK() && ok;
                }
            }
            catch (...) { ok = false; }
            state = ok ? DriveGainState::Ready : DriveGainState::Fault;
        }
    }
};

struct Drive::Telemetry
{
    nt::StructArrayPublisher<frc::SwerveModuleState> measured =
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/ModuleStates")
            .Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> targets =
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/ModuleTargets")
            .Publish();
    nt::StructPublisher<frc::Pose2d> pose =
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/AdvantageScope/Drive/Pose").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> speeds =
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/ChassisSpeeds")
            .Publish();
    nt::StructPublisher<frc::ChassisSpeeds> requested =
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/RequestedChassisSpeeds")
            .Publish();
};
#ifndef __FRC_ROBORIO__
namespace
{
Drive *simulationClockOwner{};

void AwaitPhoenixSimulationTransport(const DriveConfig &config)
{
    // Probe with the native Apply/Refresh transport; status can precede configuration readiness.
    // Destroy this temporary handle before NativeDrive constructs its devices.
    p::hardware::TalonFX probe{config.modules[0].drive, p::CANBus{config.bus}};
    p::configs::MotorOutputConfigs requested;
    requested.PeakForwardDutyCycle = 0;
    requested.PeakReverseDutyCycle = 0;
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::seconds(10);
    std::string status;
    int attempts = 0;
    do
    {
        ++attempts;
        auto applied = probe.GetConfigurator().Apply(requested, .1_s);
        status = applied.GetName();
        if (applied.IsOK())
        {
            p::configs::MotorOutputConfigs actual;
            auto refreshed = probe.GetConfigurator().Refresh(actual, .1_s);
            status = refreshed.GetName();
            if (refreshed.IsOK() && actual.PeakForwardDutyCycle == requested.PeakForwardDutyCycle &&
                actual.PeakReverseDutyCycle == requested.PeakReverseDutyCycle)
            {
                std::cout << "[DriveSim] Verified Phoenix configuration transport after "
                          << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << " s, "
                          << attempts << " attempt(s)" << std::endl;
                // Native SwerveModule applies the full DriveMotorInitialConfigs;
                // its default +/-1 duty limits replace these temporary zero limits.
                return;
            }
        }
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("Phoenix desktop simulation configuration transport did not complete verified "
                             "Apply/Refresh within 10 seconds for drive device " +
                             std::to_string(config.modules[0].drive) + " on bus " + config.bus + ": " + status);
}
} // namespace

#endif
void DriveConfig::Validate() const
{
    if (backend != DriveBackend::Phoenix && backend != DriveBackend::SparkMaxNavX)
        throw std::invalid_argument("Unsupported drivetrain backend");
    if (backend == DriveBackend::SparkMaxNavX && odometryHz != 0 && (odometryHz < 20 || odometryHz > 200))
        throw std::invalid_argument("Portable acquisition requires 20 to 200 Hz");
    auto finite = [](double value) {
        if (!std::isfinite(value))
            throw std::invalid_argument("DriveConfig requires finite values");
    };
    auto positive = [&](double value) {
        finite(value);
        if (value <= 0)
            throw std::invalid_argument("DriveConfig requires positive physical dimensions, ratios and limits");
    };
    auto nonnegative = [&](double value) {
        finite(value);
        if (value < 0)
            throw std::invalid_argument("DriveConfig requires nonnegative gains, friction and damping");
    };
    for (double v : {driveRatio,
                     steerRatio,
                     wheelRadius,
                     maxSpeed,
                     maxAngularRate,
                     driveCurrent,
                     steerCurrent,
                     slipCurrent,
                     massKg,
                     moi,
                     simulation.bumperHalfLength,
                     simulation.bumperHalfWidth,
                     simulation.bumperHalfHeight,
                     simulation.rideHeight,
                     simulation.rollInertia,
                     simulation.pitchInertia,
                     simulation.wheelInertia,
                     simulation.steerInertia,
                     simulation.longitudinalStiffness,
                     simulation.lateralStiffness,
                     simulation.suspensionRestLength,
                     simulation.suspensionTravel,
                     simulation.suspensionStiffness,
                     simulation.suspensionDamping})
        positive(v);
    for (double v : {spark.driveP, spark.driveD, spark.driveKS, spark.driveKV, spark.steerP, spark.steerD, odometryHz,
                     autoTranslationKP, autoRotationKP, cof, driveKP, driveKD, driveKS, driveKV, steerKP, steerKD,
                     steerKS, steerKV})
        nonnegative(v);
    if (backend == DriveBackend::SparkMaxNavX)
        for (double derivative : {spark.driveD, spark.steerD})
            if (!std::isfinite(derivative * 1000))
                throw std::invalid_argument("Spark derivative gain overflows millisecond normalization");
    for (double v : {couplingRatio, simulation.centerOfMassX, simulation.centerOfMassY, simulation.centerOfMassZ})
        finite(v);
    if (backend == DriveBackend::SparkMaxNavX)
        for (double current : {driveCurrent, steerCurrent})
            if (current < 1 || current > 200 || std::floor(current) != current)
                throw std::invalid_argument("Spark swerve current limits must be integer amperes in [1,200]");
    // Talons share a device type; CANcoders and Pigeon have separate type IDs.
    std::set<int> motors, encoders;
    auto id = [](int v) {
        if (v < 0 || v > 62)
            throw std::invalid_argument("DriveConfig CAN IDs must be in [0,62]");
    };
    if (backend == DriveBackend::Phoenix)
        id(pigeon);
    else if (!bus.empty() && bus != "rio")
        throw std::invalid_argument("Spark swerve supports the roboRIO CAN bus only");
    double twiceArea = 0;
    const auto validOrientation = [](SimulationDeviceOrientation orientation) {
        return orientation == SimulationDeviceOrientation::CounterClockwisePositive ||
               orientation == SimulationDeviceOrientation::ClockwisePositive;
    };
    for (const auto &module : simulation.modules)
        if (!validOrientation(module.driveOrientation) || !validOrientation(module.steerOrientation) ||
            !validOrientation(module.encoderOrientation))
            throw std::invalid_argument("Drive simulation device orientations must be valid chassis directions");
    for (size_t i = 0; i < modules.size(); ++i)
    {
        const auto &m = modules[i];
        id(m.drive);
        id(m.steer);
        if (backend == DriveBackend::Phoenix)
            id(m.encoder);
        if (!motors.insert(m.drive).second || !motors.insert(m.steer).second ||
            (backend == DriveBackend::Phoenix && !encoders.insert(m.encoder).second))
            throw std::invalid_argument("DriveConfig repeats a CAN device identity on its bus");
        finite(m.offsetTurns);
        finite(m.xMeters);
        finite(m.yMeters);
        if (backend == DriveBackend::SparkMaxNavX && (m.offsetTurns < 0 || m.offsetTurns >= 1))
            throw std::invalid_argument("REV absolute zero offsets must be in [0,1) rotations");
        for (size_t j = 0; j < i; ++j)
            if (std::hypot(m.xMeters - modules[j].xMeters, m.yMeters - modules[j].yMeters) < 1e-6)
                throw std::invalid_argument("DriveConfig module locations must be distinct");
    }
    for (size_t i = 1; i + 1 < modules.size(); ++i)
        twiceArea +=
            std::abs((modules[i].xMeters - modules[0].xMeters) * (modules[i + 1].yMeters - modules[0].yMeters) -
                     (modules[i].yMeters - modules[0].yMeters) * (modules[i + 1].xMeters - modules[0].xMeters));
    if (twiceArea < 1e-9)
        throw std::invalid_argument("DriveConfig module geometry must span a plane");
}
Drive::Drive(DriveConfig config) : config_(std::move(config))
{
    config_.Validate();
    SetName("Drive");
    telemetry_ = std::make_shared<Telemetry>();
    if (replay::Active())
    {
        replayKinematics_ = std::make_unique<ReplayKinematics>(config_);
        return; // Before constructing hardware adapters.
    }
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation() && config_.backend == DriveBackend::Phoenix)
        AwaitPhoenixSimulationTransport(config_);
#endif
    if (config_.backend == DriveBackend::SparkMaxNavX)
    {
        portable_ = std::make_unique<PortableSwerve>(
            config_, MakeSparkNavXSwerveIO(config_, frc::RobotBase::IsSimulation()), !frc::RobotBase::IsSimulation());
#ifndef __FRC_ROBORIO__
#endif
        return;
    }
    p::configs::TalonFXConfiguration drive, steer;
    drive.CurrentLimits.WithSupplyCurrentLimit(units::ampere_t{config_.driveCurrent})
        .WithSupplyCurrentLimitEnable(true);
    steer.CurrentLimits.WithStatorCurrentLimit(units::ampere_t{config_.steerCurrent})
        .WithStatorCurrentLimitEnable(true);
    auto factory = s::SwerveModuleConstantsFactory<p::configs::TalonFXConfiguration, p::configs::TalonFXConfiguration,
                                                   p::configs::CANcoderConfiguration>{}
                       .WithDriveMotorGearRatio(config_.driveRatio)
                       .WithSteerMotorGearRatio(config_.steerRatio)
                       .WithCouplingGearRatio(config_.couplingRatio)
                       .WithWheelRadius(units::meter_t{config_.wheelRadius})
                       .WithDriveMotorGains(p::configs::Slot0Configs{}
                                                .WithKP(config_.driveKP)
                                                .WithKD(config_.driveKD)
                                                .WithKS(config_.driveKS)
                                                .WithKV(config_.driveKV))
                       .WithSteerMotorGains(p::configs::Slot0Configs{}
                                                .WithKP(config_.steerKP)
                                                .WithKD(config_.steerKD)
                                                .WithKS(config_.steerKS)
                                                .WithKV(config_.steerKV))
                       .WithDriveMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithSteerMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithDriveMotorType(s::DriveMotorArrangement::TalonFX_Integrated)
                       .WithSteerMotorType(s::SteerMotorArrangement::TalonFX_Integrated)
                       .WithFeedbackSource(s::SteerFeedbackType::RemoteCANcoder)
                       .WithSlipCurrent(units::ampere_t{config_.slipCurrent})
                       .WithSpeedAt12Volts(units::meters_per_second_t{config_.maxSpeed})
                       .WithDriveMotorInitialConfigs(drive)
                       .WithSteerMotorInitialConfigs(steer)
                       .WithSteerInertia(units::kilogram_square_meter_t{.004})
                       .WithDriveInertia(units::kilogram_square_meter_t{.025})
                       .WithSteerFrictionVoltage(.1_V)
                       .WithDriveFrictionVoltage(.1_V);
    auto make = [&](int i) {
        const auto &m = config_.modules[i];
        return factory.CreateModuleConstants(m.steer, m.drive, m.encoder, units::turn_t{m.offsetTurns},
                                             units::meter_t{m.xMeters}, units::meter_t{m.yMeters}, m.driveInverted,
                                             m.steerInverted, m.encoderInverted);
    };
    native_ = std::make_unique<NativeDrive>(
        s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon),
        units::hertz_t{config_.odometryHz}, make(0), make(1), make(2), make(3));
#ifndef __FRC_ROBORIO__
#endif
    SetName("Drive");
}
Drive::Drive(DriveConfig config, SwerveIO io) : config_(std::move(config))
{
    if (config_.backend != DriveBackend::SparkMaxNavX)
        throw std::invalid_argument("Injected portable Drive requires SparkMaxNavX backend");
    config_.Validate();
    SetName("Drive");
    telemetry_ = std::make_shared<Telemetry>();
    if (replay::Active())
        replayKinematics_ = std::make_unique<ReplayKinematics>(config_);
    else
        portable_ = std::make_unique<PortableSwerve>(config_, std::move(io), !frc::RobotBase::IsSimulation());
}
Drive::~Drive()
{
#ifndef __FRC_ROBORIO__
    if (simulationClockOwner == this)
        simulationClockOwner = nullptr;
#endif
}
void Drive::EnableLiveMotorTuning()
{
    if (motorTuningEnabled_) return;
    if (replay::Active()) { motorTuningEnabled_ = true; return; }
    if (native_) motorTuning_ = std::make_unique<MotorTuningWorker>(*native_);
    else if (portable_ && config_.backend==DriveBackend::SparkMaxNavX)
        motorTuning_ = std::make_unique<MotorTuningWorker>(*portable_);
    else throw std::logic_error("Live motor tuning requires configured swerve hardware");
    motorTuningEnabled_ = true;
}
DriveGainState Drive::MotorGainState() const
{
    if (!motorTuningEnabled_) return DriveGainState::Ready;
    const auto state = replay::Active() ? recordedMotorGainState_ : motorTuning_->state.load();
    return motorGainPending_ && state == DriveGainState::Ready ? DriveGainState::Pending : state;
}
DriveMotorGains Drive::MotorGains() const
{
    if (config_.backend == DriveBackend::SparkMaxNavX)
        return {config_.spark.driveP,0,config_.spark.driveD,config_.spark.driveKS,config_.spark.driveKV,0,
                config_.spark.steerP,0,config_.spark.steerD,0,0,0};
    return {config_.driveKP, appliedMotorGains_.driveI, config_.driveKD, config_.driveKS, config_.driveKV, appliedMotorGains_.driveA,
            config_.steerKP, appliedMotorGains_.steerI, config_.steerKD, config_.steerKS, config_.steerKV, appliedMotorGains_.steerA};
}
bool Drive::SetMotorGains(const DriveMotorGains &gains)
{
    for (double value : {gains.driveP, gains.driveI, gains.driveD, gains.driveS, gains.driveV, gains.driveA,
                         gains.steerP, gains.steerI, gains.steerD, gains.steerS, gains.steerV, gains.steerA})
        if (!std::isfinite(value) || value < 0)
            throw std::invalid_argument("Swerve gains must be finite and nonnegative");
    if (!motorTuningEnabled_) throw std::logic_error("Live motor tuning is not enabled");
    if (config_.backend == DriveBackend::SparkMaxNavX &&
        (gains.driveI!=0 || gains.driveA!=0 || gains.steerI!=0 || gains.steerS!=0 || gains.steerV!=0 || gains.steerA!=0))
        throw std::invalid_argument("Unsupported Spark swerve gain");
    if (motorGainPending_) return false;
    if (!replay::Active() && !motorTuning_->Submit(gains)) return false;
    submittedMotorGains_ = gains;
    motorGainPending_ = true;
    requestedSpeeds_ = {};
    return true;
}
void Drive::Periodic()
{
    if (motorTuningEnabled_)
    {
        const auto input = replay::ProcessInputs("Drive/MotorTuningState", [this] {
            return wpi::json{{"state", static_cast<int>(motorTuning_->state.load())}};
        });
        const int state = input.at("state").get<int>();
        if (state < 0 || state > 2) throw std::runtime_error("Invalid recorded drive tuning state");
        recordedMotorGainState_ = static_cast<DriveGainState>(state);
        if (motorGainPending_ && recordedMotorGainState_ != DriveGainState::Pending)
        {
            motorGainPending_ = false;
            if (recordedMotorGainState_ == DriveGainState::Ready)
            {
                const auto &g = submittedMotorGains_;
                appliedMotorGains_ = g;
                if (config_.backend == DriveBackend::SparkMaxNavX)
                    config_.spark={g.driveP,g.driveD,g.driveS,g.driveV,g.steerP,g.steerD};
                else {
                    config_.driveKP = g.driveP; config_.driveKD = g.driveD; config_.driveKS = g.driveS; config_.driveKV = g.driveV;
                    config_.steerKP = g.steerP; config_.steerKD = g.steerD; config_.steerKS = g.steerS; config_.steerKV = g.steerV;
                }
            }
        }
    }
    if (frc::DriverStation::IsDisabled())
    {
        if (native_)
        {
            auto a = frc::DriverStation::GetAlliance();
            native_->SetOperatorPerspectiveForward(
                frc::Rotation2d{a && *a == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
        }
        Stop();
    }
    if (portable_)
    {
#ifndef __FRC_ROBORIO__
        // Injected desktop IO without an external physics adapter samples at robot cadence.
        if (frc::RobotBase::IsSimulation() && !simulationAdapter_)
            portable_->Sample(frc::RobotController::GetFPGATime() * 1e-6, frc::DriverStation::IsEnabled());
#endif
        portable_->Update();
    }
    auto state = ReadInputs();
    std::array<frc::SwerveModuleState, 4> targets = replayTargets_;
    if (native_)
    {
        auto measured = native_->GetState();
        std::copy(measured.ModuleTargets.begin(), measured.ModuleTargets.end(), targets.begin());
    }
    if (portable_)
        targets = portable_->State().targets;
    const auto requested = requestedSpeeds_;
    auto timestamp = frc::RobotController::GetFPGATime();
    telemetry::Post("Drive", [telemetry = telemetry_, state, targets, requested, timestamp] {
        telemetry->measured.Set(state.states, timestamp);
        telemetry->targets.Set(targets, timestamp);
        telemetry->pose.Set(state.pose, timestamp);
        telemetry->speeds.Set(state.speeds, timestamp);
        telemetry->requested.Set(requested, timestamp);
    });
}
void Drive::SimulationPeriodic()
{
    if (replay::Active())
        return;
#ifndef __FRC_ROBORIO__
    // Every Drive shares one motor/physics runtime. The first Drive called by
    // the scheduler owns its clock, preventing additional Drive instances from
    // stepping all registered plants again during the same robot period.
    if (simulationClockOwner && simulationClockOwner != this)
        return;
    simulationClockOwner = this;
    // Base owns the single fixed-step clock for deterministic motor plants.
    // The optional physics adapter registers on this same clock, so it must
    // advance regardless of whether drive physics is installed.
    simulation::GetSimulationRuntime().StepRobotPeriod();
#endif
}
#ifndef __FRC_ROBORIO__
std::optional<DriveSimulationTruth> Drive::SimulationTruth() const
{
    if (simulationAdapter_)
        return simulationAdapter_->Truth();
    return std::nullopt;
}
void Drive::SetSimulationAdapter(std::unique_ptr<DriveSimulationAdapter> adapter)
{
    if (simulationAdapter_)
        throw std::logic_error("Drive simulation adapter is already installed");
    if (!adapter)
        throw std::invalid_argument("Drive simulation adapter must not be null");
    adapter->Attach(*this);
    simulationAdapter_ = std::move(adapter);
}
void Drive::ResetSimulation(const frc::Pose2d &pose)
{
    if (simulationAdapter_)
    {
        simulationAdapter_->Reset(pose);
        ResetPose(pose);
    }
}

#endif
void Drive::SetRuntimeLimits(double maxTranslation, double maxRotation)
{
    if (!std::isfinite(maxTranslation) || maxTranslation <= 0 || maxTranslation > config_.maxSpeed ||
        !std::isfinite(maxRotation) || maxRotation <= 0 || maxRotation > config_.maxAngularRate)
        throw std::invalid_argument("Runtime drive limits must be positive and within configured limits");
    runtimeLimits_ = std::pair{maxTranslation, maxRotation};
}
bool Drive::ApplyRuntimeLimits(frc::ChassisSpeeds &speeds) const
{
    if (!runtimeLimits_) return false;
    if (!std::isfinite(speeds.vx.value()) || !std::isfinite(speeds.vy.value()) || !std::isfinite(speeds.omega.value()))
    {
        speeds = {};
        return true;
    }
    const auto [maxTranslation, maxRotation] = *runtimeLimits_;
    const double magnitude = std::hypot(speeds.vx.value(), speeds.vy.value());
    bool limited = false;
    if (magnitude > maxTranslation)
    {
        speeds.vx *= maxTranslation / magnitude;
        speeds.vy *= maxTranslation / magnitude;
        limited = true;
    }
    const double omega = std::clamp(speeds.omega.value(), -maxRotation, maxRotation);
    limited |= omega != speeds.omega.value();
    speeds.omega = units::radians_per_second_t{omega};
    return limited;
}
void Drive::RunVelocity(const frc::ChassisSpeeds &speeds)
{
    if (MotorGainState() != DriveGainState::Ready) { requestedSpeeds_ = {}; return; }
    auto limitedSpeeds = speeds;
    ApplyRuntimeLimits(limitedSpeeds);
    requestedSpeeds_ = limitedSpeeds;
    if (native_)
        native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(limitedSpeeds).WithDriveRequestType(
            s::impl::DriveRequestType::Velocity));
    else if (portable_)
        portable_->Velocity(limitedSpeeds);
    else
    {
        auto &kinematics = replayKinematics_->value;
        auto targets = kinematics.ToSwerveModuleStates(limitedSpeeds);
        kinematics.DesaturateWheelSpeeds(&targets, units::meters_per_second_t{config_.maxSpeed});
        auto measured = ReadInputs();
        for (size_t i = 0; i < 4; ++i)
        {
            targets[i].Optimize(measured.states[i].angle);
            replayTargets_[i] = targets[i];
        }
    }
    RecordDriveOutput();
}
void Drive::SetPathOutputLimits(double maxTranslation, double maxRotation)
{
    if (!std::isfinite(maxTranslation) || maxTranslation <= 0 || !std::isfinite(maxRotation) || maxRotation <= 0)
        throw std::invalid_argument("Path output limits must be finite and positive");
    pathOutputLimits_ = std::pair{maxTranslation, maxRotation};
}
void Drive::ClearPathOutputLimits() { pathOutputLimits_.reset(); }
void Drive::RunVelocity(const frc::ChassisSpeeds &input, const pathplanner::DriveFeedforwards &feedforward)
{
    if (MotorGainState() != DriveGainState::Ready) { requestedSpeeds_ = {}; return; }
    auto speeds = input;
    bool limited = ApplyRuntimeLimits(speeds);
    if (pathOutputLimits_)
    {
        const auto [maxTranslation, maxRotation] = *pathOutputLimits_;
        const double magnitude = std::hypot(speeds.vx.value(), speeds.vy.value());
        if (magnitude > maxTranslation)
        {
            speeds.vx *= maxTranslation / magnitude;
            speeds.vy *= maxTranslation / magnitude;
            limited = true;
        }
        const double omega = std::clamp(speeds.omega.value(), -maxRotation, maxRotation);
        limited |= omega != speeds.omega.value();
        speeds.omega = units::radians_per_second_t{omega};
    }
    // Trajectory forces correspond to the original output, not the limited
    // velocity. Drop them when limiting instead of applying inconsistent forces.
    const auto ff = limited ? pathplanner::DriveFeedforwards::zeros(4) : feedforward;
    requestedSpeeds_ = speeds;
    if (native_)
        native_->SetControl(s::requests::ApplyRobotSpeeds{}
                                .WithSpeeds(speeds)
                                .WithDriveRequestType(s::impl::DriveRequestType::Velocity)
                                .WithWheelForceFeedforwardsX(ff.robotRelativeForcesX)
                                .WithWheelForceFeedforwardsY(ff.robotRelativeForcesY));
    else if (portable_)
    {
        std::array<double, 4> x{}, y{};
        if (!ff.robotRelativeForcesX.empty() || !ff.robotRelativeForcesY.empty())
        {
            if (ff.robotRelativeForcesX.size() != 4 || ff.robotRelativeForcesY.size() != 4)
            {
                Stop();
                throw std::invalid_argument("Swerve force feedforwards require four module values");
            }
            for (size_t i = 0; i < 4; ++i)
            {
                x[i] = ff.robotRelativeForcesX[i].value();
                y[i] = ff.robotRelativeForcesY[i].value();
            }
        }
        portable_->Velocity(speeds, x, y);
    }
    else
        RunVelocity(speeds);
    wpi::json x = wpi::json::array(), y = wpi::json::array();
    for (auto v : ff.robotRelativeForcesX)
        x.push_back(v.value());
    for (auto v : ff.robotRelativeForcesY)
        y.push_back(v.value());
    replay::RecordOutput("Drive/FeedforwardForces", {{"x", x}, {"y", y}});
    RecordDriveOutput();
}
} // namespace base
