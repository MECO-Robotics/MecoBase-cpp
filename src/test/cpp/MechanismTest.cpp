#include "base/Mechanism.h"
#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <limits>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <frc/smartdashboard/SmartDashboard.h>

TEST(MechanismConfig, RejectsInvalidMappingAndLimits)
{
    base::MotorConfig valid{{1, 2}, {false, true}, 2, 40, ""};
    EXPECT_NO_THROW(valid.Validate());
    valid.ids[1] = 1;
    EXPECT_THROW(valid.Validate(), std::invalid_argument);
    base::Gains gains;
    gains.minimum = 1;
    gains.maximum = 0;
    EXPECT_THROW(gains.Validate(true), std::invalid_argument);
}
TEST(MechanismEncoder, AppliesDirectionBeforeOffsetAndScale)
{
    base::EncoderCalibration encoder{2, .25, true};
    EXPECT_DOUBLE_EQ(encoder.Apply(.5), -.5);
}
TEST(MechanismControl, DisabledAndSoftLimitsPreventVoltage)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.minimum = 0;
    gains.maximum = 1;
    gains.p = 5;
    base::PositionJoint joint("LimitTest", std::move(io), gains);
    observed->inputs.position = 1;
    joint.SetVoltage(4);
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    observed->inputs.position = .5;
    joint.SetVoltage(4);
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 4);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
}
TEST(MechanismReference, ResetRetainsPhysicalSoftLimits)
{
    HAL_Initialize(500, 0);
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.minimum = 0;
    gains.maximum = 1;
    base::PositionJoint joint("ResetTest", std::move(io), gains);
    observed->inputs.position = .75;
    joint.Periodic();
    joint.ResetPosition();
    EXPECT_DOUBLE_EQ(joint.Position(), 0);
    joint.SetPosition(5);
    EXPECT_DOUBLE_EQ(joint.DesiredPosition(), .25);
}
TEST(MechanismIO, ReplayDoesNotConstructHardwareAndSimulationMoves)
{
    base::MotorConfig config{{1}, {false}, 2, 40, ""};
    auto replay = base::MakeMotorIO(base::Mode::Replay, base::Vendor::TalonFX, config);
    ASSERT_NE(dynamic_cast<base::InjectedMotorIO *>(replay.get()), nullptr);
    auto sim = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, config);
    sim->SetVoltage(4);
    for (int n = 0; n < 20; ++n)
        sim->StepSimulation(units::second_t{.02});
    EXPECT_GT(sim->Read().position, 0);
    EXPECT_GT(sim->Read().velocity, 0);
}
TEST(MechanismSimulation, ClosedLoopVelocityConverges)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    base::MotorConfig config{{1}, {false}, 1, 40, ""};
    base::Gains gains;
    gains.p = .8;
    gains.v = .127;
    gains.maxVelocity = 50;
    gains.maxAcceleration = 100;
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, config);
    auto *plant = io.get();
    base::Flywheel wheel("SimControl", std::move(io), gains);
    wheel.SetVelocity(20);
    for (int n = 0; n < 500; ++n)
    {
        wheel.Periodic();
        plant->StepSimulation(units::second_t{.02});
    }
    EXPECT_NEAR(wheel.Velocity(), 20, 1);
    wheel.Stop();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismJoint, VelocityModeUsesVelocityErrorAndPhysicalLimits)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.minimum = 0;
    gains.maximum = 1;
    gains.p = 5;
    base::PositionJoint joint("JointVelocity", std::move(io), gains);
    observed->inputs.position = .5;
    joint.Periodic();
    EXPECT_NO_THROW(joint.SetVelocity(.5));
    joint.Periodic();
    EXPECT_TRUE(joint.VelocityControlActive());
    EXPECT_GT(observed->requestedVoltage, 0);
    observed->inputs.position = 1;
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismProfile, ReversingDeceleratesBeforeChangingDirection)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.p = 100;
    gains.minimum = 0;
    gains.maximum = 1;
    base::PositionJoint joint("ProfileReverse", std::move(io), gains);
    observed->inputs.position = .5;
    observed->inputs.velocity = .9;
    joint.Periodic();
    joint.SetPosition(.499);
    joint.Periodic();
    // The profile is still moving positively while slowing down; it must not snap backward.
    EXPECT_GT(observed->requestedVoltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismFlywheel, VelocityTargetHasNoJointProfileSpeedCap)
{
    auto io = std::make_unique<base::InjectedMotorIO>();
    base::Flywheel wheel("FlywheelTarget", std::move(io), base::Gains{});
    wheel.SetVelocity(30);
    EXPECT_DOUBLE_EQ(wheel.VelocitySetpoint(), 30);
}

TEST(MechanismSafety, InvalidExternalFeedbackStopsPreviousOutput)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    double sensor = .5;
    base::PositionJoint joint("InvalidEncoder", std::move(io), base::Gains{}, 1, base::Gravity::Constant,
                              [&sensor] { return sensor; });
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        sensor = .5;
        joint.SetVoltage(4);
        joint.Periodic();
        ASSERT_DOUBLE_EQ(observed->requestedVoltage, 4);
        sensor = invalid;
        EXPECT_NO_THROW(joint.Periodic());
        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
        joint.SetPosition(.5);
        EXPECT_FALSE(joint.AtSetpoint());
        const double offset = joint.PositionOffset();
        EXPECT_NO_THROW(joint.ResetPosition());
        EXPECT_DOUBLE_EQ(joint.PositionOffset(), offset);
        sensor = .5;
        joint.Periodic();
        EXPECT_TRUE(std::isfinite(joint.Position()));
        EXPECT_DOUBLE_EQ(joint.PositionOffset(), offset);
        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    }
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismSafety, OverflowingControlOutputStopsInsteadOfSaturating)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.p = std::numeric_limits<double>::max();
    gains.maxAcceleration = 1000;
    base::Flywheel wheel("OverflowControl", std::move(io), gains);
    wheel.SetVoltage(4);
    wheel.Periodic();
    ASSERT_DOUBLE_EQ(observed->requestedVoltage, 4);
    wheel.SetVelocity(10);
    wheel.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    wheel.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismProfile, CloseTargetDoesNotViolateAccelerationAtCrossing)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.a = 1;
    gains.minimum = 0;
    gains.maximum = 2;
    base::PositionJoint joint("ProfileCloseTarget", std::move(io), gains);
    observed->inputs.position = .5;
    observed->inputs.velocity = .9;
    joint.Periodic();
    joint.SetPosition(.501);
    // With only kA enabled, requested volts numerically equal profile acceleration.
    // The nearby target requires overshoot before the initial speed can be stopped.
    for (int n = 0; n < 200; ++n)
    {
        joint.Periodic();
        EXPECT_LE(std::abs(observed->requestedVoltage), 1 + 1e-12);
    }
    // Verify that allowing overshoot still converges to the requested position.
    gains.a = 0;
    gains.p = 1;
    joint.SetGains(gains);
    joint.Periodic();
    EXPECT_NEAR(observed->requestedVoltage, .001, 1e-9);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismSafety, DisconnectedFeedbackStopsAndCannotReportAtSetpoint)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    observed->inputs.position = .5;
    base::PositionJoint joint("DisconnectedFeedback", std::move(io), base::Gains{});
    joint.Periodic();
    joint.SetPosition(.5);
    ASSERT_TRUE(joint.AtSetpoint());
    observed->inputs.connected = false;
    joint.Periodic();
    base::telemetry::Flush();
    EXPECT_FALSE(frc::SmartDashboard::GetBoolean("DisconnectedFeedback/Connected", true));
    EXPECT_FALSE(joint.AtSetpoint());
    joint.SetVoltage(4);
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    EXPECT_FALSE(joint.AtSetpoint());
    observed->inputs.connected = true;
    joint.Periodic();
    base::telemetry::Flush();
    EXPECT_TRUE(frc::SmartDashboard::GetBoolean("DisconnectedFeedback/Connected", false));
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    EXPECT_FALSE(joint.AtSetpoint());
    joint.SetVoltage(4);
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 4);
    joint.Stop();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismControl, SwitchingToPositionHoldClearsVelocityPidHistory)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    for (bool derivative : {false, true})
    {
        auto io = std::make_unique<base::InjectedMotorIO>();
        auto *observed = io.get();
        observed->inputs.connected = true;
        observed->inputs.position = .5;
        base::Gains gains;
        gains.maxAcceleration = 1000;
        if (derivative)
            gains.d = 1;
        else
            gains.i = 1;
        base::PositionJoint joint("ModeTransition", std::move(io), gains);
        joint.Periodic();
        for (int n = 0; n < 100; ++n)
        {
            joint.SetVelocity(.5);
            joint.Periodic();
        }
        // Repeated requests in one mode retain integral history and a settled derivative.
        EXPECT_NEAR(observed->requestedVoltage, derivative ? 0 : 1, 1e-12);
        joint.SetPosition(.5);
        joint.Periodic();
        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    }
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismProfile, VelocityEntryRestoresConfiguredAccelerationAfterDynamicPosition)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    observed->inputs.position = .5;
    base::Gains gains;
    gains.a = 1;
    gains.maxAcceleration = 1;
    base::PositionJoint joint("ModeConstraints", std::move(io), gains);
    joint.Periodic();
    joint.SetPositionDynamic(.5, 1, 1000);
    joint.SetVelocity(.5);
    joint.Periodic();
    // kA-only voltage equals acceleration: the position request's override is over.
    EXPECT_NEAR(observed->requestedVoltage, 1, 1e-12);
    joint.Stop();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismNative, RequestsProfiledPhysicalCoordinatesAndStopsOnFaults)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->nativeEnabled = true;
    observed->inputs.connected = true;
    observed->inputs.position = .25;
    base::Gains gains;
    gains.minimum = 0;
    gains.maximum = 2;
    gains.maxVelocity = 2;
    gains.maxAcceleration = 4;
    base::PositionJoint joint("NativeContract", std::move(io), gains, 2);
    joint.Periodic();
    joint.SetPosition(1.5);
    joint.Periodic();
    ASSERT_TRUE(observed->nativeRequest);
    EXPECT_EQ(observed->nativeRequest->kind, base::NativeControlRequest::Kind::MotionMagic);
    EXPECT_DOUBLE_EQ(observed->nativeRequest->target, 1.5);
    joint.ResetPosition();
    joint.Periodic();
    ASSERT_TRUE(observed->nativeRequest);
    EXPECT_DOUBLE_EQ(joint.Position(), 0);
    EXPECT_DOUBLE_EQ(observed->nativeRequest->target, .5); // no sensor/physical-limit teleport
    joint.SetPositionDynamic(1, .3, .7);
    joint.Periodic();
    ASSERT_TRUE(observed->nativeRequest);
    EXPECT_EQ(observed->nativeRequest->kind, base::NativeControlRequest::Kind::Position);
    EXPECT_GT(observed->nativeRequest->target, .5);
    EXPECT_LT(observed->nativeRequest->target, .51);
    EXPECT_THROW(joint.SetGains(gains), std::logic_error);
    observed->inputs.connected = false;
    joint.Periodic();
    EXPECT_FALSE(observed->nativeRequest);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    observed->inputs.connected = true;
    observed->inputs.position = 1;
    joint.SetVelocity(1);
    joint.Periodic();
    EXPECT_FALSE(observed->nativeRequest); // physical upper limit
    observed->inputs.position = .25;
    joint.SetVelocity(1);
    joint.Periodic();
    ASSERT_TRUE(observed->nativeRequest);
    EXPECT_GT(observed->nativeRequest->target, 0);
    EXPECT_LT(observed->nativeRequest->target, 1);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    joint.Periodic();
    EXPECT_FALSE(observed->nativeRequest);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
}

TEST(MechanismNative, FailedReconfigurationLatchesStopUntilDisabledRecovery)
{
    class ConfigurableIO final : public base::MotorIO
    {
      public:
        bool fail = false;
        int requests = 0;
        double voltage = 0;
        base::MotorInputs Read() override
        {
            base::MotorInputs input;
            input.connected = true;
            return input;
        }
        void SetVoltage(double value) override
        {
            voltage = value;
        }
        void SetBrake(bool) override
        {
        }
        bool ConfigureNative(const base::Gains &, double, base::Gravity, bool) override
        {
            if (fail)
                throw std::runtime_error("configuration rejected");
            return true;
        }
        bool SetNative(const base::NativeControlRequest &) override
        {
            ++requests;
            return true;
        }
    };
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<ConfigurableIO>();
    auto *observed = io.get();
    base::Gains gains;
    base::Flywheel wheel("NativeConfigFault", std::move(io), gains);
    observed->fail = true;
    EXPECT_THROW(wheel.SetGains(gains), std::runtime_error);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetVelocity(1);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 0);
    EXPECT_DOUBLE_EQ(observed->voltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    observed->fail = false;
    wheel.SetGains(gains);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetVelocity(1);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 1);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismControl, ExplicitTimestepScalesIntegralDerivativeAndFeedforward)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    for (double dt : {.02, .01, .004})
    {
        auto io = std::make_unique<base::InjectedMotorIO>();
        auto *observed = io.get();
        observed->inputs.connected = true;
        base::Gains gains;
        gains.p = .3;
        gains.i = .4;
        gains.d = .002;
        gains.v = .2;
        gains.a = .1;
        gains.maxAcceleration = 2;
        base::Flywheel wheel("TimestepTest", std::move(io), gains);
        wheel.SetVelocity(1);
        wheel.Control(units::second_t{dt}, true);
        const double profiledVelocity = 2 * dt;
        const double expected = .3 * profiledVelocity + .4 * profiledVelocity * dt + .002 * profiledVelocity / dt +
                                .2 * profiledVelocity + .1 * 2;
        EXPECT_NEAR(observed->requestedVoltage, expected, 1e-12);
        wheel.PublishTelemetry();
        EXPECT_NEAR(observed->requestedVoltage, expected, 1e-12);
        EXPECT_THROW(wheel.Control(units::second_t{0}, true), std::invalid_argument);
        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
        wheel.SetVoltage(6);
        EXPECT_THROW(wheel.Control(units::second_t{.101}, true), std::invalid_argument);
        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    }
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismControl, ExternalSchedulingAvoidsDuplicateTicksAndTelemetryDoesNotControl)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    base::Gains gains;
    gains.p = 1;
    gains.maxAcceleration = 2;
    base::Flywheel wheel("ExternalTickTest", std::move(io), gains);
    wheel.UseExternalScheduling();
    wheel.SetVelocity(1);
    wheel.Control(units::second_t{.004}, true);
    const double voltage = observed->requestedVoltage;
    ASSERT_GT(voltage, 0);
    observed->inputs.velocity = 100;
    wheel.Periodic();
    EXPECT_DOUBLE_EQ(wheel.Velocity(), 0);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, voltage);
    wheel.Control(units::second_t{.004}, false);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetVoltage(6);
    wheel.Control(units::second_t{.004}, true);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
}

TEST(MechanismAdapter, IoOwnsControllerAndUsesSuppliedFeedbackWithoutReadingAgain)
{
    class ObservedIO final : public base::MotorIO
    {
      public:
        int reads = 0;
        double volts = 0;
        base::MotorInputs Read() override
        {
            ++reads;
            return {};
        }
        void SetVoltage(double voltage) override
        {
            volts = voltage;
        }
        void SetBrake(bool) override
        {
        }
    } io;
    base::Gains gains;
    gains.p = 2;
    gains.i = 1;
    gains.v = .3;
    gains.a = .2;
    EXPECT_FALSE(io.ConfigureControl({gains, 2, base::Gravity::Constant, false, false}));
    base::MotorInputs sample;
    sample.connected = true;
    sample.velocity = .25;
    base::NativeControlRequest setpoint{base::NativeControlRequest::Kind::Velocity, 1, 1, 1, 1, .5};
    auto result = io.Control({setpoint, 0}, sample, 0, units::second_t{.01});
    ASSERT_TRUE(result.ok);
    ASSERT_TRUE(result.voltage);
    EXPECT_NEAR(io.volts, 1 + .005 + .3 + .1, 1e-12);
    EXPECT_EQ(io.reads, 0); // Replay-substituted sample is the only controller input.
    io.Control({setpoint, 0}, sample, 0, units::second_t{.004});
    EXPECT_NEAR(io.volts, 1 + .007 + .3 + .1, 1e-12);
    io.ResetControl();
    io.Control({setpoint, 0}, sample, 0, units::second_t{.004});
    EXPECT_NEAR(io.volts, 1 + .002 + .3 + .1, 1e-12);
}

TEST(MechanismAdapter, NativeRequestFailureLatchesUntilDisabledReconfiguration)
{
    class FaultIO final : public base::MotorIO
    {
      public:
        int requests = 0;
        bool fail = true;
        double volts = 0;
        base::MotorInputs Read() override
        {
            base::MotorInputs input;
            input.connected = true;
            return input;
        }
        void SetVoltage(double voltage) override
        {
            volts = voltage;
        }
        void SetBrake(bool) override
        {
        }
        bool ConfigureNative(const base::Gains &, double, base::Gravity, bool) override
        {
            return true;
        }
        bool SetNative(const base::NativeControlRequest &) override
        {
            ++requests;
            return !fail;
        }
    };
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<FaultIO>();
    auto *observed = io.get();
    base::Flywheel wheel("AdapterRequestFault", std::move(io), base::Gains{});
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetVelocity(1);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 1);
    EXPECT_DOUBLE_EQ(observed->volts, 0);
    observed->fail = false;
    wheel.SetVelocity(1);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 1);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetGains(base::Gains{});
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    wheel.SetVelocity(1);
    wheel.Periodic();
    EXPECT_EQ(observed->requests, 2);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    // Drain this test's asynchronous snapshots before NT static teardown.
    base::telemetry::Flush();
}

TEST(MechanismAdapter, NativeAndEmulatedSourcesRetainTheirReplayBoundary)
{
    base::InjectedMotorIO io;
    base::Gains gains;
    gains.p = 2;
    EXPECT_FALSE(io.ConfigureControl({gains, 1, base::Gravity::Constant, false, false}));
    base::MotorInputs input;
    input.connected = true;
    base::NativeControlRequest request{base::NativeControlRequest::Kind::Velocity, 1, 1, 1, 1, 0};
    io.UseRecordedControl(false);
    auto emulated = io.Control({request, 0}, input, 0, units::second_t{.02});
    ASSERT_TRUE(emulated.voltage);
    EXPECT_DOUBLE_EQ(*emulated.voltage, 2);
    io.UseRecordedControl(true);
    auto native = io.Control({request, 0}, input, 0, units::second_t{.02});
    EXPECT_TRUE(native.ok);
    EXPECT_FALSE(native.voltage);
    ASSERT_TRUE(io.nativeRequest);
    EXPECT_DOUBLE_EQ(io.nativeRequest->target, 1);
    io.UseRecordedControl(false);
    emulated = io.Control({request, 0}, input, 0, units::second_t{.02});
    ASSERT_TRUE(emulated.voltage);
    EXPECT_FALSE(io.nativeRequest);
}
