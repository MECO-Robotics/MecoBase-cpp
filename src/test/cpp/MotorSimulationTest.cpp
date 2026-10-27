#include "base/Mechanism.h"
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include "base/SimulationRuntime.h"

TEST(MotorSimulation, HighReductionConvergesWithoutNumericalReversal)
{
    // No-load NEO speed at 4 V, including the model's output viscous drag.
    constexpr double ratio = 200, inertia = .025, voltage = 4;
    constexpr double resistance = 12. / 105, kt = 2.6 / 105;
    constexpr double kv = 5676 * 2 * std::numbers::pi / 60 / 12;
    const double expected = voltage / (ratio / kv + .0001 * resistance / (kt * ratio)) / (2 * std::numbers::pi);
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, base::MotorConfig{{1}, {false}, ratio, 40, ""},
                                inertia);
    io->SetVoltage(voltage);
    base::MotorInputs state;
    for (int n = 0; n < 50; ++n)
    {
        io->StepSimulation(units::second_t{.02});
        state = io->Read();
        EXPECT_GE(state.velocity, 0);
        EXPECT_LE(state.velocity, expected + 1e-9);
        EXPECT_LE(state.current, 40 + 1e-9);
    }
    EXPECT_NEAR(state.velocity, expected, 1e-6);
    io->SetVoltage(-voltage);
    for (int n = 0; n < 50; ++n)
    {
        io->StepSimulation(units::second_t{.02});
        state = io->Read();
    }
    EXPECT_NEAR(state.velocity, -expected, 1e-6);
    io->SetBrake(false);
    io->SetVoltage(0);
    io->StepSimulation(units::second_t{.02});
    const double coastSpeed = std::abs(io->Read().velocity);
    io->SetBrake(true);
    io->StepSimulation(units::second_t{.02});
    EXPECT_LT(std::abs(io->Read().velocity), coastSpeed * .01);
}

TEST(MotorSimulation, CurrentLimitBoundsStartupAcceleration)
{
    constexpr double inertia = .025, ratio = 2, kt = 2.6 / 105;
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax,
                                base::MotorConfig{{1, 2}, {false, true}, ratio, 10, ""}, inertia);
    io->SetVoltage(12);
    io->StepSimulation(units::second_t{.02});
    const auto state = io->Read();
    const double acceleration = kt * 10 * 2 * ratio / inertia;
    EXPECT_GT(state.velocity, 0);
    EXPECT_LE(state.velocity, acceleration * .02 / (2 * std::numbers::pi));
    EXPECT_LE(state.position, .5 * acceleration * .02 * .02 / (2 * std::numbers::pi));
    EXPECT_LE(state.current, 20 + 1e-9);
}

TEST(MotorSimulation, ReadsDoNotAdvanceAndRuntimeOwnsLifetime)
{
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, base::MotorConfig{{1}, {false}, 2, 40, ""});
    io->SetVoltage(4);
    const auto before = io->Read();
    for (int n = 0; n < 20; ++n)
        EXPECT_DOUBLE_EQ(io->Read().position, before.position);
    base::simulation::GetSimulationRuntime().StepRobotPeriod();
    const auto once = io->Read();
    EXPECT_GT(once.position, before.position);
    EXPECT_DOUBLE_EQ(io->Read().position, once.position);
    io.reset();
    EXPECT_NO_THROW(base::simulation::GetSimulationRuntime().StepRobotPeriod());
}

TEST(MotorSimulation, GravityAndTravelStopsAllowMotionAway)
{
    const base::MotorConfig motor{{1}, {false}, 1, 40, ""};
    base::MotorSimulationConfig arm;
    arm.load = base::MotorSimulationConfig::Load::Arm;
    arm.mass = units::kilogram_t{.2};
    arm.armCOM = units::meter_t{.2};
    arm.minimum = units::radian_t{-.1};
    arm.maximum = units::radian_t{.2};
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, motor, arm);
    io->SetBrake(false);
    for (int n = 0; n < 50; ++n)
        io->StepSimulation(units::second_t{.02});
    EXPECT_NEAR(io->Read().position, -.1 / (2 * std::numbers::pi), 1e-12);
    EXPECT_DOUBLE_EQ(io->Read().velocity, 0);
    io->SetVoltage(12);
    for (int n = 0; n < 50; ++n)
        io->StepSimulation(units::second_t{.02});
    EXPECT_NEAR(io->Read().position, .2 / (2 * std::numbers::pi), 1e-12);
    EXPECT_DOUBLE_EQ(io->Read().velocity, 0);

    base::MotorSimulationConfig elevator;
    elevator.load = base::MotorSimulationConfig::Load::Elevator;
    elevator.mass = units::kilogram_t{2};
    elevator.drumRadius = units::meter_t{.05};
    auto lift = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, motor, elevator);
    lift->SetBrake(false);
    lift->StepSimulation(units::second_t{.02});
    // Closed-form free fall with reflected translating mass and viscous drag:
    // J * omega_dot = -m*g*r - b*omega, starting from rest.
    constexpr double inertia = .025 + 2 * .05 * .05, viscousDrag = .0001, duration = .02;
    const double expected = (-2 * 9.80665 * .05 / viscousDrag) * (-std::expm1(-viscousDrag * duration / inertia));
    EXPECT_NEAR(lift->Read().velocity * 2 * std::numbers::pi, expected, 1e-10);
}

TEST(MotorSimulation, RejectsInvalidConfigurationAndTime)
{
    const base::MotorConfig motor{{1}, {false}, 1, 40, ""};
    base::MotorSimulationConfig invalid;
    invalid.load = base::MotorSimulationConfig::Load::Arm;
    EXPECT_THROW(base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, motor, invalid), std::invalid_argument);
    auto io = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, motor);
    EXPECT_THROW(io->StepSimulation(units::second_t{-.01}), std::invalid_argument);
    EXPECT_THROW(io->StepSimulation(units::second_t{NAN}), std::invalid_argument);
    EXPECT_THROW(io->SetVoltage(NAN), std::invalid_argument);
}

TEST(MotorSimulation, HeldTravelStopReportsStallCurrent)
{
    base::MotorSimulationConfig model;
    model.maximum = units::radian_t{0};
    auto io =
        base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, base::MotorConfig{{1}, {false}, 200, 40, ""}, model);
    io->SetVoltage(12);
    io->StepSimulation(units::second_t{.02});
    EXPECT_DOUBLE_EQ(io->Read().position, 0);
    EXPECT_DOUBLE_EQ(io->Read().velocity, 0);
    EXPECT_NEAR(io->Read().current, (12. / 105) * 40 * 40 / 12, 1e-9);
    // Current is battery-side: winding limiting lowers terminal voltage at stall.
    EXPECT_NEAR(io->Read().voltage, (12. / 105) * 40, 1e-9);
    io->SetBrake(false);
    io->SetVoltage(0);
    io->StepSimulation(units::second_t{.02});
    EXPECT_DOUBLE_EQ(io->Read().current, 0);
}

TEST(MotorSimulation, RuntimeBusClampsPartialVoltageAndRecovers)
{
    auto &clock = base::simulation::GetSimulationRuntime();
    clock.Reset();
    base::MotorSimulationConfig model;
    model.maximum = units::radian_t{0};
    auto io =
        base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, base::MotorConfig{{1}, {false}, 1, 1000, ""}, model);
    io->SetVoltage(6);
    clock.SetBusVoltage(4);
    clock.StepRobotPeriod();
    EXPECT_DOUBLE_EQ(io->Read().supplyVoltage, 4);
    EXPECT_DOUBLE_EQ(io->Read().voltage, 4);
    EXPECT_NEAR(io->Read().current, 4 / (12. / 105), 1e-9);
    EXPECT_NEAR(clock.SupplyCurrent(), io->Read().current, 1e-9);
    clock.SetBusVoltage(12);
    clock.StepRobotPeriod();
    EXPECT_DOUBLE_EQ(io->Read().voltage, 6);
    EXPECT_NEAR(io->Read().current, 6 * 6 / (12. / 105) / 12, 1e-9);
    io->SetVoltage(0);
    clock.StepRobotPeriod();
    EXPECT_DOUBLE_EQ(clock.SupplyCurrent(), 0);
    io.reset();
    clock.Reset();
}
