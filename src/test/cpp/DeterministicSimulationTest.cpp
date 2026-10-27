#include "base/DeterministicSimulation.h"
#include <gtest/gtest.h>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc/Timer.h>

TEST(DeterministicSimulation, MultirateReplayRetainsSensorLatencyAndOrdering)
{
    const std::array<base::MotorReplaySample, 3> log{{{0, {1}}, {4000, {2}}, {8000, {3}}}};
    auto run = [&] {
        base::ReplayMotorIO replay(log);
        base::LatencyChannel<base::MotorInputs, 4> sensor;
        base::DeterministicSimulation simulation;
        std::vector<std::int64_t> commands;
        std::vector<double> positions;
        for (int i = 0; i < 10; ++i)
            simulation.Step([&](auto time) { commands.push_back(time); },
                            [&](auto time) {
                                EXPECT_EQ(commands.size(), static_cast<std::size_t>(time / 20000 + 1));
                                if (auto sample = sensor.Receive(time))
                                    positions.push_back(sample->position);
                                replay.AdvanceTo(time);
                                EXPECT_TRUE(sensor.Push(time + 8000, replay.Read()));
                                const auto position = replay.Read().position;
                                replay.SetVoltage(6);
                                EXPECT_DOUBLE_EQ(replay.Read().position, position);
                            },
                            [](auto) {});
        EXPECT_EQ(commands, (std::vector<std::int64_t>{0, 20000}));
        EXPECT_THROW(replay.AdvanceTo(0), std::invalid_argument);
        return positions;
    };
    const auto positions = run();
    EXPECT_EQ(positions, run());
    ASSERT_EQ(positions.size(), 8u);
    EXPECT_DOUBLE_EQ(positions[0], 1);
    EXPECT_DOUBLE_EQ(positions[1], 2);
    EXPECT_DOUBLE_EQ(positions[2], 3);
    base::LatencyChannel<int, 1> bounded;
    EXPECT_TRUE(bounded.Push(4, 1));
    EXPECT_FALSE(bounded.Push(5, 2));
    EXPECT_FALSE(bounded.Receive(3));
    EXPECT_EQ(bounded.Receive(4), 1);
}

TEST(DeterministicSimulation, ActualControllerAndSchedulerRepeatWithDelayedFeedbackAndVirtualTime)
{
    auto run = [] {
        auto &scheduler = frc2::CommandScheduler::GetInstance();
        scheduler.CancelAll();
        scheduler.Enable();
        frc::sim::PauseTiming();
        const auto start = frc::Timer::GetFPGATimestamp();
        auto plant = base::MakeMotorIO(base::Mode::Sim, base::Vendor::SparkMax, {{1}, {false}, 1, 40, ""});
        auto input = std::make_unique<base::InjectedMotorIO>();
        auto *observed = input.get();
        base::Gains gains;
        gains.p = .8;
        gains.v = .127;
        gains.maxAcceleration = 100;
        base::Flywheel wheel("VirtualController", std::move(input), gains);
        wheel.UseExternalScheduling();
        auto command = wheel.VelocityCommand([] { return 20.; });
        base::LatencyChannel<base::MotorInputs, 8> transport;
        base::DeterministicSimulation sim;
        std::vector<double> result;
        for (int tick = 0; tick < 500; ++tick)
            sim.StepWithSensors(
                [&](auto time) {
                    const bool enabled = time < 1600000;
                    frc::sim::DriverStationSim::SetDsAttached(true);
                    frc::sim::DriverStationSim::SetAutonomous(false);
                    frc::sim::DriverStationSim::SetTest(false);
                    frc::sim::DriverStationSim::SetEStop(false);
                    frc::sim::DriverStationSim::SetEnabled(enabled);
                    frc::sim::DriverStationSim::NotifyNewData();
                    frc::DriverStation::RefreshData();
                    if (auto sample = transport.Receive(time))
                        observed->inputs = *sample;
                },
                [&](auto time) {
                    if (time == 0)
                        command.Schedule();
                    scheduler.Run();
                },
                [&](auto time) {
                    wheel.Control(units::second_t{.004});
                    if (time >= 1600000)
                        EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
                    result.push_back(observed->requestedVoltage);
                    plant->SetVoltage(observed->requestedVoltage);
                },
                [&](auto time) {
                    plant->AdvanceSimulation(units::second_t{.004});
                    EXPECT_TRUE(transport.Push(time + 4000 + 8000, plant->Read()));
                    frc::sim::StepTiming(units::second_t{.004});
                });
        EXPECT_FALSE(command.IsScheduled());
        EXPECT_NEAR((frc::Timer::GetFPGATimestamp() - start).value(), 2., 1e-6);
        // Current limiting/inertia constrain acceleration; require observable motion,
        // not ideal instantaneous setpoint tracking.
        EXPECT_GT(wheel.Position(), 1);
        testing::Test::RecordProperty("delayed_final_position_rotations", wheel.Position());
        scheduler.CancelAll();
        frc::sim::ResumeTiming();
        return result;
    };
    const auto first = run();
    EXPECT_EQ(first, run());
}
