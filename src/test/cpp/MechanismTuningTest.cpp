#include "base/MechanismTuning.h"
#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <hal/HAL.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/json.h>
namespace
{
wpi::json GainRequest(uint64_t revision, double p)
{
    return {{"revision", revision},
            {"gains",
             {{"p", p},
              {"i", 0},
              {"d", 0},
              {"s", 0},
              {"v", 0},
              {"a", 0},
              {"gravity", 0},
              {"maxVelocity", 1},
              {"maxAcceleration", 1},
              {"tolerance", .01},
              {"minimum", 0},
              {"maximum", 1},
              {"defaultPosition", 0}}}};
}
void State(bool enabled, bool test)
{
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetTest(test);
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
}
} // namespace
TEST(MechanismTuning, AtomicGainsAndRejectedRequestsNeverApplyLater)
{
    HAL_Initialize(500, 0);
    State(false, false);
    auto io = std::make_unique<base::InjectedMotorIO>();
    io->inputs.connected = true;
    base::PositionJoint joint("TuneAtomic", std::move(io), {});
    base::MechanismTuning tuning(joint);
    auto pub = nt::NetworkTableInstance::GetDefault().GetStringTopic("/Tuning/TuneAtomic/Request").Publish();
    auto invalid = GainRequest(1, 4);
    invalid["gains"]["maxAcceleration"] = 0;
    pub.Set(invalid.dump());
    tuning.Periodic();
    EXPECT_EQ(joint.GetGains().p, 0);
    pub.Set(GainRequest(2, 3).dump());
    tuning.Periodic();
    EXPECT_EQ(joint.GetGains().p, 3);
    State(true, false);
    pub.Set(GainRequest(3, 7).dump());
    tuning.Periodic();
    EXPECT_EQ(joint.GetGains().p, 3);
    State(false, false);
    tuning.Periodic();
    pub.Set(GainRequest(3, 7).dump());
    tuning.Periodic();
    EXPECT_EQ(joint.GetGains().p, 3);
    EXPECT_EQ(tuning.LastRevision(), 3u);
    base::telemetry::Flush();
}
TEST(MechanismTuning, TestCommandOwnsAndStopsMechanismWithoutLatchedRequests)
{
    HAL_Initialize(500, 0);
    State(true, true);
    auto io = std::make_unique<base::InjectedMotorIO>();
    auto *observed = io.get();
    observed->inputs.connected = true;
    observed->inputs.position = .1;
    base::Gains gains;
    gains.p = 4;
    base::PositionJoint joint("TuneCommand", std::move(io), gains);
    base::MechanismTuning tuning(joint);
    auto pub = nt::NetworkTableInstance::GetDefault().GetStringTopic("/Tuning/TuneCommand/Request").Publish();
    auto send = [&](int revision, double target) {
        pub.Set(wpi::json{{"revision", revision}, {"setpoint", target}}.dump());
        tuning.Periodic();
    };
    pub.Set(wpi::json{{"revision", 1}, {"setpoint", .8}}.dump()); // Queued before ownership.
    auto command = tuning.TestCommand();
    auto &scheduler = frc2::CommandScheduler::GetInstance();
    scheduler.Schedule(command);
    scheduler.Run();
    EXPECT_TRUE(command.IsScheduled());
    EXPECT_EQ(joint.GetCurrentCommand(), command.get());
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    send(2, .8);
    scheduler.Run();
    joint.Periodic();
    EXPECT_GT(observed->requestedVoltage, 0);
    command.Cancel();
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    send(3, .7);
    scheduler.Schedule(command);
    scheduler.Run();
    joint.Periodic();
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    State(true, false);
    scheduler.Run();
    joint.Periodic();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    State(false, false);
    base::telemetry::Flush();
}
