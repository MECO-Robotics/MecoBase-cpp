#include "base/LiveMechanismTuning.h"
#include <gtest/gtest.h>
#include <limits>
#include <hal/HAL.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc/simulation/DriverStationSim.h>
#include <networktables/NetworkTableInstance.h>

TEST(LiveMechanismTuning, EnabledEditAndInvalidLimits)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    io->inputs.connected = true;
    base::PositionJoint joint("LiveJoint", std::move(io), {});
    base::LiveMechanismTuning tuning(joint, true);
    auto nt = nt::NetworkTableInstance::GetDefault();
    auto p = nt.GetEntry("/TunableNumbers/LiveJoint/Gains/kP");
    auto max = nt.GetEntry("/TunableNumbers/LiveJoint/Gains/kMaxPosition");
    p.SetDouble(4);
    tuning.Periodic();
    EXPECT_DOUBLE_EQ(joint.GetGains().p, 4);
    p.SetDouble(std::numeric_limits<double>::quiet_NaN());
    tuning.Periodic();
    EXPECT_EQ(nt.GetEntry("/Tuning/LiveJoint/LiveStatus").GetString(""), "rejected: nonfinite input");
    EXPECT_DOUBLE_EQ(joint.GetGains().p, 4);
    p.SetDouble(4);
    max.SetDouble(-1);
    tuning.Periodic();
    EXPECT_DOUBLE_EQ(joint.GetGains().maximum, 1);
    max.SetDouble(2);
    tuning.Periodic();
    EXPECT_DOUBLE_EQ(joint.GetGains().maximum, 2);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(LiveMechanismTuning, GainChangesDoNotPreemptOwningCommand)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io = std::make_unique<base::InjectedMotorIO>();
    io->inputs.connected = true;
    base::PositionJoint joint("LiveOwned", std::move(io), {});
    base::LiveMechanismTuning tuning(joint, true);
    auto command = frc2::cmd::RunEnd([&] { joint.SetPosition(.5); }, [&] { joint.Stop(); }, {&joint});
    auto &scheduler = frc2::CommandScheduler::GetInstance();
    scheduler.Schedule(command);
    scheduler.Run();
    ASSERT_EQ(joint.GetCurrentCommand(), command.get());
    nt::NetworkTableInstance::GetDefault().GetEntry("/TunableNumbers/LiveOwned/Gains/kP").SetDouble(6);
    scheduler.Run();
    EXPECT_TRUE(command.IsScheduled());
    EXPECT_EQ(joint.GetCurrentCommand(), command.get());
    EXPECT_DOUBLE_EQ(joint.GetGains().p, 6);
    command.Cancel();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
