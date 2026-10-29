#include "base/FootprintCommand.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <stdexcept>

TEST(FootprintCommand, OwnsDriveAndStopsOnDisableAndBlockedGeometry) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    const base::FieldRectangle field{-5,-5,5,5};
    const base::RectangularFootprint robot{.6,.4};
    bool blocked=false, throwing=false;
    auto command=base::FollowFootprintRoute(drive,{{0,0,0},{2,0,0}},robot,field,[&] {
        if (throwing) throw std::runtime_error("obstacle source failed");
        return blocked ? std::vector<base::FieldRectangle>{{.8,-1,1.2,1}} : std::vector<base::FieldRectangle>{};
    });
    command.Schedule();scheduler.Run();scheduler.Run();
    EXPECT_EQ(scheduler.Requiring(&drive),command.get());
    EXPECT_GT(drive.RequestedSpeeds().vx.value(),0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    blocked=true;
    command.Schedule();scheduler.Run();scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    blocked=false;
    command.Schedule();scheduler.Run();scheduler.Run();
    EXPECT_GT(drive.RequestedSpeeds().vx.value(),0);
    throwing=true;scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    throwing=false;
    command.Schedule();scheduler.Run();scheduler.Run();
    EXPECT_GT(drive.RequestedSpeeds().vx.value(),0);
    command.Cancel();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    scheduler.CancelAll();
}

TEST(FootprintCommand, StalledRouteReleasesDriveAndStops) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    base::FootprintFollowConfig config; config.translationStallTimeout=.5;
    auto result=std::make_shared<base::FootprintFollowStatus>();
    auto command=base::FollowFootprintRoute(drive,{{0,0,0},{2,0,0}}, {.6,.4}, {-5,-5,5,5},
        [] {return std::vector<base::FieldRectangle>{};},config,result);
    command.Schedule();
    for (int i=0;i<80 && command.IsScheduled();++i) scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(*result,base::FootprintFollowStatus::stalled);
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    scheduler.CancelAll();
}
