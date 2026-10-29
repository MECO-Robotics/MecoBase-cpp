#include "base/FootprintAutoDrive.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <chrono>
#include <thread>

TEST(FootprintAutoDrive, PlansOffThreadOwnsDriveAndStopsWhenTargetExpires) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    bool fresh=true;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4}; config.field={-5,-5,5,5};
    auto command=base::AutoDriveFootprintToPose(drive,[&]() -> std::optional<frc::Pose2d> {
        if (!fresh) return std::nullopt;
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config);
    command.Schedule();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    EXPECT_EQ(scheduler.Requiring(&drive),command.get());
    fresh=false;scheduler.Run();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    for (int i=0;i<5 && command.IsScheduled();++i) scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    scheduler.CancelAll();
}
