#include "base/AutoDrive.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <limits>

TEST(AutoDrive, RejectsInvalidGoalsAndMissingTargetOwnsAndStopsDrive) {
    base::AutoDriveConfig config;
    EXPECT_FALSE(base::ValidAutoDriveTarget({units::meter_t{-1},units::meter_t{1},frc::Rotation2d{}},config));
    EXPECT_FALSE(base::ValidAutoDriveTarget({units::meter_t{std::numeric_limits<double>::quiet_NaN()},units::meter_t{1},frc::Rotation2d{}},config));
    EXPECT_TRUE(base::ValidAutoDriveTarget({units::meter_t{3},units::meter_t{1},frc::Rotation2d{}},config));
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll(); scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
    base::Drive drive;
    auto command=base::AutoDriveToPose(drive, [] { return std::optional<frc::Pose2d>{}; });
    drive.RunVelocity({units::meters_per_second_t{1},units::meters_per_second_t{0},units::radians_per_second_t{0}});
    command.Schedule(); scheduler.Run();
    EXPECT_EQ(scheduler.Requiring(&drive),command.get());
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    const auto zero=pathplanner::DriveFeedforwards::zeros(4);
    const frc::ChassisSpeeds fast{units::meters_per_second_t{6},units::meters_per_second_t{0},units::radians_per_second_t{0}};
    drive.RunVelocity(fast,zero);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),2);
    scheduler.Run();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData(); scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    drive.RunVelocity(fast,zero);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),6); // Disable releases the scoped limit.
    drive.Stop();
}

TEST(AutoDrive, LimitsPathFeedbackWithoutChangingOtherDriveCommands) {
    base::Drive drive;
    const auto zero=pathplanner::DriveFeedforwards::zeros(4);
    const frc::ChassisSpeeds fast{units::meters_per_second_t{6},units::meters_per_second_t{8},
                                  units::radians_per_second_t{5}};
    drive.SetPathOutputLimits(2,3);
    drive.RunVelocity(fast,zero);
    auto limited=drive.RequestedSpeeds();
    EXPECT_NEAR(std::hypot(limited.vx.value(),limited.vy.value()),2,1e-9);
    EXPECT_DOUBLE_EQ(limited.omega.value(),3);
    drive.RunVelocity(fast);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),6);
    drive.ClearPathOutputLimits();
    drive.RunVelocity(fast,zero);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),6);
}

#include "../../main/cpp/base/drive/NativeADStar.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
TEST(AutoDrive, RepeatedAndSubcellGoalsProduceFreshPaths) {
    using namespace units::literals;
    const auto grid=std::filesystem::temp_directory_path()/
        ("meco-clear-navgrid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
    {
        std::ofstream out(grid);
        out << R"({"nodeSizeMeters":0.2,"field_size":{"x":5,"y":5},"grid":[)";
        for (int row=0; row<25; ++row) {
            if (row) out << ',';
            out << '[';
            for (int col=0; col<25; ++col) { if (col) out << ','; out << "false"; }
            out << ']';
        }
        out << "]}";
    }
    base::detail::NativeADStar planner(grid.string());
    const pathplanner::PathConstraints limits{2_mps,3_mps_sq,3_rad_per_s,6_rad_per_s_sq};
    auto request=[&](double goal) {
        planner.setStartPosition({2.02_m,2.02_m});
        planner.setGoalPosition({units::meter_t{goal},2.02_m});
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds{2};
        while(std::chrono::steady_clock::now()<until) {
            if(planner.isNewPathAvailable()) {
                auto path=planner.getCurrentPath(limits,pathplanner::GoalEndState{0_mps,frc::Rotation2d{}});
                if(path && std::abs(path->getPoint(path->numPoints()-1).position.X().value()-goal)<1e-6) return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
        }
        return false;
    };
    ASSERT_TRUE(request(4.02));
    ASSERT_TRUE(request(4.06)); // Same grid cell; must update precise endpoint.
    ASSERT_TRUE(request(4.06)); // An identical request must also restart after completion.
    std::filesystem::remove(grid);
}

TEST(AutoDrive, ShortApproachChecksStaticAndDynamicGridObstacles) {
    using namespace units::literals;
    const auto path=std::filesystem::temp_directory_path()/
        ("meco-navgrid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
    {
        std::ofstream out(path);
        out << R"({"nodeSizeMeters":1.0,"field_size":{"x":5.0,"y":5.0},"grid":[[false,true,false,false,false],[false,false,false,false,false],[false,false,false,false,false],[false,false,false,false,false],[false,false,false,false,false]]})";
    }
    {
        base::detail::NativeADStar planner(path.string());
        EXPECT_TRUE(planner.DirectSegmentClear({.2_m,.2_m},{.7_m,.2_m}));
        EXPECT_FALSE(planner.DirectSegmentClear({.2_m,.2_m},{1.2_m,.2_m}));
        // A diagonal cannot cut the blocked corner at cell (1, 0).
        EXPECT_FALSE(planner.DirectSegmentClear({.9_m,.9_m},{1.1_m,1.1_m}));
        EXPECT_TRUE(planner.DirectSegmentClear({.2_m,1.2_m},{.7_m,1.2_m}));
        planner.setDynamicObstacles({{{.3_m,1.3_m},{.4_m,1.4_m}}},{.2_m,1.2_m});
        EXPECT_FALSE(planner.DirectSegmentClear({.2_m,1.2_m},{.7_m,1.2_m}));
        EXPECT_FALSE(planner.DirectSegmentClear({-.1_m,1.2_m},{.7_m,1.2_m}));
    }
    std::filesystem::remove(path);
    base::detail::NativeADStar missing(path.string());
    EXPECT_FALSE(missing.DirectSegmentClear({.2_m,.2_m},{.3_m,.2_m}));
}

TEST(AutoDrive, MissingOrMalformedGridNeverProducesAPath) {
    using namespace units::literals;
    const auto path=std::filesystem::temp_directory_path()/
        ("meco-invalid-navgrid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
    const pathplanner::PathConstraints limits{2_mps,3_mps_sq,3_rad_per_s,6_rad_per_s_sq};
    auto check=[&] {
        base::detail::NativeADStar planner(path.string());
        planner.setStartPosition({1_m,1_m});
        planner.setGoalPosition({4_m,1_m});
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
        EXPECT_EQ(planner.getCurrentPath(limits,pathplanner::GoalEndState{0_mps,frc::Rotation2d{}}),nullptr);
    };
    check(); // Absent file must not be treated as an empty field.
    {
        std::ofstream out(path);
        out << R"({"nodeSizeMeters":0.2,"grid":[[false]]})"; // field_size is required
    }
    check();
    {
        std::ofstream out(path);
        out << R"({"nodeSizeMeters":0,"field_size":{"x":5,"y":5},"grid":[[false]]})";
    }
    check(); // Invalid resolution cannot enter grid-coordinate arithmetic.
    {
        std::ofstream out(path);
        out << R"({"nodeSizeMeters":1,"field_size":{"x":5,"y":5},"grid":[[false]]})";
    }
    check(); // The declared field must fit inside the supplied grid.
    std::filesystem::remove(path);
}

TEST(AutoDrive, TrackingStopsForChangedOrExpiredTargetAndReleasesDrive) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll(); scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
    base::Drive drive;
    std::optional<frc::Pose2d> goal=frc::Pose2d{units::meter_t{.1},units::meter_t{.1},frc::Rotation2d{}};
    int calls=0;
    base::AutoDriveTrackingConfig tracking;
    tracking.stabilitySeconds=.05;
    auto command=base::AutoDriveTrackTarget(drive,[&] {++calls;return goal;},{},tracking);
    command.Schedule();
    ASSERT_TRUE(command.IsScheduled());
    goal=frc::Pose2d{units::meter_t{1},units::meter_t{1},frc::Rotation2d{}};
    scheduler.Run();
    EXPECT_TRUE(command.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive),command.get());
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    goal.reset();
    for (int i=0;i<10 && command.IsScheduled();++i) scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    EXPECT_GT(calls,1);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
}
