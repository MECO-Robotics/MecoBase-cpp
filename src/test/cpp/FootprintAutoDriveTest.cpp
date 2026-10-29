#include "base/FootprintAutoDrive.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <chrono>
#include <cmath>
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

TEST(FootprintAutoDrive, ReplansWhenAnObstacleBlocksTheActiveRoute) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    bool blocked=false;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4};config.field={-5,-5,5,5};
    auto command=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[&] {
        return blocked ? std::vector<base::FieldRectangle>{{.8,-.4,1.2,.4}} : std::vector<base::FieldRectangle>{};
    },config);
    command.Schedule();
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    blocked=true;scheduler.Run();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    bool resumed=false;
    deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && command.IsScheduled()) {
        scheduler.Run();
        const auto speed=drive.RequestedSpeeds();
        if (std::hypot(speed.vx.value(),speed.vy.value())>.01) {resumed=true;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    EXPECT_TRUE(resumed);
    command.Cancel();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    scheduler.CancelAll();
}

TEST(FootprintAutoDrive, WaitsForEstimatedMotionToStopBeforePlanning) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll(); scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
    base::Drive drive;
    int snapshots=0;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4}; config.field={-5,-5,5,5};
    auto command=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[&] { ++snapshots; return std::vector<base::FieldRectangle>{}; },config);
    command.Schedule();
    for (int i=0;i<20;++i) {
        drive.ResetPose(frc::Pose2d{units::meter_t{.01*i},units::meter_t{0},frc::Rotation2d{}});
        scheduler.Run();
        EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_EQ(snapshots,0);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run(); std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_GT(snapshots,0);
    EXPECT_GT(drive.RequestedSpeeds().vx.value(),0);
    command.Cancel();

    snapshots=0; config.replanSettleTimeout=.15;
    auto timedOut=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[&] { ++snapshots; return std::vector<base::FieldRectangle>{}; },config);
    timedOut.Schedule();
    // The scheduler needs several subsequent cycles to unwind the command;
    // allow a loaded CI runner to execute them after the .15 s product timeout.
    const auto timeoutDeadline=std::chrono::steady_clock::now()+std::chrono::seconds{5};
    int i=0;
    while (timedOut.IsScheduled() && std::chrono::steady_clock::now()<timeoutDeadline) {
        drive.ResetPose(frc::Pose2d{units::meter_t{.01*i++},units::meter_t{0},frc::Rotation2d{}});
        scheduler.Run(); std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_FALSE(timedOut.IsScheduled());
    EXPECT_EQ(snapshots,0);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    scheduler.CancelAll();
}

TEST(FootprintAutoDrive, StalledRouteStopsWithoutRetrying) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4};config.field={-5,-5,5,5};
    config.follow.translationStallTimeout=.2;
    auto outcome=std::make_shared<base::FootprintAutoDriveOutcome>();
    auto command=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config,outcome);
    command.Schedule();
    bool wasMoving=false; int motionStarts=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (command.IsScheduled() && std::chrono::steady_clock::now()<deadline) {
        scheduler.Run();
        const bool moving=std::hypot(drive.RequestedSpeeds().vx.value(),drive.RequestedSpeeds().vy.value())>.01;
        if (moving && !wasMoving) ++motionStarts;
        wasMoving=moving;
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(motionStarts,1);
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::stalled);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    EXPECT_EQ(scheduler.Requiring(&drive),nullptr);
    command.Cancel(); scheduler.CancelAll();
}

TEST(FootprintAutoDrive, MovingTargetStopsAndReplansWhenEnabled) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive; double goalY=0;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4};config.field={-5,-5,5,5};
    config.targetTranslationReplanThreshold=.3;
    auto command=base::AutoDriveFootprintToPose(drive,[&]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{goalY},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config);
    command.Schedule();
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    goalY=1;scheduler.Run();
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vy.value(),0);
    // Rapidly alternating fresh targets must not repeatedly restart motion.
    for (int i=0;i<30;++i) {
        goalY=(i/3)%2 ? 1.5 : 1.;
        scheduler.Run();
        EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
        EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vy.value(),0);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    goalY=1;
    bool towardNewGoal=false;
    deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (command.IsScheduled() && std::chrono::steady_clock::now()<deadline) {
        scheduler.Run();
        if (drive.RequestedSpeeds().vy.value()>.02) {towardNewGoal=true;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    EXPECT_TRUE(towardNewGoal);
    command.Cancel();
    goalY=0;config.targetTranslationReplanThreshold=0;
    auto fixed=base::AutoDriveFootprintToPose(drive,[&]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{goalY},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config);
    fixed.Schedule();deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    goalY=1;scheduler.Run();
    EXPECT_GT(drive.RequestedSpeeds().vx.value(),0);
    EXPECT_NEAR(drive.RequestedSpeeds().vy.value(),0,1e-9);
    fixed.Cancel();scheduler.CancelAll();
}

TEST(FootprintAutoDrive, ReportsMissingTargetWithoutDriving) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;
    base::FootprintAutoDriveConfig config;config.footprint={.6,.4};config.field={-5,-5,5,5};
    auto outcome=std::make_shared<base::FootprintAutoDriveOutcome>();
    auto command=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {return std::nullopt;},
        [] {return std::vector<base::FieldRectangle>{};},config,outcome);
    command.Schedule();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (command.IsScheduled() && std::chrono::steady_clock::now()<deadline) {
        scheduler.Run(); std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::target_unavailable);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    command.Cancel();
    auto noRoute=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{{-4,-4,4,4}};},config,outcome);
    noRoute.Schedule();
    const auto noRouteDeadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (noRoute.IsScheduled() && std::chrono::steady_clock::now()<noRouteDeadline) {
        scheduler.Run(); std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    EXPECT_FALSE(noRoute.IsScheduled());
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::no_route);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    noRoute.Cancel();
    auto interrupted=base::AutoDriveFootprintToPose(drive,[]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{0},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config,outcome);
    interrupted.Schedule();scheduler.Run();interrupted.Cancel();
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::interrupted);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    scheduler.CancelAll();
}

TEST(FootprintAutoDrive, UnstableTargetTimesOutStopped) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;double goalY=0;
    base::FootprintAutoDriveConfig config;
    config.footprint={.6,.4};config.field={-5,-5,5,5};
    config.targetTranslationReplanThreshold=.3;config.targetStabilitySeconds=.2;config.targetStabilityTimeout=.7;
    auto outcome=std::make_shared<base::FootprintAutoDriveOutcome>();
    auto command=base::AutoDriveFootprintToPose(drive,[&]() -> std::optional<frc::Pose2d> {
        return frc::Pose2d{units::meter_t{2},units::meter_t{goalY},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config,outcome);
    command.Schedule();
    const auto startDeadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (std::chrono::steady_clock::now()<startDeadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    int i=0;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (command.IsScheduled() && std::chrono::steady_clock::now()<deadline) {
        goalY=(i++/3)%2 ? 1.5 : 1.;scheduler.Run();
        EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
        EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vy.value(),0);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::target_unstable);
    command.Cancel();scheduler.CancelAll();
}

TEST(FootprintAutoDrive, TargetWaitDeadlineWinsAfterSchedulerPause) {
    auto& scheduler=frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();
    base::Drive drive;double goalY=0;int changedTargetReads=0;
    base::FootprintAutoDriveConfig config;config.footprint={.6,.4};config.field={-5,-5,5,5};
    config.targetTranslationReplanThreshold=.3;config.targetStabilitySeconds=.2;config.targetStabilityTimeout=.3;
    auto outcome=std::make_shared<base::FootprintAutoDriveOutcome>();
    auto command=base::AutoDriveFootprintToPose(drive,[&]() -> std::optional<frc::Pose2d> {
        if (goalY!=0) ++changedTargetReads;
        return frc::Pose2d{units::meter_t{2},units::meter_t{goalY},frc::Rotation2d{}};
    },[] {return std::vector<base::FieldRectangle>{};},config,outcome);
    command.Schedule();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds{5};
    while (std::chrono::steady_clock::now()<deadline && drive.RequestedSpeeds().vx.value()<=0) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GT(drive.RequestedSpeeds().vx.value(),0);
    goalY=1;
    const auto waitStartDeadline=std::chrono::steady_clock::now()+std::chrono::seconds{5};
    while (command.IsScheduled() && changedTargetReads<3 && std::chrono::steady_clock::now()<waitStartDeadline) {
        scheduler.Run();std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    ASSERT_GE(changedTargetReads,3);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    std::this_thread::sleep_for(std::chrono::milliseconds{400});
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds{2};
    while (command.IsScheduled() && std::chrono::steady_clock::now()<end) scheduler.Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_EQ(*outcome,base::FootprintAutoDriveOutcome::target_unstable);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    command.Cancel();scheduler.CancelAll();
}
