#include "base/Drive.h"
#include "base/LiveDriveMotorTuning.h"
#include <networktables/NetworkTableInstance.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/HAL.h>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <stdexcept>

TEST(DriveOwnership, AimPreemptsJoystickAndDisableCancelsOwner)
{
    if (!std::getenv("FRC_DRIVE_OWNERSHIP_CHILD"))
    {
#ifdef _WIN32
        _putenv_s("FRC_DRIVE_OWNERSHIP_CHILD", "1");
#else
        setenv("FRC_DRIVE_OWNERSHIP_CHILD", "1", 1);
#endif
        const int result = std::system(("\"" + ::testing::internal::GetArgvs()[0] +
                                        "\" --gtest_filter=DriveOwnership.AimPreemptsJoystickAndDisableCancelsOwner")
                                           .c_str());
#ifdef _WIN32
        _putenv_s("FRC_DRIVE_OWNERSHIP_CHILD", "");
#else
        unsetenv("FRC_DRIVE_OWNERSHIP_CHILD");
#endif
        ASSERT_EQ(result, 0);
        return;
    }
    HAL_Initialize(500, 0);
    auto &scheduler = frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.Enable();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetEStop(false);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();

    base::Drive drive;
    drive.ConfigureAuto();
    drive.SetAutoGains(8, 6);
    EXPECT_DOUBLE_EQ(drive.Config().autoTranslationKP, 8);
    EXPECT_DOUBLE_EQ(drive.Config().autoRotationKP, 6);
    EXPECT_THROW(drive.SetAutoGains(-1, 6), std::invalid_argument);
    EXPECT_DOUBLE_EQ(drive.Config().autoTranslationKP, 8);
    base::LiveDriveMotorTuning motorTuning(drive, true);
    auto motorGains = drive.MotorGains();
    motorGains.driveP += .1;
    motorGains.driveI = .02;
    motorGains.driveA = .03;
    auto gainTable = nt::NetworkTableInstance::GetDefault();
    gainTable.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kP").SetDouble(motorGains.driveP);
    gainTable.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kI").SetDouble(motorGains.driveI);
    gainTable.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kA").SetDouble(motorGains.driveA);
    motorTuning.Periodic();
    EXPECT_EQ(drive.MotorGainState(), base::DriveGainState::Pending);
    drive.RunVelocity({units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(), 0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (drive.MotorGainState() == base::DriveGainState::Pending && std::chrono::steady_clock::now() < deadline)
    {
        drive.Periodic();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(drive.MotorGainState(), base::DriveGainState::Ready);
    EXPECT_DOUBLE_EQ(drive.MotorGains().driveP, motorGains.driveP);
    ctre::phoenix6::hardware::TalonFX probe{drive.Config().modules[0].drive, drive.Config().bus};
    ctre::phoenix6::configs::Slot0Configs actual;
    ASSERT_TRUE(probe.GetConfigurator().Refresh(actual, units::second_t{.25}).IsOK());
    EXPECT_NEAR(actual.kP, motorGains.driveP, 1e-5);
    EXPECT_NEAR(actual.kI, motorGains.driveI, 1e-5);
    EXPECT_NEAR(actual.kA, motorGains.driveA, 1e-5);
    motorTuning.Periodic();
    auto nt = nt::NetworkTableInstance::GetDefault();
    auto status = nt.GetEntry("/Tuning/Drive/MotorStatus");
    EXPECT_EQ(status.GetString(""), "applied");
    auto p = nt.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kP");
    p.SetDouble(-1);
    motorTuning.Periodic();
    EXPECT_EQ(status.GetString(""), "rejected: invalid gains");
    EXPECT_DOUBLE_EQ(drive.MotorGains().driveP, motorGains.driveP);
    p.SetDouble(motorGains.driveP);
    motorTuning.Periodic();
    drive.SetRuntimeLimits(1, 2);
    drive.RunVelocity({units::meters_per_second_t{2}, units::meters_per_second_t{0}, units::radians_per_second_t{3}});
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(), 1);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 2);
    EXPECT_THROW(drive.SetRuntimeLimits(100, 2), std::invalid_argument);
    drive.RunVelocity({units::meters_per_second_t{std::numeric_limits<double>::quiet_NaN()},
                       units::meters_per_second_t{0}, units::radians_per_second_t{0}});
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(), 0);
    drive.SetRuntimeLimits(drive.Config().maxSpeed, drive.Config().maxAngularRate);
    auto path = std::make_shared<pathplanner::PathPlannerPath>(
        pathplanner::PathPlannerPath::waypointsFromPoses({frc::Pose2d{},
            frc::Pose2d{units::meter_t{2}, units::meter_t{0}, frc::Rotation2d{}}}),
        pathplanner::PathConstraints{units::meters_per_second_t{1}, units::meters_per_second_squared_t{1},
                                    units::radians_per_second_t{2}, units::radians_per_second_squared_t{2}},
        std::nullopt, pathplanner::GoalEndState{units::meters_per_second_t{0}, frc::Rotation2d{}});
    auto follow = pathplanner::AutoBuilder::followPath(path);
    scheduler.Schedule(follow);
    ASSERT_TRUE(follow.IsScheduled());
    drive.SetAutoGains(9, 7);
    EXPECT_NO_THROW(scheduler.Run());
    follow.Cancel();
    auto joystick = drive.Joystick([] { return .5; }, [] { return 0.; }, [] { return 0.; });
    auto aim = drive.Aim([] { return 0.; }, [] { return 0.; }, [] { return frc::Rotation2d{}; });
    scheduler.Schedule(joystick.get());
    scheduler.Run();
    EXPECT_EQ(scheduler.Requiring(&drive), joystick.get());
    EXPECT_TRUE(joystick.IsScheduled());

    scheduler.Schedule(aim.get());
    scheduler.Run();
    EXPECT_FALSE(joystick.IsScheduled());
    EXPECT_TRUE(aim.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive), aim.get());

    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
    scheduler.Run();
    EXPECT_FALSE(aim.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(&drive), nullptr);
    int resets = 0;
    const frc::Pose2d resetPose{units::meter_t{1}, units::meter_t{2}, frc::Rotation2d{}};
    drive.SetPoseResetCallback([&](const frc::Pose2d &pose) {
        ++resets;
        EXPECT_EQ(pose, resets == 3 ? frc::Pose2d{} : resetPose);
        (void)drive.Pose(); // Observer can read state without a held snapshot lock.
    });
    drive.ResetPose(resetPose);
    EXPECT_EQ(resets, 1);
    drive.RunHeading(1., -.5, frc::Rotation2d{units::degree_t{180}}, 5., 1.);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(), 1.);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vy.value(), -.5);
    EXPECT_LE(std::abs(drive.RequestedSpeeds().omega.value()), 1.);
    EXPECT_THROW(drive.RunHeading(0, 0, frc::Rotation2d{}, -1, 1), std::invalid_argument);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 0.);
    drive.Stop();
    // Motion/neutral-output timing belongs to the running native simulator test;
    // this regression checks real command ownership and cancellation only.
    scheduler.CancelAll();
}
