#include "base/Drive.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/HAL.h>
#include <cstdlib>
#include <cmath>
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
