#include "base/PieceDetection.h"
#include <gtest/gtest.h>
#include <hal/HAL.h>
#include <frc/RobotController.h>
#include <frc/simulation/SimHooks.h>
using namespace units::literals;
TEST(PieceDetection, MountFieldTransformFreshnessAndNoTarget)
{
    HAL_Initialize(500, 0);
    frc::sim::PauseTiming();
    frc::sim::StepTimingAsync(.1_s);
    base::PieceDetectionConfig config;
    config.robotToCamera = {frc::Translation3d{1_m, 0_m, 1_m}, frc::Rotation3d{0_rad, 45_deg, 90_deg}};
    std::optional<base::PieceRay> ray =
        base::PieceRay{units::second_t{frc::RobotController::GetFPGATime() * 1e-6}, 0, 0};
    frc::Pose3d robotPose{2_m, 3_m, 0_m, frc::Rotation3d{0_rad, 0_rad, 90_deg}};
    base::PhotonPieceDetection detection(config, [&] { return robotPose; }, [&] { return ray; });
    detection.Periodic();
    ASSERT_TRUE(detection.Target());
    EXPECT_NEAR(detection.Target()->X().value(), 1, 1e-9);
    EXPECT_NEAR(detection.Target()->Y().value(), 4, 1e-9);
    robotPose = frc::Pose3d{5_m, 6_m, 0_m, frc::Rotation3d{}};
    frc::sim::StepTimingAsync(.02_s);
    detection.Periodic();
    ASSERT_TRUE(detection.Target());
    EXPECT_NEAR(detection.Target()->X().value(), 1, 1e-9);
    EXPECT_NEAR(detection.Target()->Y().value(), 4, 1e-9);
    ray->pitchDegrees = 45;
    detection.Periodic();
    EXPECT_FALSE(detection.Target());
    ray->pitchDegrees = 0;
    frc::sim::StepTimingAsync(.2_s);
    detection.Periodic();
    EXPECT_FALSE(detection.Target());
    ray.reset();
    detection.Periodic();
    EXPECT_FALSE(detection.Target());
    frc::sim::ResumeTiming();
}
