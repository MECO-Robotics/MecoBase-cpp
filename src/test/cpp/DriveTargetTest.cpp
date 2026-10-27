#include "base/DriveTarget.h"
#include <gtest/gtest.h>
#include <limits>
using namespace units::literals;
TEST(DriveTarget, DisabledCannotResurrectTargetAndStaleCommandsStop)
{
    base::DriveTarget target;
    base::DriveTarget::Value value;
    value.speeds.vx = 2_mps;
    value.timestamp = 1;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(1.02, true).speeds.vx, 2_mps);
    EXPECT_EQ(target.Consume(1.101, true).speeds.vx, 0_mps);
    EXPECT_EQ(target.Consume(1.03, false).speeds.vx, 0_mps);
    EXPECT_EQ(target.Consume(1.04, true).speeds.vx, 0_mps);
    value.timestamp = 1.04;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(1.05, true).speeds.vx, 2_mps);
}
TEST(DriveTarget, ForcesAreCoherentAndInvalidInputStops)
{
    base::DriveTarget target;
    base::DriveTarget::Value value;
    value.speeds.vx = 1_mps;
    value.forceX = {1, 2, 3, 4};
    value.timestamp = 2;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(2, true).forceX, value.forceX);
    value.forceY[1] = std::numeric_limits<double>::quiet_NaN();
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(2, true).speeds.vx, 0_mps);
    value.forceY[1] = 0;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(1.99, true).speeds.vx, 0_mps);
    value.timestamp = std::numeric_limits<double>::quiet_NaN();
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(2, true).speeds.vx, 0_mps);
    value.timestamp = 2;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(std::numeric_limits<double>::quiet_NaN(), true).speeds.vx, 0_mps);
}
TEST(DriveTarget, HeadingFeedbackUsesLatestPoseAndPreservesJavaProportionalLaw)
{
    base::DriveTarget::Value value;
    value.headingControl = value.fieldRelative = true;
    value.heading = frc::Rotation2d{170_deg};
    value.headingKP = 5;
    value.maxOmega = 7;
    value.speeds.vx = 1_mps;
    auto resolved = base::DriveTarget::Resolve(value, frc::Rotation2d{-170_deg});
    EXPECT_NEAR(resolved.omega.value(), -20 * std::acos(-1) / 180 * 5, 1e-12);
    EXPECT_NEAR(resolved.vx.value(), std::cos(-170 * std::acos(-1) / 180), 1e-12);
    EXPECT_NEAR(base::DriveTarget::Resolve(value, frc::Rotation2d{0_deg}).omega.value(), 7, 1e-12);
    const auto invalidHeading =
        base::DriveTarget::Resolve(value, frc::Rotation2d{units::radian_t{std::numeric_limits<double>::quiet_NaN()}});
    EXPECT_DOUBLE_EQ(invalidHeading.vx.value(), 0);
    EXPECT_DOUBLE_EQ(invalidHeading.vy.value(), 0);
    EXPECT_DOUBLE_EQ(invalidHeading.omega.value(), 0);
}
TEST(DriveTarget, StationaryBrakeLatchesUntilDisabledOrReplaced)
{
    base::DriveTarget target;
    base::DriveTarget::Value value;
    value.brake = true;
    value.timestamp = 1;
    target.Publish(value, [] { return true; });
    EXPECT_TRUE(target.Consume(3, true).brake);
    EXPECT_FALSE(target.Consume(3, false).brake);
    EXPECT_FALSE(target.Consume(3, true).brake);
}

TEST(DriveTarget, DisableRacingAuthorizationInvalidatesOldEpoch)
{
    base::DriveTarget target;
    base::DriveTarget::Value value;
    value.timestamp = 1;
    value.speeds.vx = 2_mps;
    target.Publish(value, [&] {
        target.Consume(1, false);
        return true;
    });
    EXPECT_EQ(target.Consume(1.01, true).speeds.vx, 0_mps);
    target.Publish(value, [] { return false; });
    EXPECT_EQ(target.Consume(1.02, true).speeds.vx, 0_mps);
}

TEST(DriveTarget, RobotThreadDisableInvalidatesWithoutNativeDisabledTick)
{
    base::DriveTarget target;
    base::DriveTarget::Value value;
    value.timestamp = 1;
    value.speeds.vx = 2_mps;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(1, true).speeds.vx, 2_mps);
    // Stop publication while disabled; a fast mode transition may occur between
    // native callbacks. A prior enabled packet must not survive that transition.
    target.Publish({}, [] { return false; });
    EXPECT_EQ(target.Consume(1.01, true).speeds.vx, 0_mps);
    value.timestamp = 1.02;
    target.Publish(value, [] { return true; });
    EXPECT_EQ(target.Consume(1.02, true).speeds.vx, 2_mps);
}
