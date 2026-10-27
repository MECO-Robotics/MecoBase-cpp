#include "base/DriveWheelRadius.h"
#include <gtest/gtest.h>
#include <numbers>
#include <cmath>
#include <limits>
TEST(DriveWheelRadius, KnownRadiusUsesEachModuleLocationAndUnwrapsSignedYaw)
{
    std::array<double, 4> distances{.3, .4, .5, .6};
    base::WheelRadiusMeasurement measurement(distances);
    for (int i = 0; i <= 800; ++i)
    {
        const double yaw = i * .01;
        std::array<double, 4> angles{};
        for (int m = 0; m < 4; ++m)
            angles[m] = 10 + (m % 2 ? -1 : 1) * yaw * distances[m] / .052;
        ASSERT_TRUE(measurement.Add(i * .02, std::remainder(yaw + 3, 2 * std::numbers::pi), angles, true));
    }
    ASSERT_TRUE(measurement.Valid());
    for (double radius : measurement.Radii())
        EXPECT_NEAR(radius, .052, 1e-12);
}
TEST(DriveWheelRadius, RejectsInsufficientTravelStaleFeedbackAndUnequalWheelEstimates)
{
    std::array<double, 4> distances{.4, .4, .4, .4};
    base::WheelRadiusMeasurement shortRun(distances), stale(distances), unequal(distances), zero(distances);
    EXPECT_TRUE(shortRun.Add(0, 0, {0, 0, 0, 0}, true));
    EXPECT_TRUE(shortRun.Add(.02, .01, {.08, .08, .08, .08}, true));
    EXPECT_FALSE(shortRun.Valid());
    for (int i = 2; i <= 700; ++i)
        ASSERT_TRUE(shortRun.Add(i * .02, i % 2 ? .01 : 0, {.08, .08, .08, .08}, true));
    EXPECT_NEAR(shortRun.YawTravel(), 0, 1e-12);
    EXPECT_FALSE(shortRun.Valid());
    EXPECT_TRUE(stale.Add(0, 0, {0, 0, 0, 0}, true));
    EXPECT_FALSE(stale.Add(.2, .1, {1, 1, 1, 1}, true));
    EXPECT_FALSE(stale.Add(.22, .11, {1.1, 1.1, 1.1, 1.1}, true));
    for (int i = 0; i <= 700; ++i)
    {
        const double yaw = i * .01;
        ASSERT_TRUE(unequal.Add(i * .02, yaw, {yaw * 8, yaw * 8, yaw * 8, yaw * 4}, true));
        ASSERT_TRUE(zero.Add(i * .02, yaw, {0, 0, 0, 0}, true));
    }
    EXPECT_FALSE(unequal.Valid());
    EXPECT_FALSE(zero.Valid());
    EXPECT_FALSE(shortRun.Add(14.02, std::numeric_limits<double>::quiet_NaN(), {0, 0, 0, 0}, true));
}
