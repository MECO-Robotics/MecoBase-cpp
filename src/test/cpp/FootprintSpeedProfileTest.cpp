#include "base/FootprintSpeedProfile.h"
#include <gtest/gtest.h>
#include <limits>

TEST(FootprintSpeedProfile, TerrainAndTurnsConstrainFeasibleSpeeds) {
    const std::vector<base::FootprintPose> route{{0,0,0},{1,0,0},{2,0,0},{3,1,0},{4,1,0}};
    const base::FootprintProfileLimits limits{4,3,3,1,1};
    std::vector<base::FootprintTerrainSegment> flat(4), uphill(4);
    uphill[0]={.15,.02};
    const auto a=base::FootprintSpeedProfile(route,flat,limits,0);
    const auto b=base::FootprintSpeedProfile(route,uphill,limits,0);
    ASSERT_EQ(a.size(),route.size()); ASSERT_EQ(b.size(),route.size());
    EXPECT_LT(b[1],a[1]);
    EXPECT_LT(a[2],limits.maxSpeed);
    EXPECT_DOUBLE_EQ(a.back(),0);
}

TEST(FootprintSpeedProfile, RejectsUnstoppableAndInvalidInputs) {
    const std::vector<base::FootprintPose> route{{0,0,0},{.1,0,0}};
    const std::vector<base::FootprintTerrainSegment> terrain(1);
    EXPECT_TRUE(base::FootprintSpeedProfile(route,terrain,{},2).empty());
    EXPECT_TRUE(base::FootprintSpeedProfile(route,{}, {},0).empty());
    auto invalid=terrain; invalid[0].grade=std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(base::FootprintSpeedProfile(route,invalid,{},0).empty());
}
