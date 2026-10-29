#include "base/FootprintTerrainMap.h"
#include <gtest/gtest.h>
#include <limits>

TEST(FootprintTerrainMap, SplitsNarrowRegionsAndProjectsGradeByTravelDirection) {
    base::FootprintTerrainMap map({{{1,0,2,2},.1,.2,.03}});
    const auto cuts=map.Boundaries({0,1,0},{3,1,0});
    ASSERT_EQ(cuts.size(),2u);
    EXPECT_NEAR(cuts[0],1./3,1e-9);
    EXPECT_NEAR(cuts[1],2./3,1e-9);
    EXPECT_THROW(map.Segment({0,1,0},{3,1,0}),std::invalid_argument);
    const auto uphill=map.Segment({1,1,0},{2,1,0});
    const auto downhill=map.Segment({2,1,0},{1,1,0});
    EXPECT_NEAR(uphill.grade,.1,1e-9);
    EXPECT_NEAR(downhill.grade,-.1,1e-9);
    EXPECT_NEAR(uphill.rollingResistance,.03,1e-9);
    EXPECT_DOUBLE_EQ(map.Segment({0,1,0},{1,1,0}).grade,0);
}
TEST(FootprintTerrainMap, RejectsInvalidAndOverlappingRegions) {
    EXPECT_THROW(base::FootprintTerrainMap({{{0,0,1,1},0,0,-.1}}),std::invalid_argument);
    EXPECT_THROW(base::FootprintTerrainMap({{{0,0,2,2},0,0,0},{{1,1,3,3},0,0,0}}),std::invalid_argument);
    base::FootprintTerrainMap map({{{0,0,1,1},0,0,0},{{1,0,2,1},0,0,0}});
    EXPECT_THROW(map.Boundaries({std::numeric_limits<double>::quiet_NaN(),0,0},{1,1,0}),std::invalid_argument);
    EXPECT_THROW(map.Segment({1,.2,0},{1,.8,0}),std::invalid_argument);
}
