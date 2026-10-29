#include "base/FootprintClearance.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

TEST(FootprintClearance, OrientationChangesPassageAndRotationSweepsCorners) {
    const base::FieldRectangle field{0,0,5,5};
    const std::array obstacles{base::FieldRectangle{1,1,4,1.6}, base::FieldRectangle{1,2.4,4,3}};
    const base::RectangularFootprint robot{1,.6};
    EXPECT_TRUE(base::SweptFootprintClear({1.5,2,0},{3.5,2,0},robot,field,obstacles));
    EXPECT_FALSE(base::SweptFootprintClear({1.5,2,0},{3.5,2,0},{1,.9},field,obstacles));
    EXPECT_FALSE(base::SweptFootprintClear({1.5,2,std::numbers::pi/2},{3.5,2,std::numbers::pi/2},robot,field,obstacles));
    EXPECT_FALSE(base::SweptFootprintClear({2,2,0},{2,2,std::numbers::pi},robot,field,obstacles));
}

TEST(FootprintClearance, RejectsCrossedObstaclesFieldEdgesAndInvalidGeometry) {
    const base::FieldRectangle field{0,0,5,5};
    const std::array thin{base::FieldRectangle{2.48,1,2.52,4}};
    const base::RectangularFootprint robot{.5,.5};
    EXPECT_FALSE(base::SweptFootprintClear({1,2,0},{4,2,0},robot,field,thin));
    EXPECT_FALSE(base::SweptFootprintClear({.24,2,0},{1,2,0},robot,field,{}));
    EXPECT_FALSE(base::SweptFootprintClear({1,2,0},{2,2,0},{-1,.5},field,{}));
    EXPECT_FALSE(base::SweptFootprintClear({1,2,0},{std::numeric_limits<double>::quiet_NaN(),2,0},robot,field,{}));
}

TEST(FootprintClearance, FindsRouteThatTurnsToFitAndRejectsClosedPassage) {
    const base::FieldRectangle field{0,0,6,4};
    const std::array walls{base::FieldRectangle{1.5,1,4.5,1.6}, base::FieldRectangle{1.5,2.4,4.5,3}};
    const base::RectangularFootprint robot{1,.6};
    const base::FootprintPose start{1,2,std::numbers::pi/2}, goal{5,2,std::numbers::pi/2};
    auto route=base::FindFootprintRoute(start,goal,robot,field,walls,.2,30000);
    ASSERT_GE(route.size(),2u);
    for (std::size_t i=1;i<route.size();++i)
        EXPECT_TRUE(base::SweptFootprintClear(route[i-1],route[i],robot,field,walls));
    bool turned=false;
    for (const auto& p:route) if (std::abs(std::sin(p.heading))<.5) turned=true;
    EXPECT_TRUE(turned);
    const std::array closed{base::FieldRectangle{2.5,0,3.5,4}};
    EXPECT_TRUE(base::FindFootprintRoute(start,goal,robot,field,closed,.2,30000).empty());
}

TEST(FootprintClearance, DirectMotionDoesNotRequireGridCenterSnap) {
    const base::FieldRectangle field{0,0,5,5};
    const base::RectangularFootprint robot{.4,.4};
    const std::array wall{base::FieldRectangle{.45,.5,.7,2.5}};
    const auto route=base::FindFootprintRoute({.23,1,0},{.23,2,0},robot,field,wall,.5,1);
    EXPECT_EQ(route.size(),2u);
}

TEST(FootprintClearance, CancelledSearchDoesNotReturnAPath) {
    std::stop_source stop;
    stop.request_stop();
    EXPECT_TRUE(base::FindFootprintRoute({1,1,0},{2,1,0},{.5,.5},
        {0,0,5,5},{},.2,100,stop.get_token()).empty());
}

TEST(FootprintClearance, ReplansFromClearPoseWhenContainingCellCenterIsBlocked) {
    const base::FieldRectangle field{0,0,16.541,8.069};
    const std::array support{base::FieldRectangle{4.6256194-53*.0254/2,50.34*.0254,
                                                 4.6256194+53*.0254/2,(50.34+12)*.0254}};
    const base::RectangularFootprint robot{.86,.86};
    const base::FootprintPose start{3.5148289529,.8095564654,-.0112862046}, goal{3.5,2.4,0};
    ASSERT_TRUE(base::SweptFootprintClear(start,start,robot,field,support));
    ASSERT_FALSE(base::SweptFootprintClear(start,goal,robot,field,support));
    const auto route=base::FindFootprintRoute(start,goal,robot,field,support,.2,30000);
    ASSERT_GE(route.size(),2u);
    for (std::size_t i=1;i<route.size();++i)
        EXPECT_TRUE(base::SweptFootprintClear(route[i-1],route[i],robot,field,support));
}
