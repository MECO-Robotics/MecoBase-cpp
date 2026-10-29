#include "base/FootprintClearance.h"
#include <gtest/gtest.h>
#include <array>
#include <limits>
#include <numbers>

TEST(FootprintClearance, OrientationChangesPassageAndRotationSweepsCorners) {
    const base::FieldRectangle field{0,0,5,5};
    const std::array obstacles{base::FieldRectangle{1,1,4,1.6}, base::FieldRectangle{1,2.4,4,3}};
    const base::RectangularFootprint robot{1,.6};
    EXPECT_TRUE(base::SweptFootprintClear({1.5,2,0},{3.5,2,0},robot,field,obstacles));
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
