#include "base/FootprintFollower.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <numbers>

TEST(FootprintFollower, FollowsTranslationAndInPlaceTurnThenSettles) {
    const base::FieldRectangle field{0,0,5,5};
    const base::RectangularFootprint robot{.6,.4};
    base::FootprintFollower follower({{1,1,0},{2,1,0},{2,1,std::numbers::pi/2}},robot,field);
    base::FootprintPose pose{1,1,0};
    base::FieldVelocity measured{};
    base::FootprintFollowOutput output{};
    for (int i=0;i<1200;++i) {
        output=follower.Update(pose,measured,{},.02);
        if (output.status!=base::FootprintFollowStatus::running) break;
        EXPECT_LE(std::hypot(output.velocity.vx,output.velocity.vy),1.000001);
        measured=output.velocity;
        pose.x+=measured.vx*.02;pose.y+=measured.vy*.02;pose.heading+=measured.omega*.02;
    }
    EXPECT_EQ(output.status,base::FootprintFollowStatus::arrived);
    EXPECT_NEAR(pose.x,2,.04);
    EXPECT_NEAR(pose.heading,std::numbers::pi/2,.05);
    EXPECT_DOUBLE_EQ(output.velocity.omega,0);
}

TEST(FootprintFollower, NewObstacleOrInvalidFeedbackLatchesStop) {
    const base::FieldRectangle field{0,0,5,5};
    const base::RectangularFootprint robot{.6,.4};
    base::FootprintFollower follower({{1,1,0},{4,1,0}},robot,field);
    EXPECT_EQ(follower.Update({1,1,0},{},{},.02).status,base::FootprintFollowStatus::running);
    const std::array obstacle{base::FieldRectangle{2,0,2.2,2}};
    const auto stopped=follower.Update({1,1,0},{},obstacle,.02);
    EXPECT_EQ(stopped.status,base::FootprintFollowStatus::blocked);
    EXPECT_DOUBLE_EQ(stopped.velocity.vx,0);
    EXPECT_EQ(follower.Update({1,1,0},{},{},.02).status,base::FootprintFollowStatus::blocked);
    base::FootprintFollower invalid({{1,1,0},{4,1,0}},robot,field);
    EXPECT_EQ(invalid.Update({1,1,0},{},{},0).status,base::FootprintFollowStatus::blocked);
}

TEST(FootprintFollower, FinalWaypointCanRequireLongerSettling) {
    base::FootprintFollowConfig config;
    config.settleSamples=2;config.finalSettleSamples=8;
    base::FootprintFollower follower({{1,1,0},{2,1,0}}, {.6,.4}, {0,0,5,5}, config);
    for (int i=0;i<7;++i)
        EXPECT_EQ(follower.Update({2,1,0},{},{},.02).status,base::FootprintFollowStatus::running);
    EXPECT_EQ(follower.Update({2,1,0},{},{},.02).status,base::FootprintFollowStatus::arrived);
}

TEST(FootprintFollower, PoseRateDampsRotationAndCanSettleWithWheelDisagreement) {
    base::FootprintFollowConfig config;
    config.maxOmega=5;config.maxAlpha=1000;config.poseAngularDamping=.5;
    config.usePoseRateForSettling=true;config.settleSamples=2;
    base::FootprintFollower braking({{1,1,0},{1,1,1}}, {.6,.4}, {0,0,5,5}, config);
    braking.Update({1,1,0},{},{},.02);
    const auto moving=braking.Update({1,1,.1},{0,0,.1},{},.02);
    EXPECT_LT(moving.velocity.omega,0); // Heading is approaching too quickly.
    base::FootprintFollower settled({{1,1,0},{1,1,1}}, {.6,.4}, {0,0,5,5}, config);
    EXPECT_EQ(settled.Update({1,1,1},{0,0,.1},{},.02).status,base::FootprintFollowStatus::running);
    EXPECT_EQ(settled.Update({1,1,1},{0,0,.1},{},.02).status,base::FootprintFollowStatus::running);
    EXPECT_EQ(settled.Update({1,1,1},{0,0,.1},{},.02).status,base::FootprintFollowStatus::arrived);
}

TEST(FootprintFollower, StopsAngularCorrectionWithinToleranceWhileSettling) {
    base::FootprintFollower follower({{1,1,0},{1,1,1}}, {.6,.4}, {0,0,5,5});
    const auto output=follower.Update({1,1,1.02},{0,0,.2},{},.02);
    EXPECT_EQ(output.status,base::FootprintFollowStatus::running);
    EXPECT_DOUBLE_EQ(output.velocity.omega,0);
}

TEST(FootprintFollower, ProfiledRoutePassesIntermediateWaypointAndStopsAtGoal) {
    base::FootprintFollowConfig config;
    config.terrain=[](base::FootprintPose,base::FootprintPose) {return base::FootprintTerrainSegment{.05,.01};};
    base::FootprintFollower follower({{1,1,0},{2,1,0},{3,1,0}}, {.6,.4}, {0,0,5,5}, config);
    base::FootprintPose pose{1,1,0}; base::FieldVelocity measured{};
    bool passed=false; base::FootprintFollowOutput output{};
    for (int i=0;i<2000;++i) {
        output=follower.Update(pose,measured,{},.02);
        if (output.status!=base::FootprintFollowStatus::running) break;
        if (pose.x>2 && output.velocity.vx>.01) passed=true;
        measured=output.velocity; pose.x+=measured.vx*.02; pose.y+=measured.vy*.02;
    }
    EXPECT_TRUE(passed);
    EXPECT_EQ(output.status,base::FootprintFollowStatus::arrived);
    EXPECT_NEAR(pose.x,3,.04);
    EXPECT_DOUBLE_EQ(output.velocity.vx,0);
}

TEST(FootprintFollower, ProfiledTerrainLimitsInitialRequestAndInvalidTerrainStops) {
    base::FootprintFollowConfig flat;
    flat.terrain=[](base::FootprintPose,base::FootprintPose) {return base::FootprintTerrainSegment{};};
    auto uphill=flat;
    uphill.terrain=[](base::FootprintPose,base::FootprintPose) {return base::FootprintTerrainSegment{.1,0};};
    const std::vector<base::FootprintPose> route{{1,1,0},{3,1,0}};
    base::FootprintFollower a(route,{.6,.4},{0,0,5,5},flat), b(route,{.6,.4},{0,0,5,5},uphill);
    EXPECT_LT(b.Update({1,1,0},{},{},.02).velocity.vx,a.Update({1,1,0},{},{},.02).velocity.vx);
    uphill.terrain=[](base::FootprintPose,base::FootprintPose) {return base::FootprintTerrainSegment{1,0};};
    base::FootprintFollower invalid(route,{.6,.4},{0,0,5,5},uphill);
    EXPECT_EQ(invalid.Update({1,1,0},{},{},.02).status,base::FootprintFollowStatus::blocked);
}
