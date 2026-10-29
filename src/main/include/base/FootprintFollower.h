#pragma once
#include "base/FootprintClearance.h"
#include "base/FootprintSpeedProfile.h"
#include <functional>

namespace base {
struct FieldVelocity { double vx, vy, omega; }; // m/s and rad/s, field frame
enum class FootprintFollowStatus { running, arrived, blocked };
struct FootprintFollowOutput { FootprintFollowStatus status; FieldVelocity velocity; };
struct FootprintFollowConfig {
    double maxSpeed=1, maxOmega=1, maxAcceleration=2, maxAlpha=3;
    double positionTolerance=.03, headingTolerance=.04, linearSettleSpeed=.1, angularSettleSpeed=.15;
    int settleSamples=5;
    int finalSettleSamples=0; // Zero uses settleSamples.
    // Optional heading-rate feedback derived from successive pose samples.
    // This remains opt-in until the consumer validates its localization timing.
    double poseAngularDamping=0, wheelAngularSanityLimit=.25;
    bool usePoseRateForSettling=false;
    double angularStopTolerance=0; // Zero uses headingTolerance.
    // Optional translation progress watchdog. Zero timeout disables it.
    double translationStallTimeout=0, translationStallProgress=.03, translationStallRequest=.1;
    // Empty preserves stop-at-each-waypoint following. The supplier describes
    // each directed segment and runs on the scheduler thread during setup.
    std::function<FootprintTerrainSegment(FootprintPose,FootprintPose)> terrain;
    // Optional ordered fractions strictly inside each directed segment. The
    // supplier identifies terrain boundaries and runs during setup.
    std::function<std::vector<double>(FootprintPose,FootprintPose)> terrainBoundaries;
    double profileLateralAcceleration=2, profileSteeringRate=1;
};

// Pure feedback state machine for a checked route. The caller owns scheduling,
// drivetrain requirements, clocking, and stopping when interrupted or disabled.
// A blocked result latches and requests zero until a new follower is constructed.
class FootprintFollower {
 public:
    FootprintFollower(std::vector<FootprintPose> route, RectangularFootprint footprint,
                      FieldRectangle field, FootprintFollowConfig config = {});
    FootprintFollowOutput Update(FootprintPose pose, FieldVelocity measured,
                                 std::span<const FieldRectangle> obstacles, double dt);
 private:
    std::vector<FootprintPose> route_;
    RectangularFootprint footprint_;
    FieldRectangle field_;
    FootprintFollowConfig config_;
    std::vector<FootprintTerrainSegment> terrain_;
    std::vector<double> speedProfile_;
    std::size_t next_=1;
    int settled_=0;
    FieldVelocity previous_{};
    double lastHeading_=0, filteredPoseRate_=0;
    double progressDistance_=0, stalledSeconds_=0;
    bool haveProgressDistance_=false;
    bool haveHeading_=false, havePoseRate_=false;
    FootprintFollowStatus status_=FootprintFollowStatus::running;
};
}
