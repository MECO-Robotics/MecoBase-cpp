#pragma once
#include "base/FootprintClearance.h"

namespace base {
struct FieldVelocity { double vx, vy, omega; }; // m/s and rad/s, field frame
enum class FootprintFollowStatus { running, arrived, blocked };
struct FootprintFollowOutput { FootprintFollowStatus status; FieldVelocity velocity; };
struct FootprintFollowConfig {
    double maxSpeed=1, maxOmega=1, maxAcceleration=2, maxAlpha=3;
    double positionTolerance=.03, headingTolerance=.04, linearSettleSpeed=.1, angularSettleSpeed=.15;
    int settleSamples=5;
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
    std::size_t next_=1;
    int settled_=0;
    FieldVelocity previous_{};
    FootprintFollowStatus status_=FootprintFollowStatus::running;
};
}
