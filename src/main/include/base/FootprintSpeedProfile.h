#pragma once
#include "base/FootprintClearance.h"
#include <span>
#include <vector>

namespace base {
// Grade is rise/run in the direction of travel. Rolling resistance is a
// dimensionless coefficient. Values describe each segment, not a field model.
struct FootprintTerrainSegment { double grade=0, rollingResistance=0; };
struct FootprintProfileLimits {
    double maxSpeed=1, maxAcceleration=2, maxBraking=2;
    double lateralAcceleration=2, steeringRate=1;
};
// Returns one speed in m/s per pose, ending at zero. An empty result indicates
// invalid inputs or that the given initial speed cannot reach the required
// terminal stop within the available route. Pure rotations have zero speed.
// These are kinematic bounds; hardware traction and braking must be validated.
std::vector<double> FootprintSpeedProfile(std::span<const FootprintPose> route,
    std::span<const FootprintTerrainSegment> terrain, FootprintProfileLimits limits,
    double initialSpeed);
}
