#pragma once
#include "base/Drive.h"
#include <functional>
#include <optional>
#include <pathplanner/lib/path/PathConstraints.h>

namespace base {
struct AutoDriveConfig {
    double fieldLength = 16.541, fieldWidth = 8.069;
    double maxSpeed = 2., maxAcceleration = 3., maxOmega = 3., maxAlpha = 6.;
    double arrivalDistance = .05, arrivalHeading = .06, arrivalSpeed = .15, arrivalOmega = .2;
    int arrivalSamples = 5;
    bool operator==(const AutoDriveConfig &) const = default;
};
// Fixed blue-origin goals. No desktop process or simulation dependency.
// ConfigureAuto() must have been called on drive. The deployed PathPlanner
// navgrid supplies obstacle clearance. A missing/stale target stops immediately.
using AutoDriveTarget = std::function<std::optional<frc::Pose2d>()>;
// Install once before any pathfinding command is constructed.
void ConfigureNativeAutoDrivePathfinder();
bool ValidAutoDriveTarget(const frc::Pose2d& target, const AutoDriveConfig& config);
frc2::CommandPtr AutoDriveToPose(Drive& drive, AutoDriveTarget target,
                               AutoDriveConfig config = {});
// Replans safely on configuration changes. Supplier must be fast and validated.
frc2::CommandPtr AutoDriveToPoseLive(Drive& drive, AutoDriveTarget target,
                                   std::function<AutoDriveConfig()> config);
// Opt-in moving-target mode. Stops when the target moves, waits for a stable
// observation, then replans, with at most three attempts per command.
struct AutoDriveTrackingConfig {
    double translationThreshold = .3;
    double stabilitySeconds = .25;
    double stabilityTimeout = 2.;
    double linearSettleSpeed = .15;
    double angularSettleSpeed = .2;
    int settleSamples = 5;
};
frc2::CommandPtr AutoDriveTrackTarget(Drive& drive, AutoDriveTarget target,
                                    AutoDriveConfig config = {},
                                    AutoDriveTrackingConfig tracking = {});
// Restarts tracking safely when the supplied drive configuration changes.
frc2::CommandPtr AutoDriveTrackTargetLive(Drive& drive, AutoDriveTarget target,
    std::function<AutoDriveConfig()> config, AutoDriveTrackingConfig tracking = {});
}
