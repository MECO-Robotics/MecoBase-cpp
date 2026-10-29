#pragma once
#include "base/AutoDrive.h"
#include "base/FootprintCommand.h"

namespace base {
struct FootprintAutoDriveConfig {
    RectangularFootprint footprint;
    FieldRectangle field;
    double resolution=.2;
    int expansionBudget=30000;
    FootprintFollowConfig follow;
};
// Goal is sampled on start and must remain available while driving. Search uses
// a snapshot of obstacle geometry on a cancellable worker; the supplier is
// called on the scheduler thread and must be fast. At most one footprint search
// runs at a time. A blocked route can be replanned twice with fresh obstacles;
// an invalid target, busy worker, or failed search stops the drive.
frc2::CommandPtr AutoDriveFootprintToPose(Drive& drive, AutoDriveTarget target,
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintAutoDriveConfig config);
}
