#pragma once
#include "base/Drive.h"
#include "base/FootprintFollower.h"
#include <functional>

namespace base {
// Schedule a fresh route follower with exclusive drivetrain ownership. The
// supplier returns current obstacle geometry in the same field frame as the
// route. Drive and captured supplier state must outlive the command.
frc2::CommandPtr FollowFootprintRoute(Drive& drive, std::vector<FootprintPose> route,
        RectangularFootprint footprint, FieldRectangle field,
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintFollowConfig config = {});
}
