#pragma once

#include <span>
#include <stop_token>
#include <vector>

namespace base {
// All distances are metres and angles are radians in the field frame.
struct FootprintPose { double x, y, heading; };
struct RectangularFootprint { double length, width; };
struct FieldRectangle { double minX, minY, maxX, maxY; };

// Tests a continuously swept rectangular chassis, using the shortest rotation. Field
// edges and contact with an obstacle count as blocked. Invalid input fails closed.
bool SweptFootprintClear(FootprintPose start, FootprintPose end,
                         RectangularFootprint footprint, FieldRectangle field,
                         std::span<const FieldRectangle> obstacles);

// Bounded, orientation-aware grid search. Returned segments are individually
// checked with SweptFootprintClear. An empty result means no route was found
// within the budget; it does not prove that no continuous route exists. Run
// search off the robot control thread; the budget is not a wall-clock limit.
std::vector<FootprintPose> FindFootprintRoute(FootprintPose start, FootprintPose goal,
                         RectangularFootprint footprint, FieldRectangle field,
                         std::span<const FieldRectangle> obstacles,
                         double resolution, int expansionBudget, std::stop_token stop = {});
}
