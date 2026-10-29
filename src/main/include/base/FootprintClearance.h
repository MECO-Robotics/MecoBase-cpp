#pragma once

#include <span>

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
}
