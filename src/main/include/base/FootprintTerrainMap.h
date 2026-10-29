#pragma once
#include "base/FootprintSpeedProfile.h"
#include <vector>

namespace base {
// Static axis-aligned terrain region in the field frame. Grade components are
// rise/run along +X and +Y; rolling resistance is dimensionless.
struct FootprintTerrainRegion {
    FieldRectangle bounds;
    double gradeX=0, gradeY=0, rollingResistance=0;
};
// Immutable region map. Regions must be finite, nondegenerate, and not overlap
// in area. Empty space has zero grade and rolling resistance.
class FootprintTerrainMap {
public:
    explicit FootprintTerrainMap(std::vector<FootprintTerrainRegion> regions);
    // Ordered fractions strictly inside the segment where terrain changes.
    std::vector<double> Boundaries(FootprintPose from, FootprintPose to) const;
    // Sample a segment already split at Boundaries; grade follows its direction.
    // A segment containing an unsplit boundary fails closed.
    // A segment on a shared boundary fails closed because the grade is ambiguous.
    FootprintTerrainSegment Segment(FootprintPose from, FootprintPose to) const;
private:
    std::vector<FootprintTerrainRegion> regions_;
};
}
