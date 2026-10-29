#include "base/FootprintClearance.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace base {
namespace {
bool Valid(FieldRectangle r) {
    return std::isfinite(r.minX) && std::isfinite(r.minY) &&
           std::isfinite(r.maxX) && std::isfinite(r.maxY) &&
           r.minX < r.maxX && r.minY < r.maxY;
}
bool Clear(FootprintPose p, RectangularFootprint f, FieldRectangle field,
           std::span<const FieldRectangle> obstacles, double pad) {
    const double c=std::cos(p.heading), s=std::sin(p.heading);
    const double l=f.length/2, w=f.width/2;
    const double ex=l*std::abs(c)+w*std::abs(s), ey=l*std::abs(s)+w*std::abs(c);
    if (p.x-ex-pad <= field.minX || p.x+ex+pad >= field.maxX ||
        p.y-ey-pad <= field.minY || p.y+ey+pad >= field.maxY) return false;
    for (const auto& box : obstacles) {
        const double dx=p.x-(box.minX+box.maxX)/2, dy=p.y-(box.minY+box.maxY)/2;
        const double hx=(box.maxX-box.minX)/2, hy=(box.maxY-box.minY)/2;
        // SAT on the two field axes and the two chassis axes. The pad bounds
        // the motion of every chassis point between adjacent samples.
        if (std::abs(dx) > hx+ex+pad || std::abs(dy) > hy+ey+pad ||
            std::abs(dx*c+dy*s) > l+hx*std::abs(c)+hy*std::abs(s)+pad ||
            std::abs(-dx*s+dy*c) > w+hx*std::abs(s)+hy*std::abs(c)+pad) continue;
        return false;
    }
    return true;
}
}
bool SweptFootprintClear(FootprintPose start, FootprintPose end,
                         RectangularFootprint footprint, FieldRectangle field,
                         std::span<const FieldRectangle> obstacles) {
    for (double v : {start.x,start.y,start.heading,end.x,end.y,end.heading,
                     footprint.length,footprint.width}) if (!std::isfinite(v)) return false;
    if (footprint.length <= 0 || footprint.width <= 0 || !Valid(field)) return false;
    for (const auto& obstacle : obstacles) if (!Valid(obstacle)) return false;
    const double dx=end.x-start.x, dy=end.y-start.y;
    const double turn=std::remainder(end.heading-start.heading,2*std::numbers::pi);
    const double radius=std::hypot(footprint.length,footprint.width)/2;
    const double motion=std::hypot(dx,dy)+radius*std::abs(turn);
    if (!std::isfinite(motion) || motion > 1000) return false;
    const int steps=std::max(1,static_cast<int>(std::ceil(motion/.025)));
    const double pad=motion/(2*steps);
    for (int i=0; i<steps; ++i) {
        const double t=(i+.5)/steps;
        if (!Clear({start.x+t*dx,start.y+t*dy,start.heading+t*turn},footprint,field,obstacles,pad)) return false;
    }
    return true;
}
}
