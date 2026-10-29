#include "base/FootprintTerrainMap.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace base {
namespace {
bool Clip(double origin,double delta,double lo,double hi,double& enter,double& exit) {
    if (std::abs(delta)<1e-12) return origin>=lo && origin<=hi;
    double a=(lo-origin)/delta,b=(hi-origin)/delta;
    if (a>b) std::swap(a,b);
    enter=std::max(enter,a);exit=std::min(exit,b);
    return enter<exit;
}
void ValidatePose(FootprintPose p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.heading))
        throw std::invalid_argument("nonfinite terrain segment");
}
}
FootprintTerrainMap::FootprintTerrainMap(std::vector<FootprintTerrainRegion> regions)
    : regions_(std::move(regions)) {
    for (std::size_t i=0;i<regions_.size();++i) {
        const auto& r=regions_[i];const auto& b=r.bounds;
        for (double v:{b.minX,b.minY,b.maxX,b.maxY,r.gradeX,r.gradeY,r.rollingResistance})
            if (!std::isfinite(v)) throw std::invalid_argument("nonfinite terrain region");
        if (b.minX>=b.maxX || b.minY>=b.maxY || r.rollingResistance<0)
            throw std::invalid_argument("invalid terrain region");
        for (std::size_t j=0;j<i;++j) {
            const auto& a=regions_[j].bounds;
            if (std::max(a.minX,b.minX)<std::min(a.maxX,b.maxX) &&
                std::max(a.minY,b.minY)<std::min(a.maxY,b.maxY))
                throw std::invalid_argument("overlapping terrain regions");
        }
    }
}
std::vector<double> FootprintTerrainMap::Boundaries(FootprintPose from,FootprintPose to) const {
    ValidatePose(from);ValidatePose(to);
    if (!std::isfinite(to.x-from.x) || !std::isfinite(to.y-from.y))
        throw std::invalid_argument("terrain segment overflow");
    std::vector<double> cuts;
    for (const auto& r:regions_) {
        double enter=0,exit=1;
        if (!Clip(from.x,to.x-from.x,r.bounds.minX,r.bounds.maxX,enter,exit) ||
            !Clip(from.y,to.y-from.y,r.bounds.minY,r.bounds.maxY,enter,exit)) continue;
        if (enter>1e-8 && enter<1-1e-8) cuts.push_back(enter);
        if (exit>1e-8 && exit<1-1e-8) cuts.push_back(exit);
    }
    std::sort(cuts.begin(),cuts.end());
    cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double a,double b) {return std::abs(a-b)<1e-7;}),cuts.end());
    return cuts;
}
FootprintTerrainSegment FootprintTerrainMap::Segment(FootprintPose from,FootprintPose to) const {
    ValidatePose(from);ValidatePose(to);
    const double dx=to.x-from.x,dy=to.y-from.y,length=std::hypot(dx,dy);
    if (!std::isfinite(length)) throw std::invalid_argument("terrain segment overflow");
    if (length<1e-9) return {};
    const double x=from.x+dx/2,y=from.y+dy/2;
    const FootprintTerrainRegion* selected=nullptr;
    for (const auto& r:regions_) {
        const auto& b=r.bounds;
        if (x>=b.minX && x<=b.maxX && y>=b.minY && y<=b.maxY) {
            if (selected) throw std::invalid_argument("ambiguous terrain boundary");
            selected=&r;
        }
    }
    if (!selected) return {};
    const double grade=selected->gradeX*(dx/length)+selected->gradeY*(dy/length);
    if (!std::isfinite(grade)) throw std::invalid_argument("terrain grade overflow");
    return {grade,selected->rollingResistance};
}
}
