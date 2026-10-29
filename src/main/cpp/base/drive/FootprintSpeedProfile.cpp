#include "base/FootprintSpeedProfile.h"
#include <algorithm>
#include <cmath>

namespace base {
std::vector<double> FootprintSpeedProfile(std::span<const FootprintPose> route,
        std::span<const FootprintTerrainSegment> terrain, FootprintProfileLimits limits,
        double initialSpeed) {
    if (route.size()<2 || terrain.size()!=route.size()-1 || !std::isfinite(initialSpeed) || initialSpeed<0)
        return {};
    for (double v : {limits.maxSpeed,limits.maxAcceleration,limits.maxBraking,
                     limits.lateralAcceleration,limits.steeringRate})
        if (!std::isfinite(v) || v<=0) return {};
    std::vector<double> distance(terrain.size()), accel(terrain.size()), braking(terrain.size());
    std::vector<double> cap(route.size(),limits.maxSpeed);
    for (std::size_t i=0;i<route.size();++i)
        if (!std::isfinite(route[i].x) || !std::isfinite(route[i].y) || !std::isfinite(route[i].heading)) return {};
    for (std::size_t i=0;i<terrain.size();++i) {
        const auto t=terrain[i];
        if (!std::isfinite(t.grade) || !std::isfinite(t.rollingResistance) || t.rollingResistance<0) return {};
        distance[i]=std::hypot(route[i+1].x-route[i].x,route[i+1].y-route[i].y);
        if (!std::isfinite(distance[i])) return {};
        // Normalize slope so the gravitational projection remains bounded.
        const double gravity=9.81*t.grade/std::hypot(1.,t.grade);
        const double rolling=9.81*t.rollingResistance/std::hypot(1.,t.grade);
        accel[i]=limits.maxAcceleration-gravity-rolling;
        braking[i]=limits.maxBraking+gravity+rolling;
        if (accel[i]<=0 || braking[i]<=0) return {};
        if (distance[i]<1e-9) cap[i]=cap[i+1]=0;
    }
    for (std::size_t i=1;i+1<route.size();++i) {
        if (distance[i-1]<1e-9 || distance[i]<1e-9) continue;
        const double ax=route[i].x-route[i-1].x, ay=route[i].y-route[i-1].y;
        const double bx=route[i+1].x-route[i].x, by=route[i+1].y-route[i].y;
        const double curvature=std::abs(std::atan2(ax*by-ay*bx,ax*bx+ay*by))/((distance[i-1]+distance[i])*.5);
        if (curvature>1e-9) cap[i]=std::min({cap[i],std::sqrt(limits.lateralAcceleration/curvature),limits.steeringRate/curvature});
    }
    std::vector<double> speed=cap;
    speed.front()=std::min(initialSpeed,cap.front());
    for (std::size_t i=0;i<distance.size();++i)
        speed[i+1]=std::min(speed[i+1],std::sqrt(speed[i]*speed[i]+2*accel[i]*distance[i]));
    speed.back()=0;
    for (std::size_t i=distance.size();i-->0;)
        speed[i]=std::min(speed[i],std::sqrt(speed[i+1]*speed[i+1]+2*braking[i]*distance[i]));
    if (speed.front()+1e-9<initialSpeed) return {};
    return speed;
}
}
