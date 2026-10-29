#include "base/FootprintClearance.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>

namespace base {
std::vector<FootprintPose> FindFootprintRoute(FootprintPose start, FootprintPose goal,
        RectangularFootprint footprint, FieldRectangle field,
        std::span<const FieldRectangle> obstacles, double resolution, int budget, std::stop_token stop) {
    if (stop.stop_requested() || !std::isfinite(resolution) || resolution <= 0 || budget <= 0 ||
        !SweptFootprintClear(start,start,footprint,field,obstacles) ||
        !SweptFootprintClear(goal,goal,footprint,field,obstacles)) return {};
    // Preserve an already-clear direct motion, including starts that cannot be
    // safely snapped to a grid-cell centre near a wall.
    if (SweptFootprintClear(start,goal,footprint,field,obstacles)) return {start,goal};
    const double columns=std::ceil((field.maxX-field.minX)/resolution);
    const double rows=std::ceil((field.maxY-field.minY)/resolution);
    // Keep allocations and integer index arithmetic bounded for bad configuration.
    if (!std::isfinite(columns) || !std::isfinite(rows) || columns < 1 || rows < 1 ||
        columns*rows > 62500) return {};
    constexpr int bins=16;
    constexpr double step=2*std::numbers::pi/bins;
    const int nx=static_cast<int>(columns), ny=static_cast<int>(rows), count=nx*ny*bins;
    auto cell=[&](FootprintPose p) {
        return std::pair{std::clamp(static_cast<int>((p.x-field.minX)/resolution),0,nx-1),
                         std::clamp(static_cast<int>((p.y-field.minY)/resolution),0,ny-1)};
    };
    auto index=[&](int x,int y,int h) {return (y*nx+x)*bins+h;};
    auto pose=[&](int x,int y,int h) {
        return FootprintPose{field.minX+(x+.5)*resolution,field.minY+(y+.5)*resolution,h*step};
    };
    const auto [sx,sy]=cell(start);
    const auto [gx,gy]=cell(goal);
    const int sh=(static_cast<int>(std::lround(std::remainder(start.heading,2*std::numbers::pi)/step))+bins)%bins;
    const auto initial=pose(sx,sy,sh);
    if (!SweptFootprintClear(start,initial,footprint,field,obstacles)) return {};
    const double infinity=std::numeric_limits<double>::infinity();
    std::vector<double> cost(count,infinity);
    std::vector<int> parent(count,-1);
    using QueueEntry=std::pair<double,int>;
    std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> open;
    auto heuristic=[&](int x,int y) {return resolution*std::hypot(x-gx,y-gy);};
    const int first=index(sx,sy,sh);
    cost[first]=0;
    open.emplace(heuristic(sx,sy),first);
    int terminal=-1;
    while (!open.empty() && budget>0 && !stop.stop_requested()) {
        const auto [estimate,current]=open.top(); open.pop();
        const int h=current%bins, cellIndex=current/bins, x=cellIndex%nx, y=cellIndex/nx;
        if (estimate > cost[current]+heuristic(x,y)+1e-10) continue;
        --budget;
        const auto from=pose(x,y,h);
        if (x==gx && y==gy && SweptFootprintClear(from,goal,footprint,field,obstacles)) {
            terminal=current; break;
        }
        auto add=[&](int tx,int ty,int th,double edge) {
            if (tx<0 || ty<0 || tx>=nx || ty>=ny) return;
            const int next=index(tx,ty,th);
            const double candidate=cost[current]+edge;
            if (candidate >= cost[next] ||
                !SweptFootprintClear(from,pose(tx,ty,th),footprint,field,obstacles)) return;
            cost[next]=candidate; parent[next]=current;
            open.emplace(candidate+heuristic(tx,ty),next);
        };
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy)
            if (dx || dy) add(x+dx,y+dy,h,resolution*std::hypot(dx,dy));
        const double turnCost=std::hypot(footprint.length,footprint.width)*step/2;
        add(x,y,(h+1)%bins,turnCost);
        add(x,y,(h+bins-1)%bins,turnCost);
    }
    if (terminal<0) return {};
    std::vector<FootprintPose> route;
    for (int state=terminal; state>=0; state=parent[state]) {
        const int h=state%bins, cellIndex=state/bins;
        route.push_back(pose(cellIndex%nx,cellIndex/nx,h));
    }
    std::reverse(route.begin(),route.end());
    route.insert(route.begin(),start);
    route.push_back(goal);
    return route;
}
}
