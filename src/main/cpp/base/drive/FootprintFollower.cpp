#include "base/FootprintFollower.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace base {
FootprintFollower::FootprintFollower(std::vector<FootprintPose> route, RectangularFootprint footprint,
        FieldRectangle field, FootprintFollowConfig config)
        : route_(std::move(route)), footprint_(footprint), field_(field), config_(config) {
    if (route_.size()<2 || config_.settleSamples<=0 || config_.finalSettleSamples<0) status_=FootprintFollowStatus::blocked;
    for (double v : {config_.maxSpeed,config_.maxOmega,config_.maxAcceleration,config_.maxAlpha,
                     config_.positionTolerance,config_.headingTolerance,config_.linearSettleSpeed,
                     config_.angularSettleSpeed})
        if (!std::isfinite(v) || v<=0) status_=FootprintFollowStatus::blocked;
}
FootprintFollowOutput FootprintFollower::Update(FootprintPose pose, FieldVelocity measured,
        std::span<const FieldRectangle> obstacles, double dt) {
    auto stop=[this](FootprintFollowStatus status) {
        status_=status; previous_={}; return FootprintFollowOutput{status,{}};
    };
    if (status_!=FootprintFollowStatus::running) return {status_,{}};
    if (!std::isfinite(dt) || dt<=0 || dt>.1 || !std::isfinite(measured.vx) ||
        !std::isfinite(measured.vy) || !std::isfinite(measured.omega)) return stop(FootprintFollowStatus::blocked);
    auto target=route_[next_];
    if (!SweptFootprintClear(pose,target,footprint_,field_,obstacles)) return stop(FootprintFollowStatus::blocked);
    double dx=target.x-pose.x, dy=target.y-pose.y;
    double angle=std::remainder(target.heading-pose.heading,2*std::numbers::pi);
    const bool atGoal=std::hypot(dx,dy)<config_.positionTolerance && std::abs(angle)<config_.headingTolerance;
    const bool still=std::hypot(measured.vx,measured.vy)<config_.linearSettleSpeed &&
                     std::abs(measured.omega)<config_.angularSettleSpeed;
    settled_=(atGoal && still) ? settled_+1 : 0;
    const int required=(next_+1==route_.size() && config_.finalSettleSamples>0)
        ? config_.finalSettleSamples : config_.settleSamples;
    if (settled_>=required) {
        settled_=0;
        if (++next_==route_.size()) return stop(FootprintFollowStatus::arrived);
        target=route_[next_];
        if (!SweptFootprintClear(pose,target,footprint_,field_,obstacles)) return stop(FootprintFollowStatus::blocked);
        dx=target.x-pose.x; dy=target.y-pose.y;
        angle=std::remainder(target.heading-pose.heading,2*std::numbers::pi);
    }
    FieldVelocity out{2*dx,2*dy,2*angle};
    const double speed=std::hypot(out.vx,out.vy);
    if (speed>config_.maxSpeed) {out.vx*=config_.maxSpeed/speed;out.vy*=config_.maxSpeed/speed;}
    out.omega=std::clamp(out.omega,-config_.maxOmega,config_.maxOmega);
    const double change=std::hypot(out.vx-previous_.vx,out.vy-previous_.vy);
    const double maxChange=config_.maxAcceleration*dt;
    if (change>maxChange) {
        out.vx=previous_.vx+(out.vx-previous_.vx)*maxChange/change;
        out.vy=previous_.vy+(out.vy-previous_.vy)*maxChange/change;
    }
    out.omega=std::clamp(out.omega,previous_.omega-config_.maxAlpha*dt,previous_.omega+config_.maxAlpha*dt);
    const FootprintPose predicted{pose.x+out.vx*dt,pose.y+out.vy*dt,pose.heading+out.omega*dt};
    if (!SweptFootprintClear(pose,predicted,footprint_,field_,obstacles)) return stop(FootprintFollowStatus::blocked);
    previous_=out;
    return {status_,out};
}
}
