#include "base/FootprintFollower.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace base {
FootprintFollower::FootprintFollower(std::vector<FootprintPose> route, RectangularFootprint footprint,
        FieldRectangle field, FootprintFollowConfig config)
        : route_(std::move(route)), footprint_(footprint), field_(field), config_(config) {
    if (route_.size()<2 || config_.settleSamples<=0 || config_.finalSettleSamples<0) status_=FootprintFollowStatus::blocked;
    if (!std::isfinite(config_.poseAngularDamping) || config_.poseAngularDamping<0 ||
        !std::isfinite(config_.wheelAngularSanityLimit) || config_.wheelAngularSanityLimit<=0 ||
        !std::isfinite(config_.angularStopTolerance) || config_.angularStopTolerance<0 ||
        config_.angularStopTolerance>config_.headingTolerance)
        status_=FootprintFollowStatus::blocked;
    for (double v : {config_.maxSpeed,config_.maxOmega,config_.maxAcceleration,config_.maxAlpha,
                     config_.positionTolerance,config_.headingTolerance,config_.linearSettleSpeed,
                     config_.angularSettleSpeed})
        if (!std::isfinite(v) || v<=0) status_=FootprintFollowStatus::blocked;
    if (config_.terrain && status_==FootprintFollowStatus::running) {
        try {
            if (config_.terrainBoundaries) {
                std::vector<FootprintPose> split;
                split.push_back(route_.front());
                for (std::size_t i=0;i+1<route_.size();++i) {
                    const auto a=route_[i], b=route_[i+1];
                    const auto fractions=config_.terrainBoundaries(a,b);
                    double previous=0;
                    for (const double t : fractions) {
                        if (!std::isfinite(t) || t<=previous || t>=1 || split.size()>=1023)
                            throw std::invalid_argument("invalid terrain boundary");
                        const double heading=a.heading+t*std::remainder(b.heading-a.heading,2*std::numbers::pi);
                        split.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),heading});
                        previous=t;
                    }
                    if (split.size()>=1024) throw std::invalid_argument("too many terrain boundaries");
                    split.push_back(b);
                }
                route_=std::move(split);
            }
            for (std::size_t i=0;i+1<route_.size();++i) terrain_.push_back(config_.terrain(route_[i],route_[i+1]));
            const FootprintProfileLimits limits{config_.maxSpeed,config_.maxAcceleration,config_.maxAcceleration,
                config_.profileLateralAcceleration,config_.profileSteeringRate};
            speedProfile_=FootprintSpeedProfile(route_,terrain_,limits,0);
            if (speedProfile_.empty()) status_=FootprintFollowStatus::blocked;
        } catch (...) { status_=FootprintFollowStatus::blocked; }
    }
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
    if (haveHeading_) {
        const double rate=std::remainder(pose.heading-lastHeading_,2*std::numbers::pi)/dt;
        if (!std::isfinite(rate)) return stop(FootprintFollowStatus::blocked);
        if (!havePoseRate_) filteredPoseRate_=rate;
        else filteredPoseRate_+=(dt/(.08+dt))*(rate-filteredPoseRate_);
        havePoseRate_=true;
    }
    lastHeading_=pose.heading;haveHeading_=true;
    double dx=target.x-pose.x, dy=target.y-pose.y;
    double angle=std::remainder(target.heading-pose.heading,2*std::numbers::pi);
    const bool atGoal=std::hypot(dx,dy)<config_.positionTolerance && std::abs(angle)<config_.headingTolerance;
    const bool angularStill=config_.usePoseRateForSettling
        ? havePoseRate_ && std::abs(filteredPoseRate_)<config_.angularSettleSpeed &&
          std::abs(measured.omega)<config_.wheelAngularSanityLimit
        : std::abs(measured.omega)<config_.angularSettleSpeed;
    const bool still=std::hypot(measured.vx,measured.vy)<config_.linearSettleSpeed && angularStill;
    settled_=(atGoal && still) ? settled_+1 : 0;
    const int required=(next_+1==route_.size() && config_.finalSettleSamples>0)
        ? config_.finalSettleSamples : config_.settleSamples;
    const bool passWaypoint=!speedProfile_.empty() && next_+1<route_.size() &&
        speedProfile_[next_]>0 && atGoal;
    if (settled_>=required || passWaypoint) {
        settled_=0;
        if (++next_==route_.size()) return stop(FootprintFollowStatus::arrived);
        target=route_[next_];
        if (!SweptFootprintClear(pose,target,footprint_,field_,obstacles)) return stop(FootprintFollowStatus::blocked);
        dx=target.x-pose.x; dy=target.y-pose.y;
        angle=std::remainder(target.heading-pose.heading,2*std::numbers::pi);
    }
    FieldVelocity out{2*dx,2*dy,2*angle-(havePoseRate_ ? config_.poseAngularDamping*filteredPoseRate_ : 0)};
    std::optional<double> profileSpeedLimit;
    if (!speedProfile_.empty()) {
        const auto& from=route_[next_-1];
        const double sx=target.x-from.x, sy=target.y-from.y;
        const double length=std::hypot(sx,sy);
        if (length>1e-9) {
            const double projection=((pose.x-from.x)*sx+(pose.y-from.y)*sy)/length;
            // At the end of a segment the reference may be zero. An overshoot
            // outside tolerance must replan instead of silently stalling there.
            if (projection>=length && std::hypot(dx,dy)>=config_.positionTolerance)
                return stop(FootprintFollowStatus::blocked);
            const double along=std::clamp(projection,0.,length);
            // Look ahead slightly to permit acceleration from a zero endpoint.
            const double sample=std::min(length,along+.5*config_.maxAcceleration*dt*dt);
            const FootprintProfileLimits limits{config_.maxSpeed,config_.maxAcceleration,config_.maxAcceleration,
                config_.profileLateralAcceleration,config_.profileSteeringRate};
            const auto reference=FootprintSegmentSpeed(length,sample,speedProfile_[next_-1],speedProfile_[next_],terrain_[next_-1],limits);
            if (!reference) return stop(FootprintFollowStatus::blocked);
            profileSpeedLimit=*reference;
            const double requested=std::hypot(out.vx,out.vy);
            if (requested>*reference && requested>0) {out.vx*=*reference/requested;out.vy*=*reference/requested;}
        }
    }
    const double speed=std::hypot(out.vx,out.vy);
    if (speed>config_.maxSpeed) {out.vx*=config_.maxSpeed/speed;out.vy*=config_.maxSpeed/speed;}
    out.omega=std::clamp(out.omega,-config_.maxOmega,config_.maxOmega);
    const double change=std::hypot(out.vx-previous_.vx,out.vy-previous_.vy);
    const double maxChange=config_.maxAcceleration*dt;
    if (change>maxChange) {
        out.vx=previous_.vx+(out.vx-previous_.vx)*maxChange/change;
        out.vy=previous_.vy+(out.vy-previous_.vy)*maxChange/change;
    }
    // A shrinking braking envelope must override the request slew limit. This
    // limits the command, not the physical deceleration of the drivetrain.
    const double limitedSpeed=std::hypot(out.vx,out.vy);
    if (profileSpeedLimit && limitedSpeed>*profileSpeedLimit && limitedSpeed>0) {
        out.vx*=*profileSpeedLimit/limitedSpeed;
        out.vy*=*profileSpeedLimit/limitedSpeed;
    }
    out.omega=std::clamp(out.omega,previous_.omega-config_.maxAlpha*dt,previous_.omega+config_.maxAlpha*dt);
    // Stop correcting inside the selected angular band, but continue to
    // require the measured/pose-rate and dwell checks before declaring arrival.
    // A zero request is allowed immediately even when acceleration limiting
    // would otherwise retain the preceding angular command.
    const double angularStop=config_.angularStopTolerance>0 ? config_.angularStopTolerance : config_.headingTolerance;
    if (std::abs(angle)<angularStop) out.omega=0;
    const FootprintPose predicted{pose.x+out.vx*dt,pose.y+out.vy*dt,pose.heading+out.omega*dt};
    if (!SweptFootprintClear(pose,predicted,footprint_,field_,obstacles)) return stop(FootprintFollowStatus::blocked);
    previous_=out;
    return {status_,out};
}
}
