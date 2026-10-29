#include "base/FootprintAutoDrive.h"
#include <frc2/command/Commands.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <thread>
#include <utility>

namespace base {
namespace {
struct SearchState {
    std::atomic_bool done{false};
    std::stop_source stop;
    std::vector<FootprintPose> route;
    FootprintAutoDriveOutcome failure=FootprintAutoDriveOutcome::no_route;
    bool settled=false, settleFailed=false;
    int settleSamples=0;
    std::optional<frc::Pose2d> lastPose;
    std::chrono::steady_clock::time_point started, lastSample;
};
struct Session {
    std::optional<frc::Pose2d> goal;
    bool finished=false;
    std::shared_ptr<SearchState> active;
};
// The gate is shared with detached workers so it outlives all scheduler state.
std::shared_ptr<std::atomic_bool> SearchGate() {
    static auto gate=std::make_shared<std::atomic_bool>(false);
    return gate;
}
bool Finite(const frc::Pose2d& p) {
    return std::isfinite(p.X().value()) && std::isfinite(p.Y().value()) &&
           std::isfinite(p.Rotation().Radians().value());
}
}
frc2::CommandPtr AutoDriveFootprintToPose(Drive& drive, AutoDriveTarget target,
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintAutoDriveConfig config,
        std::shared_ptr<FootprintAutoDriveOutcome> outcome) {
    if (!outcome) outcome=std::make_shared<FootprintAutoDriveOutcome>();
    return frc2::cmd::Defer([&drive,target=std::move(target),obstacles=std::move(obstacles),config,outcome] {
      auto session=std::make_shared<Session>();
      auto attempt=[&drive,target,obstacles,config,session,outcome]() -> frc2::CommandPtr {
       return frc2::cmd::Defer([&drive,target,obstacles,config,session,outcome]() -> frc2::CommandPtr {
        if (session->finished) return frc2::cmd::None();
        auto state=std::make_shared<SearchState>();
        session->active=state;
        auto result=std::make_shared<FootprintFollowStatus>(FootprintFollowStatus::blocked);
        auto beginSettling=frc2::cmd::RunOnce([state] {
            state->started=std::chrono::steady_clock::now();
        },{&drive});
        auto settle=frc2::cmd::Run([&drive,config,state] {
            drive.Stop();
            const auto now=std::chrono::steady_clock::now();
            if (!std::isfinite(config.replanLinearSettleSpeed) || config.replanLinearSettleSpeed<=0 ||
                !std::isfinite(config.replanAngularSettleSpeed) || config.replanAngularSettleSpeed<=0 ||
                !std::isfinite(config.replanSettleTimeout) || config.replanSettleTimeout<=0 ||
                config.replanSettleSamples<=0 ||
                std::chrono::duration<double>(now-state->started).count()>=config.replanSettleTimeout) {
                state->settleFailed=true; return;
            }
            const auto pose=drive.Pose();
            const auto speeds=drive.Speeds();
            const double vx=speeds.vx.value(), vy=speeds.vy.value(), omega=speeds.omega.value();
            if (!Finite(pose) || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega)) {
                state->settleFailed=true; return;
            }
            if (state->lastPose) {
                const double dt=std::chrono::duration<double>(now-state->lastSample).count();
                if (dt>0) {
                    const auto& previous=*state->lastPose;
                    const double linear=std::hypot(pose.X().value()-previous.X().value(),pose.Y().value()-previous.Y().value())/dt;
                    const double angular=std::abs(std::remainder(pose.Rotation().Radians().value()-previous.Rotation().Radians().value(),2*std::numbers::pi))/dt;
                    if (linear<=config.replanLinearSettleSpeed && angular<=config.replanAngularSettleSpeed &&
                        std::hypot(vx,vy)<=config.replanLinearSettleSpeed && std::abs(omega)<=config.replanAngularSettleSpeed)
                        ++state->settleSamples;
                    else state->settleSamples=0;
                    state->settled=state->settleSamples>=config.replanSettleSamples;
                }
            }
            state->lastPose=pose; state->lastSample=now;
        },{&drive}).Until([state] {return state->settled || state->settleFailed;});
        auto start=frc2::cmd::RunOnce([&drive,target,obstacles,config,state,session] {
            drive.Stop();
            try {
                if (!state->settled) {state->failure=FootprintAutoDriveOutcome::settle_failed;state->done.store(true);return;}
                if (!target) {state->failure=FootprintAutoDriveOutcome::target_unavailable;state->done.store(true);return;}
                if (!obstacles) {state->failure=FootprintAutoDriveOutcome::source_error;state->done.store(true);return;}
                const auto goal=target();
                const auto pose=drive.Pose();
                if (!goal || !Finite(*goal)) {state->failure=FootprintAutoDriveOutcome::target_unavailable;state->done.store(true);return;}
                if (!Finite(pose)) {state->failure=FootprintAutoDriveOutcome::invalid_feedback;state->done.store(true);return;}
                if (!session->goal) session->goal=*goal;
                auto boxes=obstacles();
                auto gate=SearchGate();
                bool expected=false;
                if (!gate->compare_exchange_strong(expected,true)) {state->failure=FootprintAutoDriveOutcome::planner_busy;state->done.store(true);return;}
                try {
                    const auto token=state->stop.get_token();
                    std::thread([state,gate,config,boxes=std::move(boxes),pose,goal=*session->goal,token] {
                        try {
                            state->route=FindFootprintRoute({pose.X().value(),pose.Y().value(),pose.Rotation().Radians().value()},
                                {goal.X().value(),goal.Y().value(),goal.Rotation().Radians().value()},
                                config.footprint,config.field,boxes,config.resolution,config.expansionBudget,token);
                        } catch (...) {state->route.clear();}
                        gate->store(false);
                        state->done.store(true,std::memory_order_release);
                    }).detach();
                } catch (...) {gate->store(false);state->failure=FootprintAutoDriveOutcome::source_error;state->done.store(true);}
            } catch (...) {state->failure=FootprintAutoDriveOutcome::source_error;state->done.store(true);}
        },{&drive});
        auto wait=frc2::cmd::Run([&drive] {drive.Stop();},{&drive})
            .Until([state] {return state->done.load(std::memory_order_acquire);});
        auto follow=frc2::cmd::Defer([&drive,target,obstacles,config,state,session,result,outcome]() -> frc2::CommandPtr {
            if (state->route.empty()) {
                session->finished=true;
                *outcome=state->failure;
                return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
            }
            auto guarded=[target,obstacles] {
                const auto value=target();
                if (!value || !Finite(*value)) throw std::runtime_error("footprint target unavailable");
                return obstacles();
            };
            return FollowFootprintRoute(drive,state->route,config.footprint,config.field,std::move(guarded),config.follow,result);
        },{&drive});
        auto finish=frc2::cmd::RunOnce([target,obstacles,session,result,outcome] {
            if (session->finished) return;
            if (*result==FootprintFollowStatus::arrived || *result==FootprintFollowStatus::stalled) {
                *outcome=*result==FootprintFollowStatus::arrived ? FootprintAutoDriveOutcome::arrived : FootprintAutoDriveOutcome::stalled;
                session->finished=true;return;
            }
            // Expired targets and failed suppliers do not trigger more motion.
            try {
                const auto current=target();
                if (!current || !Finite(*current)) {*outcome=FootprintAutoDriveOutcome::target_unavailable;session->finished=true;return;}
                (void)obstacles();
            } catch (...) {*outcome=FootprintAutoDriveOutcome::source_error;session->finished=true;}
        },{&drive});
        return std::move(beginSettling).AndThen(std::move(settle)).AndThen(std::move(start)).AndThen(std::move(wait)).AndThen(std::move(follow))
            .AndThen(std::move(finish)).FinallyDo([&drive,state](bool) {state->stop.request_stop();drive.Stop();});
       },{&drive});
      };
      return attempt().AndThen(attempt()).AndThen(attempt()).FinallyDo([&drive,session](bool) {
          if (session->active) session->active->stop.request_stop();
          drive.Stop();
      });
    },{&drive}).BeforeStarting([&drive,outcome] {*outcome=FootprintAutoDriveOutcome::running;drive.Stop();})
      .FinallyDo([&drive,outcome](bool interrupted) {
          if (interrupted) *outcome=FootprintAutoDriveOutcome::interrupted;
          else if (*outcome==FootprintAutoDriveOutcome::running) *outcome=FootprintAutoDriveOutcome::blocked;
          drive.Stop();
      });
}
}
