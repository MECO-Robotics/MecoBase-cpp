#include "base/FootprintAutoDrive.h"
#include <frc2/command/Commands.h>
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

namespace base {
namespace {
struct SearchState {
    std::atomic_bool done{false};
    std::stop_source stop;
    std::vector<FootprintPose> route;
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
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintAutoDriveConfig config) {
    return frc2::cmd::Defer([&drive,target=std::move(target),obstacles=std::move(obstacles),config] {
      auto session=std::make_shared<Session>();
      auto attempt=[&drive,target,obstacles,config,session]() -> frc2::CommandPtr {
       return frc2::cmd::Defer([&drive,target,obstacles,config,session]() -> frc2::CommandPtr {
        if (session->finished) return frc2::cmd::None();
        auto state=std::make_shared<SearchState>();
        session->active=state;
        auto result=std::make_shared<FootprintFollowStatus>(FootprintFollowStatus::blocked);
        auto start=frc2::cmd::RunOnce([&drive,target,obstacles,config,state,session] {
            drive.Stop();
            try {
                if (!target || !obstacles) {state->done.store(true);return;}
                const auto goal=target();
                const auto pose=drive.Pose();
                if (!goal || !Finite(*goal) || !Finite(pose)) {state->done.store(true);return;}
                if (!session->goal) session->goal=*goal;
                auto boxes=obstacles();
                auto gate=SearchGate();
                bool expected=false;
                if (!gate->compare_exchange_strong(expected,true)) {state->done.store(true);return;}
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
                } catch (...) {gate->store(false);state->done.store(true);}
            } catch (...) {state->done.store(true);}
        },{&drive});
        auto wait=frc2::cmd::Run([&drive] {drive.Stop();},{&drive})
            .Until([state] {return state->done.load(std::memory_order_acquire);});
        auto follow=frc2::cmd::Defer([&drive,target,obstacles,config,state,session,result]() -> frc2::CommandPtr {
            if (state->route.empty()) {
                session->finished=true;
                return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
            }
            auto guarded=[target,obstacles] {
                const auto value=target();
                if (!value || !Finite(*value)) throw std::runtime_error("footprint target unavailable");
                return obstacles();
            };
            return FollowFootprintRoute(drive,state->route,config.footprint,config.field,std::move(guarded),config.follow,result);
        },{&drive});
        auto finish=frc2::cmd::RunOnce([target,obstacles,session,result] {
            if (*result==FootprintFollowStatus::arrived) {session->finished=true;return;}
            // Expired targets and failed suppliers do not trigger more motion.
            try {
                const auto current=target();
                if (!current || !Finite(*current)) {session->finished=true;return;}
                (void)obstacles();
            } catch (...) {session->finished=true;}
        },{&drive});
        return std::move(start).AndThen(std::move(wait)).AndThen(std::move(follow))
            .AndThen(std::move(finish)).FinallyDo([&drive,state](bool) {state->stop.request_stop();drive.Stop();});
       },{&drive});
      };
      return attempt().AndThen(attempt()).AndThen(attempt()).FinallyDo([&drive,session](bool) {
          if (session->active) session->active->stop.request_stop();
          drive.Stop();
      });
    },{&drive}).BeforeStarting([&drive] {drive.Stop();}).FinallyDo([&drive](bool) {drive.Stop();});
}
}
