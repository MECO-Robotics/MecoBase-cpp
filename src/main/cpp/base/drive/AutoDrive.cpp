#include "base/AutoDrive.h"
#include "NativeADStar.h"
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <mutex>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <numbers>
#include <memory>
#include <utility>

namespace base {
namespace {
std::weak_ptr<detail::NativeADStar> nativePlanner;
class SharedNativePathfinder final : public pathplanner::Pathfinder {
public:
    explicit SharedNativePathfinder(std::shared_ptr<detail::NativeADStar> planner) : planner_(std::move(planner)) {}
    bool isNewPathAvailable() override { return planner_->isNewPathAvailable(); }
    std::shared_ptr<pathplanner::PathPlannerPath> getCurrentPath(pathplanner::PathConstraints c, pathplanner::GoalEndState g) override {
        return planner_->getCurrentPath(c,g);
    }
    void setStartPosition(const frc::Translation2d& start) override { planner_->setStartPosition(start); }
    void setGoalPosition(const frc::Translation2d& goal) override { planner_->setGoalPosition(goal); }
    void setDynamicObstacles(const std::vector<std::pair<frc::Translation2d,frc::Translation2d>>& obstacles,
                             const frc::Translation2d& current) override { planner_->setDynamicObstacles(obstacles,current); }
private:
    std::shared_ptr<detail::NativeADStar> planner_;
};
struct CloseApproachState { double vx{}, vy{}, omega{}; int settledSamples{}; };
}
void ConfigureNativeAutoDrivePathfinder() {
    static std::once_flag once;
    std::call_once(once, [] {
        auto planner=std::make_shared<detail::NativeADStar>();
        nativePlanner=planner;
        pathplanner::Pathfinding::setPathfinder(std::make_unique<SharedNativePathfinder>(std::move(planner)));
    });
}
bool ValidAutoDriveTarget(const frc::Pose2d& pose, const AutoDriveConfig& config) {
    return std::isfinite(pose.X().value()) && std::isfinite(pose.Y().value()) &&
        std::isfinite(pose.Rotation().Radians().value()) && pose.X().value() >= 0 &&
        pose.Y().value() >= 0 && pose.X().value() <= config.fieldLength && pose.Y().value() <= config.fieldWidth;
}
frc2::CommandPtr AutoDriveToPose(Drive& drive, AutoDriveTarget target, AutoDriveConfig config) {
    for (double value : {config.fieldLength, config.fieldWidth, config.maxSpeed,
                         config.maxAcceleration, config.maxOmega, config.maxAlpha})
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("AutoDrive limits must be finite and positive");
    if (!target) throw std::invalid_argument("AutoDrive requires a target supplier");
    return frc2::cmd::Defer([&drive, target, config]() -> frc2::CommandPtr {
        const auto goal = target();
        if (!goal || !ValidAutoDriveTarget(*goal, config))
            return frc2::cmd::RunEnd([&drive] { drive.Stop(); }, [&drive] { drive.Stop(); }, {&drive});
        // PathPlanner's pathfinding command cancels below 0.5 m. Use a
        // bounded final approach only if the same inflated planner grid says
        // the whole center-line is clear; recheck dynamic obstacles each tick.
        auto finalApproach=[&drive,goal,target,config]() -> frc2::CommandPtr {
            const auto start=drive.Pose();
            // Do not turn a pathfinding failure into an unrestricted straight
            // drive: only correct a nearby goal on the inflated, current grid.
            if(start.Translation().Distance(goal->Translation()).value()>1.)
                return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
            auto planner=nativePlanner.lock();
            if(!planner || !planner->DirectSegmentClear(start.Translation(),goal->Translation()))
                return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
            const auto robotSpeed=drive.Speeds();
            const auto fieldSpeed=frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotSpeed,start.Rotation());
            const double initialLimit=std::min({config.maxSpeed,drive.Config().maxSpeed,.5});
            const double initialMagnitude=std::hypot(fieldSpeed.vx.value(),fieldSpeed.vy.value());
            const double initialScale=initialMagnitude>initialLimit ? initialLimit/initialMagnitude : 1.;
            const double initialOmegaLimit=std::min({config.maxOmega,drive.Config().maxAngularRate,1.});
            auto state=std::make_shared<CloseApproachState>(CloseApproachState{
                fieldSpeed.vx.value()*initialScale,fieldSpeed.vy.value()*initialScale,
                std::clamp(fieldSpeed.omega.value(),-initialOmegaLimit,initialOmegaLimit)});
            auto valid=[&drive,goal,target,config] {
                const auto current=target();
                auto planner=nativePlanner.lock();
                return current && ValidAutoDriveTarget(*current,config) && planner &&
                    planner->DirectSegmentClear(drive.Pose().Translation(),goal->Translation());
            };
            return frc2::cmd::Run([&drive,goal,config,state,valid] {
                if(!valid()) {drive.Stop();return;}
                const auto pose=drive.Pose();
                double vx=(goal->X()-pose.X()).value()*drive.Config().autoTranslationKP;
                double vy=(goal->Y()-pose.Y()).value()*drive.Config().autoTranslationKP;
                const double limit=std::min({config.maxSpeed,drive.Config().maxSpeed,.5});
                const double magnitude=std::hypot(vx,vy);
                if(magnitude>limit) {vx*=limit/magnitude;vy*=limit/magnitude;}
                const double maxDelta=config.maxAcceleration*.02;
                const double delta=std::hypot(vx-state->vx,vy-state->vy);
                if(delta>maxDelta) {vx=state->vx+(vx-state->vx)*maxDelta/delta;vy=state->vy+(vy-state->vy)*maxDelta/delta;}
                const double error=std::remainder((goal->Rotation().Radians()-pose.Rotation().Radians()).value(),2*std::numbers::pi);
                const double angularLimit=std::min({config.maxOmega,drive.Config().maxAngularRate,1.});
                double omega=std::clamp(error*drive.Config().autoRotationKP,-angularLimit,angularLimit);
                omega=std::clamp(omega,state->omega-config.maxAlpha*.02,state->omega+config.maxAlpha*.02);
                state->vx=vx;state->vy=vy;state->omega=omega;
                drive.RunVelocity(frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    units::meters_per_second_t{vx},units::meters_per_second_t{vy},units::radians_per_second_t{omega},pose.Rotation()),units::second_t{.02}));
            },{&drive}).Until([&drive,goal,valid,state] {
                if(!valid()) return true;
                const auto pose=drive.Pose();const auto speed=drive.Speeds();
                const bool settled = pose.Translation().Distance(goal->Translation()).value()<.05 &&
                    std::abs(std::remainder((goal->Rotation().Radians()-pose.Rotation().Radians()).value(),2*std::numbers::pi))<.06 &&
                    std::hypot(speed.vx.value(),speed.vy.value())<.15 && std::abs(speed.omega.value())<.2;
                // Avoid completing on a single transient speed sample while the
                // plant is still moving. Five scheduler ticks give 100 ms of settling.
                state->settledSamples = settled ? state->settledSamples+1 : 0;
                return state->settledSamples>=5;
            });
        };
        // A timed PathPlanner trajectory may finish while the robot is still
        // far behind it. Retry through the same obstacle-aware planner rather
        // than declaring arrival or driving an unrestricted straight line.
        auto pathAttempt=[&drive,goal,target,config]() -> frc2::CommandPtr {
            return frc2::cmd::Defer([&drive,goal,target,config]() -> frc2::CommandPtr {
                const auto current=target();
                if(!current || !ValidAutoDriveTarget(*current,config) ||
                   drive.Pose().Translation().Distance(goal->Translation()).value()<.5)
                    return frc2::cmd::None();
                return pathplanner::AutoBuilder::pathfindToPose(*goal,
                    pathplanner::PathConstraints{
                        units::meters_per_second_t{std::min(config.maxSpeed, drive.Config().maxSpeed)},
                        units::meters_per_second_squared_t{config.maxAcceleration},
                        units::radians_per_second_t{std::min(config.maxOmega, drive.Config().maxAngularRate)},
                        units::radians_per_second_squared_t{config.maxAlpha}})
                    .Until([target,config] {
                        const auto current=target();
                        return !current || !ValidAutoDriveTarget(*current,config);
                    });
            },{&drive});
        };
        return pathAttempt().AndThen(pathAttempt()).AndThen(pathAttempt())
            .AndThen(frc2::cmd::Defer([target,config,finalApproach,&drive]() mutable -> frc2::CommandPtr {
                const auto current=target();
                if(!current || !ValidAutoDriveTarget(*current,config))
                    return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
                return finalApproach();
            },{&drive}));
    }, {&drive})
    .BeforeStarting([&drive,config] {
        drive.Stop();
        drive.SetPathOutputLimits(std::min(config.maxSpeed,drive.Config().maxSpeed),
                                  std::min(config.maxOmega,drive.Config().maxAngularRate));
    })
    .FinallyDo([&drive](bool) { drive.Stop();drive.ClearPathOutputLimits(); });
}
frc2::CommandPtr AutoDriveTrackTarget(Drive& drive, AutoDriveTarget target,
                                     AutoDriveConfig config, AutoDriveTrackingConfig tracking) {
    if (!target || !std::isfinite(tracking.translationThreshold) || tracking.translationThreshold<=0 ||
        !std::isfinite(tracking.stabilitySeconds) || tracking.stabilitySeconds<0 ||
        !std::isfinite(tracking.stabilityTimeout) || tracking.stabilityTimeout<=0 ||
        tracking.stabilitySeconds>tracking.stabilityTimeout ||
        !std::isfinite(tracking.linearSettleSpeed) || tracking.linearSettleSpeed<=0 ||
        !std::isfinite(tracking.angularSettleSpeed) || tracking.angularSettleSpeed<=0 ||
        tracking.settleSamples<=0)
        throw std::invalid_argument("Invalid AutoDrive tracking configuration");
    // Validate the common drive limits before the command is scheduled.
    for (double value : {config.fieldLength,config.fieldWidth,config.maxSpeed,
                         config.maxAcceleration,config.maxOmega,config.maxAlpha})
        if (!std::isfinite(value) || value<=0)
            throw std::invalid_argument("AutoDrive limits must be finite and positive");
    struct Session { bool finished=false, changed=false; std::optional<frc::Pose2d> goal; };
    struct WaitState {
        std::optional<frc::Pose2d> anchor, lastPose;
        int settledSamples=0;
        std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now(), stableSince=started, lastSample=started;
    };
    return frc2::cmd::Defer([&drive,target,config,tracking] {
        auto session=std::make_shared<Session>();
        auto attempt=[&drive,target,config,tracking,session]() -> frc2::CommandPtr {
            return frc2::cmd::Defer([&drive,target,config,tracking,session]() -> frc2::CommandPtr {
                if (session->finished) return frc2::cmd::None();
                try { session->goal=target(); } catch (...) { session->goal.reset(); }
                if (!session->goal || !ValidAutoDriveTarget(*session->goal,config)) {
                    session->finished=true;
                    return frc2::cmd::RunOnce([&drive] {drive.Stop();},{&drive});
                }
                session->changed=false;
                auto guarded=[target,config,tracking,session]() -> std::optional<frc::Pose2d> {
                    try {
                        const auto current=target();
                        if (!current || !ValidAutoDriveTarget(*current,config)) {session->finished=true;return {};}
                        if (current->Translation().Distance(session->goal->Translation()).value()>=tracking.translationThreshold) {
                            session->changed=true;return {};
                        }
                        return session->goal;
                    } catch (...) {session->finished=true;return {};}
                };
                return AutoDriveToPose(drive,std::move(guarded),config)
                    .Until([session] {return session->finished || session->changed;})
                    .FinallyDo([&drive,session](bool) {drive.Stop();if (!session->changed) session->finished=true;});
            },{&drive});
        };
        auto wait=[&drive,target,config,tracking,session]() -> frc2::CommandPtr {
            return frc2::cmd::Defer([&drive,target,config,tracking,session]() -> frc2::CommandPtr {
                if (session->finished || !session->changed) return frc2::cmd::None();
                auto state=std::make_shared<WaitState>();
                return frc2::cmd::Run([&drive,target,config,tracking,session,state] {
                    drive.Stop();
                    const auto now=std::chrono::steady_clock::now();
                    try {
                        const auto current=target();
                        if (!current || !ValidAutoDriveTarget(*current,config) ||
                            std::chrono::duration<double>(now-state->started).count()>=tracking.stabilityTimeout) {
                            session->finished=true;return;
                        }
                        if (!state->anchor || current->Translation().Distance(state->anchor->Translation()).value()>=tracking.translationThreshold) {
                            state->anchor=*current;state->stableSince=now;
                        }
                        const auto pose=drive.Pose();
                        const auto speeds=drive.Speeds();
                        if (!std::isfinite(pose.X().value()) || !std::isfinite(pose.Y().value()) ||
                            !std::isfinite(pose.Rotation().Radians().value()) ||
                            !std::isfinite(speeds.vx.value()) || !std::isfinite(speeds.vy.value()) ||
                            !std::isfinite(speeds.omega.value())) {session->finished=true;return;}
                        if (state->lastPose) {
                            const double dt=std::chrono::duration<double>(now-state->lastSample).count();
                            if (dt>0) {
                                const double linear=pose.Translation().Distance(state->lastPose->Translation()).value()/dt;
                                const double angular=std::abs(std::remainder((pose.Rotation().Radians()-
                                    state->lastPose->Rotation().Radians()).value(),2*std::numbers::pi))/dt;
                                if (linear<=tracking.linearSettleSpeed && angular<=tracking.angularSettleSpeed &&
                                    std::hypot(speeds.vx.value(),speeds.vy.value())<=tracking.linearSettleSpeed &&
                                    std::abs(speeds.omega.value())<=tracking.angularSettleSpeed)
                                    ++state->settledSamples;
                                else state->settledSamples=0;
                            }
                        }
                        state->lastPose=pose;state->lastSample=now;
                        if (std::chrono::duration<double>(now-state->stableSince).count()>=tracking.stabilitySeconds &&
                            state->settledSamples>=tracking.settleSamples)
                            session->changed=false;
                    } catch (...) {session->finished=true;}
                },{&drive}).Until([session] {return session->finished || !session->changed;});
            },{&drive});
        };
        return attempt().AndThen(wait()).AndThen(attempt()).AndThen(wait()).AndThen(attempt())
            .FinallyDo([&drive](bool) {drive.Stop();});
    },{&drive});
}

}
