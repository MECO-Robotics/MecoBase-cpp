#include "base/FootprintCommand.h"
#include <frc2/command/Commands.h>
#include <algorithm>
#include <memory>
#include <utility>

namespace base {
frc2::CommandPtr FollowFootprintRoute(Drive& drive, std::vector<FootprintPose> route,
        RectangularFootprint footprint, FieldRectangle field,
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintFollowConfig config,
        std::shared_ptr<FootprintFollowStatus> result) {
    config.maxSpeed=std::min(config.maxSpeed,drive.Config().maxSpeed);
    config.maxOmega=std::min(config.maxOmega,drive.Config().maxAngularRate);
    return frc2::cmd::Defer([&drive,route=std::move(route),footprint,field,obstacles=std::move(obstacles),config,result] {
        auto follower=std::make_shared<FootprintFollower>(route,footprint,field,config);
        auto status=result ? result : std::make_shared<FootprintFollowStatus>();
        *status=FootprintFollowStatus::running;
        return frc2::cmd::Run([&drive,follower,status,obstacles] {
            try {
                if (!obstacles) { *status=FootprintFollowStatus::blocked; drive.Stop(); return; }
                const auto pose=drive.Pose();
                const auto measured=frc::ChassisSpeeds::FromRobotRelativeSpeeds(drive.Speeds(),pose.Rotation());
                const auto boxes=obstacles();
                const auto result=follower->Update({pose.X().value(),pose.Y().value(),pose.Rotation().Radians().value()},
                    {measured.vx.value(),measured.vy.value(),measured.omega.value()},boxes,.02);
                *status=result.status;
                if (*status!=FootprintFollowStatus::running) {drive.Stop();return;}
                const auto& velocity=result.velocity;
                drive.RunVelocity(frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    units::meters_per_second_t{velocity.vx},units::meters_per_second_t{velocity.vy},
                    units::radians_per_second_t{velocity.omega},pose.Rotation()),units::second_t{.02}));
            } catch (...) {
                // A failed source must not leave the previous drive request active.
                *status=FootprintFollowStatus::blocked;
                drive.Stop();
            }
        },{&drive}).Until([status] {return *status!=FootprintFollowStatus::running;});
    },{&drive}).BeforeStarting([&drive] {drive.Stop();}).FinallyDo([&drive](bool) {drive.Stop();});
}
}
