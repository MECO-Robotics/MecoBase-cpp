#pragma once

// Optional integration: the consuming project must install ChoreoLib.
#include "base/Drive.h"
#include <choreo/Choreo.h>
#include <choreo/trajectory/SwerveSample.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace base {
// Loads the trajectory when called. On scheduling, snapshots alliance, resets
// odometry, and follows the trajectory until completion or interruption.
// FieldYear selects Choreo's alliance-flip geometry for the consuming season.
template <int FieldYear>
frc2::CommandPtr FollowChoreo(Drive& drive, std::string name,
                              pathplanner::PIDConstants translation,
                              pathplanner::PIDConstants rotation,
                              units::second_t period) {
  auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(name);
  if (!trajectory || !trajectory->GetInitialPose()) {
    return frc2::cmd::RunOnce([&drive, name] {
      drive.Stop();
      FRC_ReportError(frc::warn::Warning, "Unable to load Choreo trajectory: {}", name);
    }, {&drive});
  }
  struct State {
    State(choreo::Trajectory<choreo::SwerveSample> path,
          pathplanner::PIDConstants translationGains, pathplanner::PIDConstants rotationGains,
          units::second_t interval)
        : trajectory(std::move(path)), controller(translationGains, rotationGains, interval), period(interval) {}
    choreo::Trajectory<choreo::SwerveSample> trajectory;
    frc::Timer timer;
    bool flipped = false;
    pathplanner::PPHolonomicDriveController controller;
    units::second_t period;
  };
  auto state = std::make_shared<State>(std::move(*trajectory), translation, rotation, period);
  return frc2::cmd::RunEnd([&drive, state] {
    auto sample = state->trajectory.template SampleAt<FieldYear>(state->timer.Get(), state->flipped);
    if (!sample) { drive.Stop(); return; }
    pathplanner::PathPlannerTrajectoryState target;
    target.time = sample->timestamp;
    target.pose = sample->GetPose();
    target.fieldSpeeds = sample->GetChassisSpeeds();
    target.linearVelocity = units::meters_per_second_t{std::hypot(sample->vx.value(), sample->vy.value())};
    target.heading = target.linearVelocity.value() > 1e-6
        ? frc::Rotation2d{sample->vx.value(), sample->vy.value()} : target.pose.Rotation();
    drive.RunVelocity(frc::ChassisSpeeds::Discretize(
        state->controller.calculateRobotRelativeSpeeds(drive.Pose(), target), state->period));
  }, [&drive, state] { state->timer.Stop(); drive.Stop(); }, {&drive})
  .BeforeStarting([&drive, state] {
    state->flipped = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    drive.ResetPose(*state->trajectory.template GetInitialPose<FieldYear>(state->flipped));
    state->controller.reset(drive.Pose(), drive.Speeds());
    state->timer.Restart();
  }).Until([state] { return state->timer.HasElapsed(state->trajectory.GetTotalTime()); });
}
}  // namespace base
