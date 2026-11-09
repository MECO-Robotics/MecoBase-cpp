#pragma once

// Optional integration: the consuming project must install ChoreoLib.
#include "base/Drive.h"
#include "base/DriverStationMatchInfo.h"
#include "base/Replay.h"
#include <choreo/Choreo.h>
#include <choreo/trajectory/SwerveSample.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace base {
// Loads the trajectory from replay-captured deploy bytes when scheduled. On
// scheduling, snapshots alliance, resets
// odometry, and follows the trajectory until completion or interruption.
// FieldYear selects Choreo's alliance-flip geometry for the consuming season.
template <int FieldYear>
frc2::CommandPtr FollowChoreo(Drive& drive, std::string name,
                              pathplanner::PIDConstants translation,
                              pathplanner::PIDConstants rotation,
                              units::second_t period) {
  const std::string assetName = name.ends_with(".traj") ? name.substr(0, name.size() - 5) : name;
  return frc2::cmd::Defer(
      [&drive, name = std::move(name), assetName, translation, rotation, period]() -> frc2::CommandPtr {
        const auto path = frc::filesystem::GetDeployDirectory() + "/choreo/" + assetName + ".traj";
        constexpr std::size_t kMaxTrajectoryBytes = 16 * 1024 * 1024;
        const auto contents = replay::ProcessFileInput("Replay/Assets/Choreo/" + assetName, path,
                                                       kMaxTrajectoryBytes);
        std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory;
        try {
          if (contents)
            trajectory = choreo::Choreo::LoadTrajectoryString<choreo::SwerveSample>(*contents, assetName);
        } catch (...) {
          FRC_ReportError(frc::warn::Warning, "Unable to parse Choreo trajectory: {}", assetName);
        }
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
          const auto match = ReadDriverStationMatchInfo();
          state->flipped = match.alliance == frc::DriverStation::Alliance::kRed;
          drive.ResetPose(*state->trajectory.template GetInitialPose<FieldYear>(state->flipped));
          state->controller.reset(drive.Pose(), drive.Speeds());
          state->timer.Restart();
        }).Until([state] { return state->timer.HasElapsed(state->trajectory.GetTotalTime()); });
      }, {&drive});
}
}  // namespace base
