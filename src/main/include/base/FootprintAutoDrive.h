#pragma once
#include "base/AutoDrive.h"
#include "base/FootprintCommand.h"
#include <memory>

namespace base {
enum class FootprintAutoDriveOutcome {
    running, arrived, stalled, blocked, no_route, planner_busy,
    target_unavailable, target_unstable, target_changed, invalid_feedback, source_error, settle_failed, interrupted
};
// Stable telemetry names for drive outcomes; unknown enum values return "unknown".
    inline const char *FootprintOutcomeName(FootprintAutoDriveOutcome outcome)
    {
      using O = FootprintAutoDriveOutcome;
      switch (outcome)
      {
      case O::running:
        return "running";
      case O::arrived:
        return "arrived";
      case O::stalled:
        return "stalled";
      case O::blocked:
        return "blocked";
      case O::no_route:
        return "no-route";
      case O::planner_busy:
        return "planner-busy";
      case O::target_unavailable:
        return "target-unavailable";
      case O::target_unstable:
        return "target-unstable";
      case O::target_changed:
        return "target-changed";
      case O::invalid_feedback:
        return "invalid-feedback";
      case O::source_error:
        return "source-error";
      case O::settle_failed:
        return "settle-failed";
      case O::interrupted:
        return "interrupted";
      }
      return "unknown";
    }
struct FootprintAutoDriveConfig {
    RectangularFootprint footprint;
    FieldRectangle field;
    double resolution=.2;
    int expansionBudget=30000;
    FootprintFollowConfig follow;
    double replanLinearSettleSpeed=.1, replanAngularSettleSpeed=.15;
    double replanSettleTimeout=2.; // seconds
    int replanSettleSamples=5;
    // Zero retains the original sampled goal. Positive values permit bounded
    // replanning when the target translation moves at least this many metres.
    double targetTranslationReplanThreshold=0;
    double targetStabilitySeconds=.25, targetStabilityTimeout=2.;
};
// Goal is sampled on start and must remain available while driving. Search uses
// a snapshot of obstacle geometry on a cancellable worker; the supplier is
// called on the scheduler thread and must be fast. At most one footprint search
// runs at a time. Before each search, measured wheel speeds and successive
// estimated poses must settle while stopped; timeout or invalid data fails closed.
// A blocked route or an enabled target change can be replanned twice; an invalid
// target, busy worker, stalled route, or failed search stops the drive; a stall
// is not automatically retried. This does not guarantee
// braking distance or detect motion absent from both localization and wheels.
// The optional outcome is updated on the scheduler thread; no_route includes
// search budget exhaustion or worker failure and is not proof that no route exists.
frc2::CommandPtr AutoDriveFootprintToPose(Drive& drive, AutoDriveTarget target,
        std::function<std::vector<FieldRectangle>()> obstacles, FootprintAutoDriveConfig config,
        std::shared_ptr<FootprintAutoDriveOutcome> outcome = {});
}
