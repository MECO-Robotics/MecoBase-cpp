#pragma once
#include "base/AutoDrive.h"
#include "base/FootprintCommand.h"
#include <memory>

namespace base {
enum class FootprintAutoDriveOutcome {
    running, arrived, stalled, blocked, no_route, planner_busy,
    target_unavailable, invalid_feedback, source_error, settle_failed, interrupted
};
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
