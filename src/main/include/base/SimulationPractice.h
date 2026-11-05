#pragma once

#include <frc2/command/CommandPtr.h>
#include <optional>

// Simulation is an optional desktop vendordep; roboRIO never links it.
#if !defined(__FRC_ROBORIO__) && __has_include(<meco/sim/PracticeMatchController.h>)
#include <meco/sim/PracticeMatchController.h>
namespace base::simulation {
inline std::optional<frc2::CommandPtr> TryStartPracticeMatch() {
  return meco::sim::TryStartPracticeMatch();
}
}
#else
namespace base::simulation {
inline std::optional<frc2::CommandPtr> TryStartPracticeMatch() { return std::nullopt; }
}
#endif
