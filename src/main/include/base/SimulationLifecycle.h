#pragma once

// Simulation is an optional desktop vendordep; roboRIO never links it.
#if !defined(__FRC_ROBORIO__) && __has_include(<meco/sim/SimulationLifecycle.h>)
#include <meco/sim/SimulationLifecycle.h>
namespace base::simulation {
using LifecycleConfig = meco::sim::SimulationLifecycleConfig;
using Lifecycle = meco::sim::SimulationLifecycle;
}
#else
namespace base::simulation {
// Keep the same robot configuration usable without the desktop vendordep.
struct LifecycleConfig {
  double batteryCapacityAh, batteryEmptyVoltage, batteryVoltageRange, batteryResistanceOhms;
  double practiceAutonomousSeconds, practiceAssessmentSeconds, practiceTeleopSeconds;
};
class Lifecycle {
 public:
  explicit Lifecycle(LifecycleConfig) {}
  Lifecycle(const Lifecycle&) = delete;
  Lifecycle& operator=(const Lifecycle&) = delete;
  void Initialize() {}
  void BeforeScheduler() {}
  void AfterScheduler() {}
  void BeforeScene() {}
  void AfterScene() {}
};
}
#endif
