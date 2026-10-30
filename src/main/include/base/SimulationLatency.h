#pragma once

// Desktop consumers using these diagnostics also need MecoSimulation.
// The roboRIO build does not require the desktop-only vendordep.
#ifndef __FRC_ROBORIO__
#include <meco/sim/SimulationDiagnostics.h>
namespace base::simulation
{
  using meco::sim::BeginLatencyProbe;
  using meco::sim::EndLatencyProbe;
  using meco::sim::ObserveJoystick;
}
#else
namespace base::simulation
{
  inline double ObserveJoystick(double value) { return value; }
  inline void BeginLatencyProbe() {}
  inline void EndLatencyProbe() {}
}
#endif
