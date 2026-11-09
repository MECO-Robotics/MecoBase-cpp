#pragma once

#include <frc/DriverStation.h>
#include <optional>
#include <string>

namespace base {
// Season-neutral snapshot of the match data published by Driver Station.
struct DriverStationMatchInfo {
  bool enabled{};
  bool autonomous{};
  bool teleop{};
  bool test{};
  std::optional<frc::DriverStation::Alliance> alliance;
  std::optional<double> remainingTimeSeconds;
  // Empty and unavailable are distinct states in generic DS data.
  std::optional<std::string> gameSpecificMessage;
};

// Samples the live Driver Station at the behavior boundary or restores the
// recorded snapshot during code replay.
DriverStationMatchInfo ReadDriverStationMatchInfo();
}  // namespace base
