#pragma once

#include "base/Replay.h"

#include <frc/Preferences.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include <string>
#include <string_view>
#include <vector>

namespace base::replay {

inline bool DashboardBoolean(std::string_view replayKey, std::string_view topic, bool fallback) {
  const std::string key{replayKey};
  const std::string ownedTopic{topic};
  return ProcessValue<bool>(key, [ownedTopic, fallback] {
    return frc::SmartDashboard::GetBoolean(ownedTopic, fallback);
  });
}

inline double DashboardNumber(std::string_view replayKey, std::string_view topic, double fallback) {
  const std::string key{replayKey};
  const std::string ownedTopic{topic};
  return ProcessValue<double>(key, [ownedTopic, fallback] {
    return frc::SmartDashboard::GetNumber(ownedTopic, fallback);
  });
}

inline double PreferenceNumber(std::string_view replayKey, std::string_view preference, double fallback) {
  const std::string key{replayKey};
  const std::string ownedPreference{preference};
  return ProcessValue<double>(key, [ownedPreference, fallback] {
    return frc::Preferences::GetDouble(ownedPreference, fallback);
  });
}

inline double BatteryVoltage(std::string_view replayKey) {
  const std::string key{replayKey};
  return ProcessValue<double>(key, [] { return frc::RobotController::GetBatteryVoltage().value(); });
}

inline bool NetworkBoolean(std::string_view replayKey, std::string_view path, bool fallback) {
  const std::string key{replayKey};
  const std::string ownedPath{path};
  return ProcessValue<bool>(key, [ownedPath, fallback] {
    return nt::NetworkTableInstance::GetDefault().GetEntry(ownedPath).GetBoolean(fallback);
  });
}

inline double NetworkNumber(std::string_view replayKey, std::string_view path, double fallback) {
  const std::string key{replayKey};
  const std::string ownedPath{path};
  return ProcessValue<double>(key, [ownedPath, fallback] {
    return nt::NetworkTableInstance::GetDefault().GetEntry(ownedPath).GetDouble(fallback);
  });
}

inline std::vector<double> NetworkNumberArray(std::string_view replayKey, std::string_view path) {
  const std::string key{replayKey};
  const std::string ownedPath{path};
  return ProcessValue<std::vector<double>>(key, [ownedPath] {
    return nt::NetworkTableInstance::GetDefault().GetEntry(ownedPath).GetDoubleArray({});
  });
}

}  // namespace base::replay
