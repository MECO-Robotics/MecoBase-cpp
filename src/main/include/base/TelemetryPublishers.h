#pragma once

#include "base/Telemetry.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace base::telemetry {

inline void PostDashboardBoolean(std::string_view topic, bool value) {
  std::string ownedTopic{topic};
  std::string queueKey = "Dashboard/" + ownedTopic;
  Post(std::move(queueKey), [topic = std::move(ownedTopic), value] {
    frc::SmartDashboard::PutBoolean(topic, value);
  });
}

inline void PostDashboardNumber(std::string_view topic, double value) {
  std::string ownedTopic{topic};
  std::string queueKey = "Dashboard/" + ownedTopic;
  Post(std::move(queueKey), [topic = std::move(ownedTopic), value] {
    frc::SmartDashboard::PutNumber(topic, value);
  });
}

inline void PostDashboardString(std::string_view topic, std::string_view value) {
  std::string ownedTopic{topic};
  std::string ownedValue{value};
  std::string queueKey = "Dashboard/" + ownedTopic;
  Post(std::move(queueKey), [topic = std::move(ownedTopic), value = std::move(ownedValue)] {
    frc::SmartDashboard::PutString(topic, value);
  });
}

inline void PostDashboardNumberArray(std::string_view topic, std::vector<double> values) {
  std::string ownedTopic{topic};
  std::string queueKey = "Dashboard/" + ownedTopic;
  Post(std::move(queueKey), [topic = std::move(ownedTopic), values = std::move(values)] {
    frc::SmartDashboard::PutNumberArray(topic, values);
  });
}

inline void PostNetworkBoolean(std::string_view path, bool value) {
  std::string ownedPath{path};
  std::string queueKey = "NetworkTables/" + ownedPath;
  Post(std::move(queueKey), [path = std::move(ownedPath), value] {
    nt::NetworkTableInstance::GetDefault().GetEntry(path).SetBoolean(value);
  });
}

inline void PostNetworkNumber(std::string_view path, double value) {
  std::string ownedPath{path};
  std::string queueKey = "NetworkTables/" + ownedPath;
  Post(std::move(queueKey), [path = std::move(ownedPath), value] {
    nt::NetworkTableInstance::GetDefault().GetEntry(path).SetDouble(value);
  });
}

inline void PostNetworkString(std::string_view path, std::string_view value) {
  std::string ownedPath{path};
  std::string ownedValue{value};
  std::string queueKey = "NetworkTables/" + ownedPath;
  Post(std::move(queueKey), [path = std::move(ownedPath), value = std::move(ownedValue)] {
    nt::NetworkTableInstance::GetDefault().GetEntry(path).SetString(value);
  });
}

}  // namespace base::telemetry
