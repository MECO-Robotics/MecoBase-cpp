#include "base/DriverStationMatchInfo.h"
#include "base/Replay.h"

#include <cmath>

namespace base {
DriverStationMatchInfo ReadDriverStationMatchInfo() {
  const auto input = replay::ProcessInputs("DriverStation/MatchInfo", [] {
    const auto alliance = frc::DriverStation::GetAlliance();
    const double remaining = frc::DriverStation::GetMatchTime().value();
    return wpi::json{{"enabled", frc::DriverStation::IsEnabled()},
                     {"autonomous", frc::DriverStation::IsAutonomous()},
                     {"teleop", frc::DriverStation::IsTeleop()},
                     {"test", frc::DriverStation::IsTest()},
                     {"alliance", !alliance ? "unknown" :
                         *alliance == frc::DriverStation::Alliance::kRed ? "red" : "blue"},
                     {"remainingTimeSeconds", std::isfinite(remaining) && remaining >= 0
                         ? wpi::json(remaining) : wpi::json(nullptr)},
                     {"gameSpecificMessage", [&] {
                       const auto message = frc::DriverStation::GetGameSpecificMessage();
                       return message.empty() ? wpi::json(nullptr) : wpi::json(message);
                     }()}};
  });

  DriverStationMatchInfo info;
  info.enabled = input.at("enabled").get<bool>();
  info.autonomous = input.at("autonomous").get<bool>();
  info.teleop = input.at("teleop").get<bool>();
  info.test = input.value("test", false);
  const auto alliance = input.at("alliance").get<std::string>();
  if (alliance == "red") info.alliance = frc::DriverStation::Alliance::kRed;
  else if (alliance == "blue") info.alliance = frc::DriverStation::Alliance::kBlue;
  info.remainingTimeSeconds = input.at("remainingTimeSeconds").is_null()
      ? std::nullopt : std::optional{input.at("remainingTimeSeconds").get<double>()};
  if (!input.at("gameSpecificMessage").is_null())
    info.gameSpecificMessage = input.at("gameSpecificMessage").get<std::string>();
  return info;
}
}  // namespace base
