#pragma once
#include <frc/DriverStation.h>
#include <optional>
#include <string>
namespace base::season2026
{
  struct MatchContext
  {
    bool enabled{}, autonomous{}, teleop{}, gameDataValid{};
    std::optional<frc::DriverStation::Alliance> alliance, inactiveFirstAlliance, firstActiveAlliance;
    std::optional<double> periodTime;
    std::string gameData, phase{"unknown"};
    std::optional<bool> blueActive, redActive, ownActive;
    std::optional<double> blueNextChange, redNextChange, ownNextChange;
  };
  MatchContext ComputeMatchContext(bool enabled, bool autonomous, bool teleop,
                                   std::optional<frc::DriverStation::Alliance> alliance, double time, const std::string &data);
  std::string AllianceLabel(std::optional<frc::DriverStation::Alliance> alliance);
  MatchContext ReadMatchContext();
}
