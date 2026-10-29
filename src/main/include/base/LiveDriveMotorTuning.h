#pragma once
#include "base/Drive.h"
#include "base/LoggedTunableNumber.h"
#include <array>
#include <memory>
#include <networktables/NetworkTableEntry.h>

namespace base
{
// Optional Phoenix swerve tuning; construct before scheduler execution.
class LiveDriveMotorTuning final : public frc2::SubsystemBase
{
  public:
    LiveDriveMotorTuning(Drive &drive, bool enabled);
    void Periodic() override;
  private:
    Drive &drive_;
    bool enabled_, configuring_ = false;
    DriveMotorGains submitted_, applied_;
    std::array<std::unique_ptr<LoggedTunableNumber>, 12> values_;
    nt::NetworkTableEntry statusEntry_, retryEntry_;
    std::string status_;
    void Status(const std::string &status);
};
}
