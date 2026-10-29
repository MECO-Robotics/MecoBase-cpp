#pragma once

#include "base/LoggedTunableNumber.h"
#include "base/Mechanism.h"
#include <array>
#include <memory>

namespace base
{
// Own alongside the mechanism; the scheduler polls it once per robot loop.
class LiveMechanismTuning final : public frc2::SubsystemBase
{
  public:
    LiveMechanismTuning(Mechanism &mechanism, bool enabled);
    void Periodic() override;

  private:
    Mechanism &mechanism_;
    bool enabled_, configuring_ = false;
    Gains submitted_, applied_;
    nt::NetworkTableEntry statusEntry_, retryEntry_;
    std::string status_;
    void Status(const std::string &status);
    std::array<std::unique_ptr<LoggedTunableNumber>, 13> values_;
};
} // namespace base
