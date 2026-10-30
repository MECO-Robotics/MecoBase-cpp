#pragma once

#include <networktables/NetworkTableEntry.h>
#include <string>
#include <string_view>

namespace base
{
// Own on the robot thread. Disabled instances neither create nor read NT entries.
class LoggedTunableNumber
{
  public:
    LoggedTunableNumber(std::string key, double defaultValue, bool enabled);
    // Rejects nonfinite input and retains the last valid value.
    double Get();
    bool LastInputValid() const { return valid_; }
    std::string_view LastInputError() const { return inputError_; }
    // Publish the consumer-validated value and decision for NT logging.
    void ReportEffective(double effective, std::string_view status);

  private:
    std::string path_;
    double value_;
    bool enabled_, valid_ = true;
    nt::NetworkTableEntry entry_, effectiveEntry_, statusEntry_;
    double reported_ = 0;
    bool hasReport_ = false;
    std::string reportedStatus_;
    std::string inputError_;
};
} // namespace base
