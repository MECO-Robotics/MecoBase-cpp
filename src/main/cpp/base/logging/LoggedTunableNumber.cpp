#include "base/LoggedTunableNumber.h"
#include "base/Replay.h"
#include <cmath>
#include <networktables/NetworkTableInstance.h>
#include <stdexcept>
#include <utility>

namespace base
{
LoggedTunableNumber::LoggedTunableNumber(std::string key, double defaultValue, bool enabled)
    : path_("/TunableNumbers/" + std::move(key)), value_(defaultValue), enabled_(enabled)
{
    if (!std::isfinite(value_))
        throw std::invalid_argument("Nonfinite tunable default");
    if (enabled_ && !replay::Active())
    {
        entry_ = nt::NetworkTableInstance::GetDefault().GetEntry(path_);
        entry_.SetDefaultDouble(value_);
        effectiveEntry_ = nt::NetworkTableInstance::GetDefault().GetEntry(path_ + "/Effective");
        statusEntry_ = nt::NetworkTableInstance::GetDefault().GetEntry(path_ + "/Status");
    }
}
double LoggedTunableNumber::Get()
{
    if (!enabled_)
        return value_;
    const auto input = replay::ProcessInputs(path_, [this] {
        const double requested = entry_.GetDouble(value_);
        // JSON cannot represent nonfinite values; record them explicitly.
        return wpi::json{{"valid", std::isfinite(requested)},
                         {"value", std::isfinite(requested) ? wpi::json(requested) : wpi::json(nullptr)}};
    });
    valid_ = input.at("valid").get<bool>();
    if (valid_)
        value_ = input.at("value").get<double>();
    return value_;
}
void LoggedTunableNumber::ReportEffective(double effective, std::string_view status)
{
    if (!enabled_)
        return;
    if (!std::isfinite(effective))
        throw std::invalid_argument("Nonfinite effective tunable value");
    replay::RecordOutput(path_ + "/Effective", effective);
    replay::RecordOutput(path_ + "/Status", std::string(status));
    if (!replay::Active() && (!hasReport_ || reported_ != effective || reportedStatus_ != status))
    {
        effectiveEntry_.SetDouble(effective);
        statusEntry_.SetString(status);
        reported_ = effective;
        reportedStatus_ = status;
        hasReport_ = true;
    }
}
} // namespace base
