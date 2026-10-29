#include "base/LiveMechanismTuning.h"
#include "base/Replay.h"
#include <array>
#include <exception>
#include <networktables/NetworkTableInstance.h>
#include <string>

namespace base
{
namespace
{
using Field = double Gains::*;
constexpr std::array<Field, 13> fields{&Gains::p, &Gains::i, &Gains::d, &Gains::s, &Gains::v, &Gains::a,
                                       &Gains::gravity, &Gains::maxVelocity, &Gains::maxAcceleration,
                                       &Gains::tolerance, &Gains::minimum, &Gains::maximum, &Gains::defaultPosition};
constexpr std::array<const char *, 13> names{"kP", "kI", "kD", "kS", "kV", "kA", "kG", "kMaxVelo",
                                             "kMaxAccel", "kTolerance", "kMinPosition", "kMaxPosition", "kDefaultPosition"};
bool Equal(const Gains &a, const Gains &b)
{
    for (auto field : fields)
        if (a.*field != b.*field)
            return false;
    return true;
}
} // namespace
LiveMechanismTuning::LiveMechanismTuning(Mechanism &mechanism, bool enabled)
    : mechanism_(mechanism), enabled_(enabled), submitted_(mechanism.GetGains()), applied_(submitted_)
{
    SetName(mechanism.GetName() + "LiveTuning");
    if (!enabled_)
        return;
    if (!replay::Active())
    {
        auto nt = nt::NetworkTableInstance::GetDefault();
        statusEntry_ = nt.GetEntry("/Tuning/" + mechanism.GetName() + "/LiveStatus");
        retryEntry_ = nt.GetEntry("/Tuning/" + mechanism.GetName() + "/Retry");
        retryEntry_.SetBoolean(false);
    }
    for (size_t i = 0; i < fields.size(); ++i)
        values_[i] = std::make_unique<LoggedTunableNumber>(mechanism.GetName() + "/Gains/" + names[i],
                                                          submitted_.*fields[i], true);
}
void LiveMechanismTuning::Status(const std::string &status)
{
    if (status != status_)
    {
        status_ = status;
        if (!replay::Active())
            statusEntry_.SetString(status);
    }
    replay::RecordOutput("Tuning/" + mechanism_.GetName() + "/LiveStatus", status);
    for (size_t i = 0; i < fields.size(); ++i)
        values_[i]->ReportEffective(applied_.*fields[i], status);
}
void LiveMechanismTuning::Periodic()
{
    if (!enabled_)
        return;
    Gains requested = submitted_;
    bool valid = true;
    for (size_t i = 0; i < fields.size(); ++i)
    {
        requested.*fields[i] = values_[i]->Get();
        valid = values_[i]->LastInputValid() && valid;
    }
    const auto retry = replay::ProcessInputs("Tuning/" + mechanism_.GetName() + "/Retry", [this] {
        const bool pressed = retryEntry_.GetBoolean(false);
        if (pressed) retryEntry_.SetBoolean(false);
        return wpi::json{{"pressed", pressed}};
    }).at("pressed").get<bool>();
    const auto state = mechanism_.ConfigurationState();
    if (configuring_ && state == MotorConfigurationState::Pending)
    {
        Status("pending");
        return;
    }
    if (configuring_)
    {
        if (state == MotorConfigurationState::Ready)
            applied_ = submitted_;
        Status(state == MotorConfigurationState::Fault ? "fault" : "applied");
        configuring_ = false;
    }
    if (!valid)
    {
        Status(state == MotorConfigurationState::Fault ? "fault: nonfinite input" : "rejected: nonfinite input");
        return;
    }
    if (Equal(requested, submitted_) && !(retry && state == MotorConfigurationState::Fault))
    {
        Status(state == MotorConfigurationState::Fault ? "fault" : "applied");
        return;
    }
    try
    {
        requested.Validate(mechanism_.IsPositionControlled());
    }
    catch (const std::exception &)
    {
        Status(state == MotorConfigurationState::Fault ? "fault: invalid gains" : "rejected: invalid gains");
        return;
    }
    try
    {
        mechanism_.SetGains(requested);
        submitted_ = requested;
        configuring_ = true;
        Status("submitted");
    }
    catch (const std::exception &)
    {
        // Busy requests remain in NT and are retried; faults continue to inhibit output.
        Status("rejected: configuration unavailable");
    }
}
} // namespace base
