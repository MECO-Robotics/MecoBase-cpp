#include "base/LiveDriveMotorTuning.h"
#include "base/Replay.h"
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <cmath>
#include <exception>

namespace base
{
namespace
{
using Field = double DriveMotorGains::*;
constexpr std::array<Field,12> fields{&DriveMotorGains::driveP,&DriveMotorGains::driveI,&DriveMotorGains::driveD,
    &DriveMotorGains::driveS,&DriveMotorGains::driveV,&DriveMotorGains::driveA,&DriveMotorGains::steerP,
    &DriveMotorGains::steerI,&DriveMotorGains::steerD,&DriveMotorGains::steerS,&DriveMotorGains::steerV,&DriveMotorGains::steerA};
constexpr std::array<const char *,12> names{"Drive/kP","Drive/kI","Drive/kD","Drive/kS","Drive/kV","Drive/kA",
    "Steer/kP","Steer/kI","Steer/kD","Steer/kS","Steer/kV","Steer/kA"};
}
LiveDriveMotorTuning::LiveDriveMotorTuning(Drive &drive, bool enabled)
    : drive_(drive), enabled_(enabled), submitted_(drive.MotorGains()), applied_(submitted_)
{
    SetName("DriveMotorTuning");
    if (!enabled_) return;
    drive_.EnableLiveMotorTuning();
    if (!replay::Active())
    {
        auto nt = nt::NetworkTableInstance::GetDefault();
        statusEntry_ = nt.GetEntry("/Tuning/Drive/MotorStatus");
        retryEntry_ = nt.GetEntry("/Tuning/Drive/MotorRetry");
        retryEntry_.SetBoolean(false);
    }
    for (size_t i=0;i<fields.size();++i)
        values_[i]=std::make_unique<LoggedTunableNumber>(std::string("Drive/MotorGains/")+names[i],submitted_.*fields[i],true);
}
void LiveDriveMotorTuning::Status(const std::string &status)
{
    if (status_ != status)
    {
        status_=status;
        if (!replay::Active()) statusEntry_.SetString(status);
    }
    replay::RecordOutput("Tuning/Drive/MotorStatus", status);
    for (size_t i=0;i<fields.size();++i) values_[i]->ReportEffective(applied_.*fields[i],status);
}
void LiveDriveMotorTuning::Periodic()
{
    if (!enabled_) return;
    DriveMotorGains requested=submitted_;
    bool valid=true;
    for (size_t i=0;i<fields.size();++i)
    {
        requested.*fields[i]=values_[i]->Get();
        valid = values_[i]->LastInputValid() && std::isfinite(requested.*fields[i]) && requested.*fields[i]>=0 && valid;
    }
    const bool retry = replay::ProcessInputs("Tuning/Drive/MotorRetry", [this] {
        const bool pressed=retryEntry_.GetBoolean(false);
        if (pressed) retryEntry_.SetBoolean(false);
        return wpi::json{{"pressed",pressed}};
    }).at("pressed").get<bool>();
    const auto state=drive_.MotorGainState();
    if (state==DriveGainState::Pending) { Status("pending"); return; }
    if (configuring_)
    {
        if (state==DriveGainState::Ready) applied_=submitted_;
        configuring_=false;
    }
    if (!valid) { Status(state==DriveGainState::Fault ? "fault: invalid gains" : "rejected: invalid gains"); return; }
    if (requested==submitted_ && !(retry && state==DriveGainState::Fault))
    {
        Status(state==DriveGainState::Fault ? "fault" : "applied");
        return;
    }
    try
    {
        if (!drive_.SetMotorGains(requested)) { Status("busy"); return; }
        submitted_=requested;
        configuring_=true;
        Status("submitted");
    }
    catch (const std::exception &) { Status("rejected: configuration unavailable"); }
}
}
