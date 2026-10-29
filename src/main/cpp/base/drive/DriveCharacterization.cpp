#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/Replay.h"
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/Utils.hpp>
#include <cmath>
#include <stdexcept>

namespace base
{
frc2::CommandPtr Drive::Characterize(bool dynamic, frc2::sysid::Direction direction)
{
    if (native_)
        for (size_t i = 0; i < 4; ++i)
            native_->GetModule(i).GetDriveMotor().GetMotorVoltage().SetUpdateFrequency(units::hertz_t{50});
    auto apply = [this](units::volt_t volts) {
        requestedSpeeds_ = {};
        if (MotorGainState() != DriveGainState::Ready) return;
        replay::RecordOutput("Drive/CharacterizationVoltage", volts.value());
        if (portable_)
            portable_->Voltage(volts.value());
        if (native_)
            native_->SetControl(ctre::phoenix6::swerve::requests::SysIdSwerveTranslation{}.WithVolts(volts));
    };
    auto routine = std::make_shared<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            apply,
            [this, apply](frc::sysid::SysIdRoutineLog *log) {
                try
                {
                    // Read measured wheel distance/speed (including native coupling correction),
                    // never requested voltage or rotor-turn values labeled as SI.
                    const auto inputs = replay::ProcessInputs("Drive/Characterization", [this] {
                        if (portable_)
                        {
                            const auto s = portable_->State();
                            wpi::json modules = wpi::json::array();
                            for (size_t i = 0; i < 4; ++i)
                                modules.push_back(
                                    {s.voltages[i], s.positions[i].distance.value(), s.states[i].speed.value()});
                            return wpi::json{{"healthy", s.healthy}, {"modules", modules}};
                        }
                        const auto state = native_->GetState();
                        wpi::json modules = wpi::json::array();
                        bool healthy = native_->IsOdometryValid();
                        const double age = (ctre::phoenix6::utils::GetCurrentTime() - state.Timestamp).value();
                        healthy = healthy && std::isfinite(age) && age >= 0 && age <= .1;
                        for (size_t i = 0; i < 4; ++i)
                        {
                            // Refresh() is nonblocking; never wait for CAN from the control loop.
                            auto &voltage = native_->GetModule(i).GetDriveMotor().GetMotorVoltage();
                            const double latency = voltage.GetTimestamp().GetLatency().value();
                            healthy = healthy && voltage.GetStatus().IsOK() && voltage.GetTimestamp().IsValid() &&
                                      std::isfinite(latency) && latency >= 0 && latency <= .1;
                            modules.push_back({voltage.GetValue().value(), state.ModulePositions[i].distance.value(),
                                               state.ModuleStates[i].speed.value()});
                        }
                        return wpi::json{{"healthy", healthy}, {"modules", modules}};
                    });
                    if (!inputs.is_object() || !inputs.contains("healthy") || !inputs["healthy"].is_boolean() ||
                        !inputs["healthy"].get<bool>())
                        throw std::runtime_error("Unavailable or stale drivetrain characterization feedback");
                    const auto &modules = inputs.at("modules");
                    if (!modules.is_array() || modules.size() != 4)
                        throw std::runtime_error("Invalid Drive/Characterization module inputs");
                    for (size_t i = 0; i < 4; ++i)
                    {
                        if (!modules[i].is_array() || modules[i].size() != 3)
                            throw std::runtime_error("Invalid Drive/Characterization sample");
                        for (const auto &value : modules[i])
                            if (!value.is_number() || !std::isfinite(value.get<double>()))
                                throw std::runtime_error("Nonfinite Drive/Characterization sample");
                        log->Motor("drive-" + std::to_string(i))
                            .voltage(units::volt_t{modules[i][0].get<double>()})
                            .position(units::meter_t{modules[i][1].get<double>()})
                            .velocity(units::meters_per_second_t{modules[i][2].get<double>()});
                    }
                }
                catch (...)
                {
                    // A failed feedback/log read must not leave the test voltage latched.
                    apply(units::volt_t{0});
                    throw;
                }
            },
            this});
    auto command = dynamic ? routine->Dynamic(direction) : routine->Quasistatic(direction);
    return std::move(command).FinallyDo([this, routine, apply](bool) {
        apply(units::volt_t{0});
        Stop();
    });
}
} // namespace base
