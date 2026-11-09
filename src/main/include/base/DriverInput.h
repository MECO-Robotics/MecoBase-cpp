#pragma once

#include "base/NtGamepad.h"
#include "base/Replay.h"
#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <frc/geometry/Rotation2d.h>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <stdexcept>

namespace base
{
  struct DriverAxes
  {
    double leftY{}, leftX{}, rightX{};
  };

  struct DriverInputState
  {
    DriverAxes axes; // Neutral unless the selected input has teleop authority.
    DriverAxes driverStationAxes; // Raw connected, sufficiently populated DS device.
    NtGamepad::State nt;
    std::string source = "none";
    bool available{};
    bool authorized{};
    using DriveRequestObserver = std::function<void(const DriverInputState &, double, double, double, frc::Rotation2d)>;
    const DriveRequestObserver *driveRequestObserver{}; // Non-owning; must outlive this input snapshot.

    // Report the command derived from this input to an optional observer. The
    // observer runs at the real drive-request boundary and never selects input.
    void ObserveDriveRequest(double x, double y, double omega, frc::Rotation2d heading) const
    {
      if (driveRequestObserver && *driveRequestObserver)
        (*driveRequestObserver)(*this, x, y, omega, heading);
    }

  };

  namespace detail {
  inline DriverInputState DecodeDriverInput(
      const wpi::json &input,
      const DriverInputState::DriveRequestObserver *driveRequestObserver)
  {
    DriverInputState result;
    result.driveRequestObserver = driveRequestObserver;
    result.axes = {input.at("axes").at("leftY").get<double>(),
                   input.at("axes").at("leftX").get<double>(),
                   input.at("axes").at("rightX").get<double>()};
    result.driverStationAxes = {input.at("driverStationAxes").at("leftY").get<double>(),
                                input.at("driverStationAxes").at("leftX").get<double>(),
                                input.at("driverStationAxes").at("rightX").get<double>()};
    const auto &nt = input.at("nt");
    result.nt.sequence = nt.at("sequence").get<uint64_t>();
    result.nt.sourceTimestampUs = nt.at("sourceTimestampUs").get<int64_t>();
    result.nt.ageMs = nt.at("ageMs").is_null() ? std::numeric_limits<double>::infinity()
                                                  : nt.at("ageMs").get<double>();
    result.nt.leftY = nt.at("leftY").get<double>();
    result.nt.leftX = nt.at("leftX").get<double>();
    result.nt.rightX = nt.at("rightX").get<double>();
    result.nt.accepted = nt.at("accepted").get<bool>();
    result.source = input.at("source").get<std::string>();
    result.available = input.at("available").get<bool>();
    result.authorized = input.at("authorized").get<bool>();

    const auto validAxes = [](const DriverAxes &axes) {
      const auto valid = [](double axis) { return std::isfinite(axis) && std::abs(axis) <= 1.0; };
      return valid(axes.leftY) && valid(axes.leftX) && valid(axes.rightX);
    };
    if (!validAxes(result.axes) || !validAxes(result.driverStationAxes))
      throw std::runtime_error("invalid axes in replayed Input/DriverInput");
    if (result.source != "none" && result.source != "nt" && result.source != "driverstation")
      throw std::runtime_error("invalid source in replayed Input/DriverInput");
    if ((result.source == "none") != !result.available || (result.authorized && !result.available))
      throw std::runtime_error("inconsistent availability in replayed Input/DriverInput");
    if ((result.source == "nt") != result.nt.accepted)
      throw std::runtime_error("inconsistent NT selection in replayed Input/DriverInput");
    if (!result.authorized &&
        (result.axes.leftY != 0.0 || result.axes.leftX != 0.0 || result.axes.rightX != 0.0))
      throw std::runtime_error("unauthorized replayed Input/DriverInput is not neutral");
    if (result.source == "driverstation" &&
        (result.axes.leftY != result.driverStationAxes.leftY || result.axes.leftX != result.driverStationAxes.leftX ||
         result.axes.rightX != result.driverStationAxes.rightX))
      throw std::runtime_error("inconsistent Driver Station axes in replayed Input/DriverInput");
    if (result.nt.accepted && (!std::isfinite(result.nt.ageMs) || result.nt.ageMs < 0.0 ||
                               result.nt.sourceTimestampUs < 0 || !validAxes({result.nt.leftY, result.nt.leftX, result.nt.rightX})))
      throw std::runtime_error("invalid accepted NT sample in replayed Input/DriverInput");
    return result;
  }
  } // namespace detail

  // Prefer fresh NT axes when configured. DS fallback is a caller-owned policy;
  // enable and emergency-stop authority always come from Driver Station. The
  // returned drive axes are neutral without both a valid source and authority.
  // An optional observer receives the eventual drive request when the caller
  // reports it through DriverInputState::ObserveDriveRequest.
  inline DriverInputState ReadDriverInput(frc::XboxController &controller,
                                          NtGamepad *ntGamepad,
                                          bool allowDriverStationFallback,
                                          const DriverInputState::DriveRequestObserver *driveRequestObserver = nullptr)
  {
    if (replay::Active())
      return detail::DecodeDriverInput(replay::RecordedInput("Input/DriverInput"), driveRequestObserver);

    DriverInputState result;
    result.driveRequestObserver = driveRequestObserver;
    const bool dsConnected = controller.IsConnected() && controller.GetAxisCount() >= 5;
    if (dsConnected)
      result.driverStationAxes = {controller.GetLeftY(), controller.GetLeftX(), controller.GetRightX()};
    const auto valid = [](double axis) { return std::isfinite(axis) && std::abs(axis) <= 1.0; };
    const auto &ds = result.driverStationAxes;
    const bool validDs = dsConnected && valid(ds.leftY) && valid(ds.leftX) && valid(ds.rightX);
    DriverAxes selected;
    if (ntGamepad)
      result.nt = ntGamepad->Read();
    if (ntGamepad && result.nt.accepted)
    {
      selected = {result.nt.leftY, result.nt.leftX, result.nt.rightX};
      result.source = "nt";
      result.available = true;
    }
    else if (validDs && (!ntGamepad || allowDriverStationFallback))
    {
      selected = ds;
      result.source = "driverstation";
      result.available = true;
    }
    result.authorized = result.available && frc::DriverStation::IsTeleopEnabled() &&
                        !frc::DriverStation::IsEStopped();
    if (result.authorized)
      result.axes = selected;
    const auto input = replay::ProcessInputs("Input/DriverInput", [&result] {
      return wpi::json{
          {"axes", {{"leftY", result.axes.leftY}, {"leftX", result.axes.leftX}, {"rightX", result.axes.rightX}}},
          {"driverStationAxes", {{"leftY", result.driverStationAxes.leftY},
                                 {"leftX", result.driverStationAxes.leftX},
                                 {"rightX", result.driverStationAxes.rightX}}},
          {"nt", {{"sequence", result.nt.sequence},
                   {"sourceTimestampUs", result.nt.sourceTimestampUs},
                   {"ageMs", std::isfinite(result.nt.ageMs) ? wpi::json(result.nt.ageMs) : wpi::json(nullptr)},
                   {"leftY", result.nt.leftY}, {"leftX", result.nt.leftX}, {"rightX", result.nt.rightX},
                   {"accepted", result.nt.accepted}}},
          {"source", result.source}, {"available", result.available}, {"authorized", result.authorized}};
    });
    return detail::DecodeDriverInput(input, driveRequestObserver);
  }
} // namespace base
