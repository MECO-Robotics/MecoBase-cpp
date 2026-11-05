#pragma once

#include "base/NtGamepad.h"
#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <cmath>
#include <string_view>

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
    std::string_view source = "none";
    bool available{};
    bool authorized{};

    // Reapply authority after stateful filters, which can emit buffered input.
    double WhileAuthorized(double value) const { return authorized ? value : 0.0; }
  };

  // Prefer fresh NT axes when configured. DS fallback is a caller-owned policy;
  // enable and emergency-stop authority always come from Driver Station. The
  // returned drive axes are neutral without both a valid source and authority.
  inline DriverInputState ReadDriverInput(frc::XboxController &controller,
                                          NtGamepad *ntGamepad,
                                          bool allowDriverStationFallback)
  {
    DriverInputState result;
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
    return result;
  }
} // namespace base
