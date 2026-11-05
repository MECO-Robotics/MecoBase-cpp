#pragma once

#include <string>

namespace base
{
// Selects the rate profile for a CTRE CAN bus. Simulation and replay use the
// FD profile without opening a CAN bus. Set probeDefaultBus to false when an
// empty bus name should use the classic profile without querying hardware.
bool UseCanFdRates(const std::string &bus, bool probeDefaultBus = true);

template <typename T>
T SelectCanBusRate(const std::string &bus, T fd, T classic, bool probeDefaultBus = true)
{
    return UseCanFdRates(bus, probeDefaultBus) ? fd : classic;
}
} // namespace base
