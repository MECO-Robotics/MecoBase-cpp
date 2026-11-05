#include "base/CanBusMode.h"

#include "base/RuntimeMode.h"
#include <ctre/phoenix6/CANBus.hpp>

namespace base
{
bool UseCanFdRates(const std::string &bus, bool probeDefaultBus)
{
    if (CurrentRuntimeMode() != Mode::Real)
        return true;
    return (probeDefaultBus || !bus.empty()) && ctre::phoenix6::CANBus{bus}.IsNetworkFD();
}
} // namespace base
