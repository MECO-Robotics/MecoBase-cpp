#include "base/RuntimeMode.h"
#include "base/Replay.h"

#include <frc/RobotBase.h>
#include <stdexcept>

namespace base
{
Mode ResolveRuntimeMode(Mode requested)
{
    if (replay::Active())
        return Mode::Replay;
    switch (requested)
    {
    case Mode::Real:
    case Mode::Sim:
    case Mode::Replay:
        return requested;
    }
    throw std::invalid_argument("Invalid runtime mode");
}
Mode CurrentRuntimeMode()
{
    return ResolveRuntimeMode(frc::RobotBase::IsSimulation() ? Mode::Sim : Mode::Real);
}
} // namespace base
