#pragma once

#include <exception>
#include <functional>
#include <utility>

namespace base
{
enum class Mode
{
    Real,
    Sim,
    Replay
};

// Replay takes precedence over the requested platform mode.
Mode ResolveRuntimeMode(Mode requested);
// Determines the platform mode and applies replay precedence. Initialize replay
// before constructing any IO so no hardware or simulation is created in replay.
Mode CurrentRuntimeMode();

// Exactly one factory is invoked. Each must return a value convertible to T;
// callbacks and anything they capture must remain valid while invoked. T may be
// a move-only pointer or a value adapter. Factory failures propagate to caller.
template <typename T, typename RealFactory, typename SimFactory, typename ReplayFactory>
T SelectRuntimeIO(Mode requested, RealFactory &&real, SimFactory &&simulation, ReplayFactory &&replay)
{
    switch (ResolveRuntimeMode(requested))
    {
    case Mode::Real:
        return std::invoke(std::forward<RealFactory>(real));
    case Mode::Sim:
        return std::invoke(std::forward<SimFactory>(simulation));
    case Mode::Replay:
        return std::invoke(std::forward<ReplayFactory>(replay));
    }
    // ResolveRuntimeMode rejects invalid modes.
    std::terminate();
}

template <typename T, typename RealFactory, typename SimFactory, typename ReplayFactory>
T SelectRuntimeIO(RealFactory &&real, SimFactory &&simulation, ReplayFactory &&replay)
{
    return SelectRuntimeIO<T>(CurrentRuntimeMode(), std::forward<RealFactory>(real),
                              std::forward<SimFactory>(simulation), std::forward<ReplayFactory>(replay));
}
} // namespace base
