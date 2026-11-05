#include "base/Mechanism.h"
#include "base/Replay.h"
#include "MechanismHardwareInternal.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace base
{

void MotorSimulationConfig::Validate() const
{
    auto finite = [](double value) { return std::isfinite(value); };
    if (!finite(outputInertia.value()) || outputInertia.value() <= 0 || !finite(mass.value()) || mass.value() < 0 ||
        !finite(armCOM.value()) || armCOM.value() < 0 || !finite(armAngleOffset.value()) ||
        !finite(drumRadius.value()) || drumRadius.value() < 0 || !finite(initialPosition.value()) ||
        (minimum && !finite(minimum->value())) || (maximum && !finite(maximum->value())) ||
        (minimum && maximum && *minimum >= *maximum) || (minimum && initialPosition < *minimum) ||
        (maximum && initialPosition > *maximum) || (load == Load::Arm && (mass.value() <= 0 || armCOM.value() <= 0)) ||
        (load == Load::Elevator && (mass.value() <= 0 || drumRadius.value() <= 0)) ||
        (load != Load::Free && load != Load::Arm && load != Load::Elevator))
        throw std::invalid_argument("Invalid mechanism simulation configuration");
}
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config, double outputInertia)
{
    MotorSimulationConfig simulation;
    simulation.outputInertia = units::kilogram_square_meter_t{outputInertia};
    return MakeMotorIO(mode, vendor, config, simulation);
}
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config,
                                     const MotorSimulationConfig &simulation)
{
    config.Validate();
    if (mode == Mode::Replay || replay::Active())
    {
        auto injected = std::make_unique<InjectedMotorIO>();
        injected->nativeEnabled = false; // Source frames select firmware versus emulation.
        return injected;
    }
    if (mode == Mode::Sim)
        return detail::MakeSimMotorIO(config, vendor, simulation);

    if (vendor == Vendor::SparkMax)
        return MakeAsyncMotorIO(detail::MakeSparkMotorIO(config));
    return MakeAsyncMotorIO(detail::MakeTalonMotorIO(config));
}

MotorIOSelector::MotorIOSelector(SimulationProvider simulation)
    : simulation_(std::move(simulation))
{
}
std::unique_ptr<MotorIO> MotorIOSelector::Make(Vendor vendor, const MotorConfig &config) const
{
    return Make(CurrentRuntimeMode(), vendor, config);
}
std::unique_ptr<MotorIO> MotorIOSelector::Make(Mode mode, Vendor vendor, const MotorConfig &config) const
{
    config.Validate();
    return SelectRuntimeIO<std::unique_ptr<MotorIO>>(
        mode,
        [&] { return MakeMotorIO(Mode::Real, vendor, config); },
        [&]() -> std::unique_ptr<MotorIO> {
            if (!simulation_)
                return MakeMotorIO(Mode::Sim, vendor, config);
            auto io = simulation_(vendor, config);
            if (!io)
                throw std::invalid_argument("Simulation motor provider returned null");
            return io;
        },
        [&] { return MakeMotorIO(Mode::Replay, vendor, config); });
}
std::unique_ptr<MotorIO> MakeMotorIO(Vendor vendor, const MotorConfig &config)
{
    return MotorIOSelector{}.Make(vendor, config);
}
} // namespace base
