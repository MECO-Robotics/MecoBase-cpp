#pragma once

#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/RuntimeMode.h"

#include <functional>
#include <stdexcept>
#include <utility>

namespace base
{
// Configured retains the gains supplied by the robot in every environment.
// Automatic allows a simulation model to derive gains suitable for its plant.
enum class SimulationTuning { Automatic, Configured };

// A lazy desktop simulation configuration provider. The provider is never
// invoked on hardware or in replay and can be omitted on roboRIO builds.
inline DriveConfig ConfigureRuntimeDrive(DriveConfig config,
                                          std::function<DriveConfig(DriveConfig)> simulation = {})
{
    if (CurrentRuntimeMode() == Mode::Sim)
    {
        if (!simulation)
            throw std::invalid_argument("Simulation drive configuration provider is missing");
        return simulation(std::move(config));
    }
    return config;
}

enum class SimulationJointGravity { Modeled, Unmodeled };

// With automatic tuning, omit gravity feedforward only when the simulated
// plant has no gravity load. Configured tuning preserves all supplied gains.
inline Gains ConfigureRuntimeJointGains(Gains gains, SimulationTuning tuning,
                                        SimulationJointGravity gravity)
{
    if (tuning != SimulationTuning::Automatic && tuning != SimulationTuning::Configured)
        throw std::invalid_argument("Invalid simulation tuning selection");
    if (gravity != SimulationJointGravity::Modeled && gravity != SimulationJointGravity::Unmodeled)
        throw std::invalid_argument("Invalid simulation joint gravity model");
    if (CurrentRuntimeMode() == Mode::Sim && tuning == SimulationTuning::Automatic &&
        gravity == SimulationJointGravity::Unmodeled)
        gains.gravity = 0;
    return gains;
}
} // namespace base
