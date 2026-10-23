#pragma once

#include <cstdint>
#include <functional>
#include <memory>

namespace base::simulation
{
struct StepTiming
{
    double seconds{}, maximumSeconds{};
    std::uint64_t substeps{}, deadlineMisses{};
};

struct PlantCallbacks
{
    std::function<void(double)> beforePhysics;
    std::function<void(double)> physicsStep;
    std::function<void(double)> afterPhysics;
    std::function<void()> reset;
    std::function<double()> supplyCurrent;
};

class Registration
{
  public:
    Registration() = default;
    ~Registration();
    Registration(Registration &&) noexcept;
    Registration &operator=(Registration &&) noexcept;
    Registration(const Registration &) = delete;
    Registration &operator=(const Registration &) = delete;

  private:
    friend class SimulationRuntime;
    explicit Registration(std::function<void()> release);
    std::function<void()> release_;
};

// Jolt-free deterministic runtime for motor simulation and optional physics
// adapters. Callbacks are owned and advanced on one thread.
class SimulationRuntime
{
  public:
    SimulationRuntime();
    ~SimulationRuntime();
    Registration Register(PlantCallbacks);
    void StepRobotPeriod();
    void Reset();
    double Time() const { return time_; }
    double SupplyCurrent() const { return supplyCurrent_; }
    double BusVoltage() const { return busVoltage_; }
    void SetBusVoltage(double);
    const StepTiming &Timing() const { return timing_; }

  private:
    struct Registry;
    std::shared_ptr<Registry> registry_;
    double time_{};
    double supplyCurrent_{}, busVoltage_{12};
    StepTiming timing_{};
};

SimulationRuntime &GetSimulationRuntime();
} // namespace base::simulation
