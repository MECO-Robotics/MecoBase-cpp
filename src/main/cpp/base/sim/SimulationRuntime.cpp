#include "base/SimulationRuntime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace base::simulation
{
struct SimulationRuntime::Registry
{
    std::uint64_t next{};
    std::map<std::uint64_t, PlantCallbacks> callbacks;
};

Registration::Registration(std::function<void()> release) : release_(std::move(release)) {}
Registration::~Registration()
{
    if (release_)
        release_();
}
Registration::Registration(Registration &&other) noexcept : release_(std::exchange(other.release_, {})) {}
Registration &Registration::operator=(Registration &&other) noexcept
{
    if (this != &other)
    {
        if (release_)
            release_();
        release_ = std::exchange(other.release_, {});
    }
    return *this;
}

SimulationRuntime::SimulationRuntime() : registry_(std::make_shared<Registry>()) {}
SimulationRuntime::~SimulationRuntime() = default;

Registration SimulationRuntime::Register(PlantCallbacks callbacks)
{
    const auto id = registry_->next++;
    registry_->callbacks.emplace(id, std::move(callbacks));
    return Registration([weak = std::weak_ptr<Registry>(registry_), id] {
        if (auto registry = weak.lock())
            registry->callbacks.erase(id);
    });
}

void SimulationRuntime::SetBusVoltage(double voltage)
{
    if (!std::isfinite(voltage) || voltage < 0 || voltage > 16)
        throw std::invalid_argument("Simulation bus voltage must be in [0,16]");
    busVoltage_ = voltage;
}

void SimulationRuntime::StepRobotPeriod()
{
    using Clock = std::chrono::steady_clock;
    for (int step = 0; step < 5; ++step)
    {
        const auto start = Clock::now();
        std::vector<std::uint64_t> ids;
        ids.reserve(registry_->callbacks.size());
        for (const auto &[id, callback] : registry_->callbacks)
            ids.push_back(id);

        auto invoke = [&](auto select) {
            for (const auto id : ids)
                if (const auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
                {
                    auto callback = select(it->second);
                    if (callback)
                        callback(.004);
                }
        };
        invoke([](const PlantCallbacks &callbacks) { return callbacks.beforePhysics; });
        invoke([](const PlantCallbacks &callbacks) { return callbacks.physicsStep; });
        invoke([](const PlantCallbacks &callbacks) { return callbacks.afterPhysics; });

        time_ += .004;
        const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        timing_.seconds = elapsed;
        timing_.maximumSeconds = std::max(timing_.maximumSeconds, elapsed);
        ++timing_.substeps;
        if (elapsed > .004)
            ++timing_.deadlineMisses;
    }

    supplyCurrent_ = 0;
    std::vector<std::uint64_t> ids;
    ids.reserve(registry_->callbacks.size());
    for (const auto &[id, callback] : registry_->callbacks)
        ids.push_back(id);
    for (const auto id : ids)
        if (const auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
        {
            auto callback = it->second.supplyCurrent;
            if (callback)
            {
                const double current = callback();
                if (!std::isfinite(current) || current < 0)
                    throw std::runtime_error("Invalid simulated supply current");
                if (!std::isfinite(supplyCurrent_ + current))
                    throw std::runtime_error("Simulated aggregate supply current overflow");
                supplyCurrent_ += current;
            }
        }
}

void SimulationRuntime::Reset()
{
    time_ = 0;
    supplyCurrent_ = 0;
    busVoltage_ = 12;
    timing_ = {};
    std::vector<std::uint64_t> ids;
    ids.reserve(registry_->callbacks.size());
    for (const auto &[id, callback] : registry_->callbacks)
        ids.push_back(id);
    for (const auto id : ids)
        if (const auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
        {
            auto callback = it->second.reset;
            if (callback)
                callback();
        }
}

SimulationRuntime &GetSimulationRuntime()
{
    static SimulationRuntime runtime;
    return runtime;
}
} // namespace base::simulation
