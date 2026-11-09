#pragma once

#include "base/RuntimeMode.h"
#include <frc2/command/Command.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

namespace base
{
// Platform-neutral lifecycle for an optional robot-specific simulation scene.
// The default scene is inert and schedules commands normally. Implementations
// may retain non-owning references only while their owner remains alive.
class RuntimeScene
{
public:
  virtual ~RuntimeScene() = default;
  virtual void Initialize() {}
  virtual void InitializeFeedback() {}
  virtual void FinishSetup() {}
  virtual void BindReset() {}
  virtual void Periodic() {}
  virtual void Reset() {}
  virtual bool Ready() const { return true; }
  virtual void Schedule(frc2::Command &command) { command.Schedule(); }
  // Optional phase hooks for applications that coordinate an external
  // simulation around scheduler and scene work. Real/replay scenes stay inert.
  virtual void BeforeScheduler() {}
  virtual void AfterScheduler() {}
  virtual void BeforeScene() {}
  virtual void AfterScene() {}
};

// Construct the provider only in live simulation. The default concrete Scene
// is the no-op implementation for real and replay, and may extend RuntimeScene
// with robot-specific provider methods. A missing live provider is an error.
template <typename Scene, typename Provider>
std::unique_ptr<Scene> MakeRuntimeScene(Mode requested, Provider &&provider)
{
  return SelectRuntimeIO<std::unique_ptr<Scene>>(
      requested,
      [] { return std::make_unique<Scene>(); },
      [&provider]() -> std::unique_ptr<Scene> {
        auto scene = std::invoke(std::forward<Provider>(provider));
        if (!scene) throw std::runtime_error("Simulation scene provider returned null");
        return scene;
      },
      [] { return std::make_unique<Scene>(); });
}

template <typename Scene, typename Provider>
std::unique_ptr<Scene> MakeRuntimeScene(Provider &&provider)
{
  return MakeRuntimeScene<Scene>(CurrentRuntimeMode(), std::forward<Provider>(provider));
}
} // namespace base
