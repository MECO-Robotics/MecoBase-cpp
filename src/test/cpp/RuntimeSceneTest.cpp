#include "base/RuntimeScene.h"
#include <gtest/gtest.h>

namespace {
struct Scene : base::RuntimeScene {
  int value = 0;
};
struct SimScene : Scene {
  SimScene() { value = 17; }
};
}

TEST(RuntimeScene, CreatesProviderOnlyForSimulation) {
  int calls = 0;
  auto provider = [&]() -> std::unique_ptr<Scene> { ++calls; return std::make_unique<SimScene>(); };
  auto real = base::MakeRuntimeScene<Scene>(base::Mode::Real, provider);
  auto replay = base::MakeRuntimeScene<Scene>(base::Mode::Replay, provider);
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(real->value, 0);
  EXPECT_EQ(replay->value, 0);
  EXPECT_TRUE(real->Ready());
  EXPECT_TRUE(replay->Ready());
  auto simulation = base::MakeRuntimeScene<Scene>(base::Mode::Sim, provider);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(simulation->value, 17);
}

TEST(RuntimeScene, MissingSimulationProviderFails) {
  EXPECT_THROW((base::MakeRuntimeScene<Scene>(base::Mode::Sim, [] { return std::unique_ptr<Scene>{}; })), std::runtime_error);
}
