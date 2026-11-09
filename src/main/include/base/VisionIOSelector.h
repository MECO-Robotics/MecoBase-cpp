#pragma once

#include "base/ObjectPerception.h"
#include "base/QuestNav.h"
#include "base/Vision.h"
#include <functional>
#include <memory>
#include <string>

namespace base {
// Simulation providers are called only in simulation and must return a non-null
// IO. If omitted, simulation reports no observations. Replay never calls them
// or opens a live sensor. Recorded Limelight/Quest data retains its existing
// decoder; object-detection frames are captured at the ObjectPerception read
// boundary and restored from the replay frame.
using SimVisionProvider = std::function<std::unique_ptr<VisionIO>()>;
using SimObjectDetectionProvider = std::function<std::unique_ptr<ObjectDetectionIO>()>;

std::unique_ptr<VisionIO> MakeRuntimeLimelightVisionIO(
    VisionConfig config, std::function<frc::Rotation2d()> heading = {},
    SimVisionProvider simulation = {});
std::unique_ptr<VisionIO> MakeRuntimeQuestNavVisionIO(
    QuestNavConfig config, SimVisionProvider simulation = {});
std::unique_ptr<ObjectDetectionIO> MakeRuntimeObjectDetectionIO(
    std::string topic, units::second_t updatePeriod, SimObjectDetectionProvider simulation = {});
} // namespace base
