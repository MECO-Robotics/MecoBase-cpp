#include "base/VisionIOSelector.h"
#include "base/RuntimeMode.h"
#include <stdexcept>
#include <utility>

namespace base {
namespace {
std::unique_ptr<VisionIO> SimVision(const SimVisionProvider& provider) {
    if (!provider) return std::make_unique<InjectedVisionIO>(InjectedVisionIO::Source{});
    auto io = provider();
    if (!io) throw std::invalid_argument("Simulation vision provider returned null");
    return io;
}
std::unique_ptr<ObjectDetectionIO> SimDetection(const SimObjectDetectionProvider& provider) {
    if (!provider) return std::make_unique<InjectedObjectDetectionIO>([] { return std::nullopt; });
    auto io = provider();
    if (!io) throw std::invalid_argument("Simulation object detection provider returned null");
    return io;
}
}
std::unique_ptr<VisionIO> MakeRuntimeLimelightVisionIO(
    VisionConfig config, std::function<frc::Rotation2d()> heading, SimVisionProvider simulation) {
    const auto liveOrRecorded = [&] { return MakeLimelightVisionIO(std::move(config), std::move(heading)); };
    return SelectRuntimeIO<std::unique_ptr<VisionIO>>(
        liveOrRecorded, [&] { return SimVision(simulation); }, liveOrRecorded);
}
std::unique_ptr<VisionIO> MakeRuntimeQuestNavVisionIO(QuestNavConfig config, SimVisionProvider simulation) {
    const auto liveOrRecorded = [&] { return MakeQuestNavVisionIO(std::move(config)); };
    return SelectRuntimeIO<std::unique_ptr<VisionIO>>(
        liveOrRecorded, [&] { return SimVision(simulation); }, liveOrRecorded);
}
std::unique_ptr<ObjectDetectionIO> MakeRuntimeObjectDetectionIO(
    std::string topic, units::second_t updatePeriod, SimObjectDetectionProvider simulation) {
    return SelectRuntimeIO<std::unique_ptr<ObjectDetectionIO>>(
        [&] { return MakeNtObjectDetectionIO(std::move(topic), updatePeriod); },
        [&] { return SimDetection(simulation); },
        [] { return std::make_unique<InjectedObjectDetectionIO>([] { return std::nullopt; }); });
}
} // namespace base
