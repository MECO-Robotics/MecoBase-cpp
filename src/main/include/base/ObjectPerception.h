#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <cstdint>
#include <utility>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace base {
struct ObjectDetection { frc::Translation3d cameraPoint; double confidence = 1.; };
struct ObjectDetectionFrame {
    uint64_t sequence{};
    units::second_t captureTime{}; // local FPGA clock, not Jetson wall time
    std::vector<ObjectDetection> detections;
};
class ObjectDetectionIO {
 public:
    virtual ~ObjectDetectionIO() = default;
    virtual std::optional<ObjectDetectionFrame> ReadFrame() = 0;
};
class InjectedObjectDetectionIO final : public ObjectDetectionIO {
 public:
    using Source = std::function<std::optional<ObjectDetectionFrame>()>;
    explicit InjectedObjectDetectionIO(Source source) : source_(std::move(source)) {}
    std::optional<ObjectDetectionFrame> ReadFrame() override { return source_(); }
 private:
    Source source_;
};
// NT DoubleArray: [1, sequence, capture-to-publication latency seconds,
//                  camera X metres, Y metres, Z metres, confidence, ...].
// Camera coordinates are WPILib: X forward, Y left, Z up. Clock conversion uses
// the NT sample timestamp and local FPGA/NT clocks; no shared wall clock needed.
std::unique_ptr<ObjectDetectionIO> MakeNtObjectDetectionIO(std::string topic);
struct ObjectCamera {
    std::string name;
    std::unique_ptr<ObjectDetectionIO> io;
    // null means mounting calibration is unavailable. Evaluated at each sample.
    std::function<std::optional<frc::Transform3d>()> robotToCamera;
};
struct ObjectCluster {
    frc::Translation2d position;
    size_t observations{};
    double confidence{};
    units::second_t captureTime{};
};
struct ObjectPerceptionConfig {
    units::second_t freshness{.15};
    units::meter_t maxDistance{6}, clusterRadius{.30};
    double minimumConfidence = .5;
};
// Fuses anonymous camera observations through capture-time robot and mount pose.
// Mounted mechanisms supply live transforms; no static-extrinsic assumption.
class ObjectPerception : public frc2::SubsystemBase {
 public:
    ObjectPerception(std::vector<ObjectCamera> cameras, std::function<frc::Pose3d()> robotPose,
                     ObjectPerceptionConfig config = {});
    ~ObjectPerception() override;
    void Periodic() override;
    void UpdateAt(units::second_t now, const frc::Pose3d& pose);
    void Clear();
    const std::vector<ObjectCluster>& Clusters() const;
 private:
    struct State;
    std::unique_ptr<State> state_;
};
}
