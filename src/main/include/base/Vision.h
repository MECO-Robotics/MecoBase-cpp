#pragma once
#include <stdexcept>
#include <functional>
#include <utility>
#include <memory>
#include <string>
#include <array>
#include <vector>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc2/command/SubsystemBase.h>
namespace base
{
struct VisionObservation
{
    frc::Pose2d pose;
    units::second_t timestamp;
    std::array<double, 3> stdDevs;
};
struct VisionConfig
{
    std::string name = "limelight";
    frc::Transform3d robotToCamera;
    double fieldLength = 17.548;
    double fieldWidth = 8.052;
    double maxZ = 0.75;
    double maxSingleTagAmbiguity = 0.3;
    bool megatag2 = false;
    int minTagCount = 1;
    std::vector<int> allowedTagIds; // Empty accepts all positive IDs.
    int minAllowedTagCount = 1;
    double linearStdDevBaseline = .02;
    double angularStdDevBaseline = .06;
    double cameraStdDevFactor = 1;
    double megatag2LinearFactor = 1; // Preserve existing C++ covariance by default.
};
// Runtime-only filter settings; mounts, camera identity, and field geometry stay fixed.
struct VisionFilterTuning
{
    double maxZ = .75, maxSingleTagAmbiguity = .3;
    int minTagCount = 1, minAllowedTagCount = 1;
    double linearStdDevBaseline = .02, angularStdDevBaseline = .06;
    double cameraStdDevFactor = 1, megatag2LinearFactor = 1;
    bool operator==(const VisionFilterTuning &) const = default;
};
struct QuestFilterTuning
{
    double maxZ = .75, freshnessSeconds = .12;
    std::array<double, 3> stdDevs{.1, .1, .05};
    bool operator==(const QuestFilterTuning &) const = default;
};
struct VisionInputs
{
    std::vector<VisionObservation> observations;
    bool connected{}, tracking{}, aligned{};
    double targetYawDegrees{};
};
// One source update per scheduler tick. Status reads are passive; adapters own
// acquisition/filtering and retain their existing replay boundary.
class VisionIO
{
  public:
    virtual ~VisionIO() = default;
    virtual void UpdateInputs(VisionInputs &inputs) = 0;
    virtual void SetVisionFilter(const VisionFilterTuning &)
    {
        throw std::logic_error("Vision source does not support this filter");
    }
    virtual void SetQuestFilter(const QuestFilterTuning &)
    {
        throw std::logic_error("Vision source does not support this filter");
    }
    virtual bool AlignToField(const frc::Pose2d &)
    {
        return false;
    }
};
class InjectedVisionIO final : public VisionIO
{
  public:
    using Source = std::function<void(VisionInputs &)>;
    explicit InjectedVisionIO(Source source) : source_(std::move(source))
    {
    }
    void UpdateInputs(VisionInputs &inputs) override
    {
        if (source_)
            source_(inputs);
    }

  private:
    Source source_;
};
class VisionSystem : public frc2::SubsystemBase
{
  public:
    using Consumer = std::function<void(const VisionObservation &)>;
    explicit VisionSystem(std::vector<std::unique_ptr<VisionIO>> sources, Consumer consumer);
    explicit VisionSystem(std::unique_ptr<VisionIO> source, Consumer consumer);
    void Periodic() override;
    const VisionInputs &Inputs(size_t source = 0) const
    {
        return inputs_.at(source);
    }
    bool AlignToField(size_t source, const frc::Pose2d &pose);
    void SetVisionFilter(size_t source, const VisionFilterTuning &tuning);
    void SetQuestFilter(size_t source, const QuestFilterTuning &tuning);

  private:
    std::vector<std::unique_ptr<VisionIO>> sources_;
    std::vector<VisionInputs> inputs_;
    Consumer consumer_;
};
std::unique_ptr<VisionIO> MakeLimelightVisionIO(VisionConfig config, std::function<frc::Rotation2d()> heading = {});
std::unique_ptr<VisionIO> MakePhotonVisionIO(VisionConfig config, frc::AprilTagFieldLayout layout);
class Vision : public VisionSystem
{
  public:
    Vision(VisionConfig config, Consumer consumer, std::function<frc::Rotation2d()> heading = {});
    bool Connected() const
    {
        return Inputs().connected;
    }
    double TargetYawDegrees() const
    {
        return Inputs().targetYawDegrees;
    }
};
class PhotonVision : public VisionSystem
{
  public:
    PhotonVision(VisionConfig config, frc::AprilTagFieldLayout layout, Consumer consumer);
    bool Connected() const
    {
        return Inputs().connected;
    }
};
} // namespace base
