#include "base/Vision.h"
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <frc/RobotController.h>
#include <networktables/NetworkTableInstance.h>
namespace base
{
class LimelightIO final : public VisionIO
{
  public:
    LimelightIO(VisionConfig config, std::function<frc::Rotation2d()> heading);
    void UpdateInputs(VisionInputs &) override;
    void SetVisionFilter(const VisionFilterTuning &) override;

  private:
    std::pair<bool, double> Status() const;
    VisionConfig config_;
    std::function<frc::Rotation2d()> heading_;
    std::shared_ptr<nt::NetworkTable> table_;
    nt::DoubleArraySubscriber poses_;
    nt::DoubleSubscriber heartbeat_, targetYaw_;
    std::shared_ptr<nt::DoubleArrayPublisher> orientation_, cameraPose_;
};
class PhotonIO final : public VisionIO
{
  public:
    PhotonIO(VisionConfig config, frc::AprilTagFieldLayout layout);
    void UpdateInputs(VisionInputs &) override;
    void SetVisionFilter(const VisionFilterTuning &) override;

  private:
    bool Connected();
    VisionConfig config_;
    std::unique_ptr<photon::PhotonCamera> camera_;
    std::unique_ptr<photon::PhotonPoseEstimator> estimator_;
};
namespace
{
void Validate(const VisionConfig &c)
{
    if (c.name.empty() || c.minTagCount < 1 || c.minAllowedTagCount < 1)
        throw std::invalid_argument("Invalid vision name/tag counts");
    for (double v : {c.fieldLength, c.fieldWidth, c.maxZ, c.linearStdDevBaseline, c.angularStdDevBaseline,
                     c.cameraStdDevFactor, c.megatag2LinearFactor})
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Invalid vision geometry/covariance");
    if (!std::isfinite(c.maxSingleTagAmbiguity) || c.maxSingleTagAmbiguity < 0)
        throw std::invalid_argument("Invalid vision ambiguity");
    const auto q = c.robotToCamera.Rotation().GetQuaternion();
    for (double v : {c.robotToCamera.X().value(), c.robotToCamera.Y().value(), c.robotToCamera.Z().value(), q.W(),
                     q.X(), q.Y(), q.Z()})
        if (!std::isfinite(v))
            throw std::invalid_argument("Invalid vision mount");
    for (int id : c.allowedTagIds)
        if (id <= 0)
            throw std::invalid_argument("Invalid vision tag ID");
}
void UpdateFilter(VisionConfig &config, const VisionFilterTuning &t)
{
    auto next = config;
    next.maxZ = t.maxZ;
    next.maxSingleTagAmbiguity = t.maxSingleTagAmbiguity;
    next.minTagCount = t.minTagCount;
    next.minAllowedTagCount = t.minAllowedTagCount;
    next.linearStdDevBaseline = t.linearStdDevBaseline;
    next.angularStdDevBaseline = t.angularStdDevBaseline;
    next.cameraStdDevFactor = t.cameraStdDevFactor;
    next.megatag2LinearFactor = t.megatag2LinearFactor;
    Validate(next);
    config = std::move(next);
}
bool Allowed(const VisionConfig &c, int id)
{
    return id > 0 && (c.allowedTagIds.empty() ||
                      std::find(c.allowedTagIds.begin(), c.allowedTagIds.end(), id) != c.allowedTagIds.end());
}
wpi::json Encode(const VisionObservation &o)
{
    auto deviations = wpi::json::array();
    for (double value : o.stdDevs)
        deviations.push_back(std::isfinite(value) ? wpi::json(value) : wpi::json(nullptr));
    return {{"pose", {o.pose.X().value(), o.pose.Y().value(), o.pose.Rotation().Radians().value()}},
            {"timestamp", o.timestamp.value()},
            {"stdDevs", deviations}};
}
VisionObservation Decode(const wpi::json &o)
{
    const auto &p = o.at("pose");
    const auto &d = o.at("stdDevs");
    if (p.size() != 3 || d.size() != 3)
        throw std::runtime_error("Invalid replay vision observation");
    std::array<double, 3> deviations;
    for (size_t i = 0; i < 3; ++i)
        deviations[i] = d.at(i).is_null() ? std::numeric_limits<double>::infinity() : d.at(i).get<double>();
    return {frc::Pose2d{units::meter_t{p.at(0).get<double>()}, units::meter_t{p.at(1).get<double>()},
                        frc::Rotation2d{units::radian_t{p.at(2).get<double>()}}},
            units::second_t{o.at("timestamp").get<double>()}, deviations};
}
} // namespace
LimelightIO::LimelightIO(VisionConfig config, std::function<frc::Rotation2d()> heading)
    : config_(std::move(config)), heading_(std::move(heading))
{
    Validate(config_);
    if (config_.megatag2 && !heading_)
        throw std::invalid_argument("MegaTag 2 requires heading supplier");
    if (replay::Active())
        return;
    table_ = nt::NetworkTableInstance::GetDefault().GetTable(config_.name);
    poses_ = table_->GetDoubleArrayTopic(config_.megatag2 ? "botpose_orb_wpiblue" : "botpose_wpiblue").Subscribe({});
    heartbeat_ = table_->GetDoubleTopic("hb").Subscribe(0);
    targetYaw_ = table_->GetDoubleTopic("tx").Subscribe(0);
    orientation_ =
        std::make_shared<nt::DoubleArrayPublisher>(table_->GetDoubleArrayTopic("robot_orientation_set").Publish());
    cameraPose_ =
        std::make_shared<nt::DoubleArrayPublisher>(table_->GetDoubleArrayTopic("camerapose_robotspace_set").Publish());
    const auto &t = config_.robotToCamera;
    const auto cameraPose = std::array<double, 6>{t.X().value(),
                                                  t.Y().value(),
                                                  t.Z().value(),
                                                  units::degree_t{t.Rotation().X()}.value(),
                                                  units::degree_t{t.Rotation().Y()}.value(),
                                                  units::degree_t{t.Rotation().Z()}.value()};
    telemetry::Post("VisionConfig/" + config_.name,
                    [publisher = cameraPose_, cameraPose] { publisher->Set(cameraPose); });
}
void LimelightIO::SetVisionFilter(const VisionFilterTuning &tuning) { UpdateFilter(config_, tuning); }
void PhotonIO::SetVisionFilter(const VisionFilterTuning &tuning) { UpdateFilter(config_, tuning); }
std::pair<bool, double> LimelightIO::Status() const
{
    const auto status = replay::ProcessInputs("Vision/" + config_.name + "/Status", [this] {
        const auto changed = heartbeat_.GetLastChange();
        const auto yaw = targetYaw_.Get();
        return wpi::json{{"connected", changed > 0 && frc::RobotController::GetFPGATime() - changed < 250000},
                         {"yaw", std::isfinite(yaw) ? wpi::json(yaw) : wpi::json(nullptr)}};
    });
    return {status.at("connected").get<bool>(),
            status.at("yaw").is_null() ? std::numeric_limits<double>::quiet_NaN() : status.at("yaw").get<double>()};
}
void LimelightIO::UpdateInputs(VisionInputs &inputs)
{
    const auto status = Status();
    inputs.connected = status.first;
    inputs.targetYawDegrees = status.second;
    if (config_.megatag2)
    {
        const auto orientation = std::array<double, 6>{heading_().Degrees().value(), 0, 0, 0, 0, 0};
        replay::RecordOutput("Vision/" + config_.name + "/Orientation", orientation);
        if (!replay::Active())
            telemetry::Post("VisionOrientation/" + config_.name,
                            [publisher = orientation_, orientation] { publisher->Set(orientation); });
    }
    const auto observations = replay::ProcessInputs("Vision/" + config_.name + "/Observations", [this] {
        auto accepted = wpi::json::array();
        for (const auto &sample : poses_.ReadQueue())
        {
            const auto &v = sample.value;
            if (v.size() < 11)
                continue;
            bool finite = true;
            for (double value : v)
                finite &= std::isfinite(value);
            if (sample.time <= 0 || !finite || v[7] < config_.minTagCount || std::floor(v[7]) != v[7] || v[9] <= 0 ||
                v[6] < 0 || std::abs(v[2]) > config_.maxZ || v[0] < 0 || v[0] > config_.fieldLength || v[1] < 0 ||
                v[1] > config_.fieldWidth)
                continue;
            // MT1 needs usable single-tag ambiguity (raw fiducial element 17).
            // MT2 resolves this ambiguity using the supplied robot heading.
            if (!config_.megatag2 && v[7] == 1 &&
                (v.size() <= 17 || v[17] < 0 || v[17] > config_.maxSingleTagAmbiguity))
                continue;
            if (!config_.allowedTagIds.empty())
            {
                int allowed = 0;
                for (size_t i = 11; i + 6 < v.size(); i += 7)
                    if (v[i] > 0 && v[i] <= INT32_MAX && std::floor(v[i]) == v[i] &&
                        Allowed(config_, static_cast<int>(v[i])))
                        ++allowed;
                if (allowed < config_.minAllowedTagCount)
                    continue;
            }
            const double scale = v[9] * v[9] / v[7];
            if (!std::isfinite(scale) || scale <= 0)
                continue;
            VisionObservation observation{
                frc::Pose2d{units::meter_t{v[0]}, units::meter_t{v[1]}, frc::Rotation2d{units::degree_t{v[5]}}},
                units::second_t{sample.time * 1e-6 - v[6] * 1e-3},
                {config_.linearStdDevBaseline * scale * config_.cameraStdDevFactor *
                     (config_.megatag2 ? config_.megatag2LinearFactor : 1),
                 config_.linearStdDevBaseline * scale * config_.cameraStdDevFactor *
                     (config_.megatag2 ? config_.megatag2LinearFactor : 1),
                 config_.megatag2 ? std::numeric_limits<double>::infinity()
                                  : config_.angularStdDevBaseline * scale * config_.cameraStdDevFactor}};
            if (observation.timestamp.value() < 0 ||
                observation.timestamp.value() > frc::RobotController::GetFPGATime() * 1e-6 ||
                !std::isfinite(observation.stdDevs[0]) || (!config_.megatag2 && !std::isfinite(observation.stdDevs[2])))
                continue;
            accepted.push_back(Encode(observation));
        }
        return accepted;
    });
    for (const auto &observation : observations)
        inputs.observations.push_back(Decode(observation));
}
} // namespace base

namespace base
{
PhotonIO::PhotonIO(VisionConfig config, frc::AprilTagFieldLayout layout) : config_(std::move(config))
{
    Validate(config_);
    if (!replay::Active())
    {
        camera_ = std::make_unique<photon::PhotonCamera>(config_.name);
        estimator_ = std::make_unique<photon::PhotonPoseEstimator>(std::move(layout), config_.robotToCamera);
    }
}
bool PhotonIO::Connected()
{
    return replay::ProcessInputs("PhotonVision/" + config_.name + "/Connected",
                                 [this] { return wpi::json(camera_->IsConnected()); })
        .get<bool>();
}
void PhotonIO::UpdateInputs(VisionInputs &inputs)
{
    inputs.connected = Connected();
    const auto observations = replay::ProcessInputs("PhotonVision/" + config_.name + "/Observations", [this] {
        auto accepted = wpi::json::array();
        for (const auto &result : camera_->GetAllUnreadResults())
        {
            auto estimate = estimator_->EstimateCoprocMultiTagPose(result);
            if (!estimate)
                estimate = estimator_->EstimateLowestAmbiguityPose(result);
            if (!estimate || estimate->targetsUsed.size() < static_cast<size_t>(config_.minTagCount))
                continue;
            if (!config_.allowedTagIds.empty())
            {
                int allowed = 0;
                for (const auto &target : estimate->targetsUsed)
                    if (Allowed(config_, target.GetFiducialId()))
                        ++allowed;
                if (allowed < config_.minAllowedTagCount)
                    continue;
            }
            const auto &pose = estimate->estimatedPose;
            if (!std::isfinite(pose.X().value()) || !std::isfinite(pose.Y().value()) ||
                !std::isfinite(pose.Z().value()) || !std::isfinite(pose.Rotation().Z().value()) ||
                std::abs(pose.Z().value()) > config_.maxZ || pose.X().value() < 0 || pose.Y().value() < 0 ||
                pose.X().value() > config_.fieldLength || pose.Y().value() > config_.fieldWidth)
                continue;
            if (!std::isfinite(estimate->timestamp.value()) || estimate->timestamp.value() < 0 ||
                estimate->timestamp.value() > frc::RobotController::GetFPGATime() * 1e-6)
                continue;
            if (estimate->targetsUsed.size() == 1)
            {
                const double ambiguity = estimate->targetsUsed[0].GetPoseAmbiguity();
                if (!std::isfinite(ambiguity) || ambiguity < 0 || ambiguity > config_.maxSingleTagAmbiguity)
                    continue;
            }
            double distance = 0;
            for (const auto &target : estimate->targetsUsed)
                distance += target.GetBestCameraToTarget().Translation().Norm().value();
            distance /= estimate->targetsUsed.size();
            const double scale = distance * distance / estimate->targetsUsed.size();
            if (!std::isfinite(scale) || scale <= 0)
                continue;
            if (!std::isfinite(config_.linearStdDevBaseline * scale * config_.cameraStdDevFactor) ||
                !std::isfinite(config_.angularStdDevBaseline * scale * config_.cameraStdDevFactor))
                continue;
            accepted.push_back(Encode({pose.ToPose2d(),
                                       estimate->timestamp,
                                       {config_.linearStdDevBaseline * scale * config_.cameraStdDevFactor,
                                        config_.linearStdDevBaseline * scale * config_.cameraStdDevFactor,
                                        config_.angularStdDevBaseline * scale * config_.cameraStdDevFactor}}));
        }
        return accepted;
    });
    for (const auto &observation : observations)
        inputs.observations.push_back(Decode(observation));
}
} // namespace base

namespace base
{
VisionSystem::VisionSystem(std::vector<std::unique_ptr<VisionIO>> sources, Consumer consumer)
    : sources_(std::move(sources)), inputs_(sources_.size()), consumer_(std::move(consumer))
{
    for (const auto &source : sources_)
        if (!source)
            throw std::invalid_argument("Null vision IO");
}
VisionSystem::VisionSystem(std::unique_ptr<VisionIO> source, Consumer consumer) : consumer_(std::move(consumer))
{
    if (!source)
        throw std::invalid_argument("Null vision IO");
    sources_.push_back(std::move(source));
    inputs_.resize(1);
}
void VisionSystem::SetVisionFilter(size_t source, const VisionFilterTuning &tuning)
{
    sources_.at(source)->SetVisionFilter(tuning);
}
void VisionSystem::SetQuestFilter(size_t source, const QuestFilterTuning &tuning)
{
    sources_.at(source)->SetQuestFilter(tuning);
}
bool VisionSystem::AlignToField(size_t source, const frc::Pose2d &pose)
{
    const bool accepted = sources_.at(source)->AlignToField(pose);
    if (accepted)
        inputs_.at(source).aligned = true;
    return accepted;
}
void VisionSystem::Periodic()
{
    for (size_t i = 0; i < sources_.size(); ++i)
    {
        auto &input = inputs_[i];
        input.observations.clear();
        sources_[i]->UpdateInputs(input);
        if (consumer_)
            for (const auto &observation : input.observations)
                consumer_(observation);
    }
}
std::unique_ptr<VisionIO> MakeLimelightVisionIO(VisionConfig config, std::function<frc::Rotation2d()> heading)
{
    return std::make_unique<LimelightIO>(std::move(config), std::move(heading));
}
std::unique_ptr<VisionIO> MakePhotonVisionIO(VisionConfig config, frc::AprilTagFieldLayout layout)
{
    return std::make_unique<PhotonIO>(std::move(config), std::move(layout));
}
Vision::Vision(VisionConfig config, Consumer consumer, std::function<frc::Rotation2d()> heading)
    : VisionSystem(MakeLimelightVisionIO(std::move(config), std::move(heading)), std::move(consumer))
{
}
PhotonVision::PhotonVision(VisionConfig config, frc::AprilTagFieldLayout layout, Consumer consumer)
    : VisionSystem(MakePhotonVisionIO(std::move(config), std::move(layout)), std::move(consumer))
{
}
} // namespace base
