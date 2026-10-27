#include <networktables/RawTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include "base/Vision.h"
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>
#include <limits>
#include <photon/serde/PhotonPipelineResultSerde.h>
#include <photon/networktables/NTTopicSet.h>

namespace
{
std::vector<double> SingleTagPose(double ambiguity)
{
    return {1, 2, 0, 0, 0, 30, 0, 1, 0, 2, 1, 1, 0, 0, 1, 2, 2, ambiguity};
}
} // namespace

TEST(VisionLimelight, SingleTagRequiresUsableAmbiguity)
{
    base::VisionConfig config;
    config.name = "test-limelight-single-tag";
    int observations = 0;
    base::Vision vision(config, [&](const auto &) { ++observations; });
    auto publisher =
        nt::NetworkTableInstance::GetDefault().GetTable(config.name)->GetDoubleArrayTopic("botpose_wpiblue").Publish();
    auto publish = [&](const std::vector<double> &pose) {
        publisher.Set(pose);
        vision.Periodic();
    };
    publish(SingleTagPose(config.maxSingleTagAmbiguity));
    ASSERT_EQ(observations, 1);
    publish(SingleTagPose(config.maxSingleTagAmbiguity + .01));
    publish(SingleTagPose(-1));
    auto truncated = SingleTagPose(0);
    truncated.resize(11);
    publish(truncated);
    EXPECT_EQ(observations, 1);
}

TEST(VisionLimelight, MegaTag2DoesNotUseSingleTagAmbiguity)
{
    base::VisionConfig config;
    config.name = "test-limelight-megatag2";
    config.megatag2 = true;
    int observations = 0;
    base::Vision vision(config, [&](const auto &) { ++observations; }, [] { return frc::Rotation2d{}; });
    auto publisher = nt::NetworkTableInstance::GetDefault()
                         .GetTable(config.name)
                         ->GetDoubleArrayTopic("botpose_orb_wpiblue")
                         .Publish();
    publisher.Set(SingleTagPose(.9));
    vision.Periodic();
    EXPECT_EQ(observations, 1);
}

TEST(VisionLimelight, RejectsOverflowingCovariance)
{
    base::VisionConfig config;
    config.name = "test-limelight-covariance";
    int observations = 0;
    base::Vision vision(config, [&](const auto &) { ++observations; });
    auto publisher =
        nt::NetworkTableInstance::GetDefault().GetTable(config.name)->GetDoubleArrayTopic("botpose_wpiblue").Publish();
    auto pose = SingleTagPose(0);
    pose[9] = std::numeric_limits<double>::max();
    publisher.Set(pose);
    vision.Periodic();
    EXPECT_EQ(observations, 0);
}

TEST(VisionPhoton, RejectsInvalidDistanceWithFiniteCoprocPose)
{
    using namespace units::literals;
    base::VisionConfig config;
    config.name = "test-photon-covariance";
    int observations = 0;
    frc::AprilTagFieldLayout layout{
        {{1, frc::Pose3d{1_m, 2_m, 0_m, frc::Rotation3d{}}}, {2, frc::Pose3d{2_m, 2_m, 0_m, frc::Rotation3d{}}}},
        17_m,
        8_m};
    base::PhotonVision vision(config, layout, [&](const auto &) { ++observations; });
    auto publisher = nt::NetworkTableInstance::GetDefault()
                         .GetTable("photonvision")
                         ->GetSubTable(config.name)
                         ->GetRawTopic("rawBytes")
                         .Publish(photon::PhotonPipelineResult_TYPE_STRING);
    photon::PhotonPipelineResult result;
    result.metadata = photon::PhotonPipelineMetadata{1, 1000, 1000, 0};
    photon::PhotonTrackedTarget target{};
    target.fiducialId = 1;
    target.poseAmbiguity = .1;
    target.bestCameraToTarget = frc::Transform3d{2_m, 0_m, 0_m, frc::Rotation3d{}};
    result.targets = {target};
    target.fiducialId = 2;
    result.targets.push_back(target);
    photon::MultiTargetPNPResult multitag{photon::PnpResult{}, std::vector<int16_t>{1, 2}};
    multitag.estimatedPose.best = frc::Transform3d{1_m, 1_m, 0_m, frc::Rotation3d{}};
    result.multitagResult = multitag;
    auto publish = [&] {
        photon::Packet packet;
        packet.Pack(result);
        publisher.Set(packet.GetData());
        vision.Periodic();
    };
    publish();
    ASSERT_EQ(observations, 1);
    result.targets[0].bestCameraToTarget =
        frc::Transform3d{units::meter_t{std::numeric_limits<double>::quiet_NaN()}, 0_m, 0_m, frc::Rotation3d{}};
    ++result.metadata.sequenceID;
    ++result.metadata.captureTimestampMicros;
    publish();
    EXPECT_EQ(observations, 1);
}

TEST(VisionLimelight, ConfigurableTagGateAndCovariancePreserveMegaTag2Yaw)
{
    base::VisionConfig config;
    config.name = "test-limelight-tag-gate";
    config.megatag2 = true;
    config.allowedTagIds = {2};
    config.cameraStdDevFactor = 3;
    config.megatag2LinearFactor = .5;
    std::vector<base::VisionObservation> observations;
    base::Vision vision(config, [&](const auto &o) { observations.push_back(o); }, [] { return frc::Rotation2d{}; });
    auto pub = nt::NetworkTableInstance::GetDefault()
                   .GetTable(config.name)
                   ->GetDoubleArrayTopic("botpose_orb_wpiblue")
                   .Publish();
    auto pose = SingleTagPose(.9);
    pub.Set(pose);
    vision.Periodic();
    EXPECT_TRUE(observations.empty());
    pose[11] = 2;
    pub.Set(pose);
    vision.Periodic();
    ASSERT_EQ(observations.size(), 1u);
    EXPECT_NEAR(observations[0].stdDevs[0], .12, 1e-12);
    EXPECT_TRUE(std::isinf(observations[0].stdDevs[2]));
    config.cameraStdDevFactor = 0;
    EXPECT_THROW(base::Vision(config, {}, [] { return frc::Rotation2d{}; }), std::invalid_argument);
}
