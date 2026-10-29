#include "base/ObjectPerception.h"
#include <gtest/gtest.h>
#include <utility>

TEST(ObjectPerception, CaptureTimeMovingMountFusionAndExpiry) {
    using namespace units::literals;
    double extension=0;
    std::optional<base::ObjectDetectionFrame> front,rear;
    std::vector<base::ObjectCamera> cameras;
    cameras.push_back({"front",std::make_unique<base::InjectedObjectDetectionIO>([&]{return std::exchange(front,std::nullopt);}),
        [&]()->std::optional<frc::Transform3d>{return frc::Transform3d{units::meter_t{extension},0_m,0_m,frc::Rotation3d{}};}});
    cameras.push_back({"rear",std::make_unique<base::InjectedObjectDetectionIO>([&]{return std::exchange(rear,std::nullopt);}),
        []()->std::optional<frc::Transform3d>{return frc::Transform3d{0_m,0_m,0_m,frc::Rotation3d{0_deg,0_deg,180_deg}};}});
    base::ObjectPerceptionConfig config;config.clusterRadius=.15_m;
    base::ObjectPerception perception(std::move(cameras),[]{return frc::Pose3d{};},config);
    perception.UpdateAt(1_s,frc::Pose3d{});
    extension=.3;
    front=base::ObjectDetectionFrame{1,1.05_s,{{{2_m,0_m,0_m},1}}};
    rear=base::ObjectDetectionFrame{1,1.05_s,{{{1_m,0_m,0_m},1}}};
    perception.UpdateAt(1.1_s,frc::Pose3d{});
    ASSERT_EQ(perception.Clusters().size(),2u);
    EXPECT_NEAR(perception.Clusters()[0].position.X().value(),2.15,1e-9); // Mount at capture, not .3m current extension.
    EXPECT_NEAR(perception.Clusters()[1].position.X().value(),-1,1e-9);
    front=base::ObjectDetectionFrame{2,1.12_s,{}};
    perception.UpdateAt(1.12_s,frc::Pose3d{});
    EXPECT_EQ(perception.Clusters().size(),2u); // One missed frame must not erase a still-fresh observation.
    perception.UpdateAt(1.21_s,frc::Pose3d{});
    EXPECT_TRUE(perception.Clusters().empty());
    front=base::ObjectDetectionFrame{3,1.22_s,{{{2_m,0_m,0_m},1},{{2.1_m,0_m,0_m},1},{{2.35_m,0_m,0_m},1},{{5_m,0_m,0_m},.1}}};
    perception.UpdateAt(1.22_s,frc::Pose3d{});
    ASSERT_EQ(perception.Clusters().size(),2u); // Adjacent pieces stay separate.
    EXPECT_EQ(perception.Clusters()[0].observations,2u);
    EXPECT_NEAR(perception.Clusters()[0].position.X().value(),2.35,1e-9);
    auto invalid=config;invalid.freshness=2_s;
    EXPECT_THROW(perception.SetConfig(invalid),std::invalid_argument);
    EXPECT_EQ(perception.Clusters().size(),2u);
    config.maxDistance=1_m;
    perception.SetConfig(config);
    EXPECT_TRUE(perception.Clusters().empty());
    front=base::ObjectDetectionFrame{4,1.23_s,{{{2_m,0_m,0_m},1}}};
    perception.UpdateAt(1.23_s,frc::Pose3d{});
    EXPECT_TRUE(perception.Clusters().empty());
}

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <frc/Timer.h>
#include <limits>
TEST(ObjectPerception, NtTransportValidatesFramesAndProcessingLatency) {
    auto io=base::MakeNtObjectDetectionIO("/ObjectPerceptionTest/front/detections");
    auto pub=nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/ObjectPerceptionTest/front/detections").Publish();
    const auto before=frc::Timer::GetFPGATimestamp();
    pub.Set(std::array<double,7>{1,7,.04,2,.1,-.2,.9});
    const auto frame=io->ReadFrame();
    const auto after=frc::Timer::GetFPGATimestamp();
    ASSERT_TRUE(frame);
    ASSERT_EQ(frame->detections.size(),1u);
    EXPECT_EQ(frame->sequence,7u);
    EXPECT_DOUBLE_EQ(frame->detections[0].cameraPoint.X().value(),2);
    EXPECT_GE(frame->captureTime.value(),before.value()-.041);
    EXPECT_LE(frame->captureTime.value(),after.value()-.039);
    EXPECT_FALSE(io->ReadFrame());
    pub.Set(std::array<double,7>{1,8,0,2,0,0,std::numeric_limits<double>::quiet_NaN()});
    EXPECT_FALSE(io->ReadFrame());
}
