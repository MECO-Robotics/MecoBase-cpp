#include <networktables/RawTopic.h>
#include "base/QuestNav.h"
#include "QuestNavFixtures.h"
using namespace questfixture;
#include <gtest/gtest.h>
#include <frc/RobotController.h>
#include <frc/simulation/SimHooks.h>
#include <networktables/NetworkTableInstance.h>
#include <hal/HAL.h>
#include <vector>
#include <string_view>
using namespace units::literals;
namespace
{
struct Clock
{
    Clock()
    {
        HAL_Initialize(500, 0);
        frc::sim::PauseTiming();
        frc::sim::StepTimingAsync(.001_s);
    }
    ~Clock()
    {
        frc::sim::ResumeTiming();
    }
    int64_t Next()
    {
        frc::sim::StepTimingAsync(.005_s);
        return frc::RobotController::GetFPGATime();
    }
};
} // namespace
TEST(QuestNav, OfficialProtocolQueueMountAlignmentAndRestart)
{
    Clock clock;
    base::QuestNavConfig config;
    config.name = "test-questnav-mount";
    config.robotToQuest = {frc::Translation3d{1_m, 0_m, .5_m}, frc::Rotation3d{0_rad, 0_rad, 90_deg}};
    std::vector<base::VisionObservation> observations;
    base::QuestNav quest(config, [&](const auto &o) { observations.push_back(o); });
    auto pub = nt::NetworkTableInstance::GetDefault()
                   .GetTable(config.name)
                   ->GetRawTopic("frameData")
                   .Publish(base::QuestNav::kFrameType);
    EXPECT_FALSE(quest.AlignToField(frc::Pose2d{}));
    pub.Set(Bytes(first), clock.Next());
    quest.Periodic();
    ASSERT_TRUE(quest.Connected());
    ASSERT_TRUE(quest.Tracking());
    EXPECT_FALSE(quest.Aligned());
    EXPECT_TRUE(observations.empty());
    ASSERT_TRUE(quest.AlignToField(frc::Pose2d{5_m, 4_m, frc::Rotation2d{90_deg}}));
    // Four queued packets delivered before one 20 ms consumer tick. Unknown
    // protobuf fields must not discard otherwise valid tracking data.
    for (int i = 0; i < 4; ++i)
    {
        auto data = Bytes(second);
        data[10] = 8 + i;
        data.insert(data.end(), {0x98, 0x06, 0x01});
        pub.Set(data, clock.Next());
    }
    const auto last = frc::RobotController::GetFPGATime();
    quest.Periodic();
    ASSERT_EQ(observations.size(), 4u);
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(observations[i].pose.X().value(), 6, 1e-9);
        EXPECT_NEAR(observations[i].pose.Y().value(), 6, 1e-9);
        EXPECT_NEAR(std::abs(observations[i].pose.Rotation().Radians().value()), 3.141592653589793, 1e-9);
        EXPECT_NEAR(observations[i].timestamp.value(), (last - static_cast<int64_t>(3 - i) * 5000) * 1e-6, 1e-9);
    }
    // Headset restart cannot silently move the robot's field reference.
    pub.Set(Bytes(first), clock.Next());
    quest.Periodic();
    EXPECT_FALSE(quest.Aligned());
    EXPECT_EQ(observations.size(), 4u);
}
TEST(QuestNav, MalformedTrackingLossAndStaleDataDoNotFuse)
{
    Clock clock;
    base::QuestNavConfig config;
    config.name = "test-questnav-rejection";
    int observations = 0;
    base::QuestNav quest(config, [&](const auto &) { ++observations; });
    auto pub = nt::NetworkTableInstance::GetDefault()
                   .GetTable(config.name)
                   ->GetRawTopic("frameData")
                   .Publish(base::QuestNav::kFrameType);
    pub.Set(std::vector<uint8_t>{0x1a, 0xff}, clock.Next());
    quest.Periodic();
    EXPECT_FALSE(quest.Connected());
    pub.Set(Bytes(first), clock.Next());
    quest.Periodic();
    ASSERT_TRUE(quest.AlignToField(frc::Pose2d{5_m, 4_m, frc::Rotation2d{}}));
    auto lost = Bytes(second);
    lost[12] = 0;
    pub.Set(lost, clock.Next());
    quest.Periodic();
    EXPECT_TRUE(quest.Connected());
    EXPECT_FALSE(quest.Tracking());
    EXPECT_FALSE(quest.Aligned());
    EXPECT_EQ(observations, 0);
    auto next = Bytes(second);
    next[10] = 9;
    pub.Set(next, clock.Next());
    quest.Periodic();
    ASSERT_TRUE(quest.AlignToField(frc::Pose2d{5_m, 4_m, frc::Rotation2d{}}));
    // Regression: a fresh packet arriving in the first tick after a long gap
    // must not fuse against the old tracking-space anchor.
    frc::sim::StepTimingAsync(.121_s);
    next[10] = 10;
    pub.Set(next, clock.Next());
    quest.Periodic();
    EXPECT_TRUE(quest.Connected());
    EXPECT_FALSE(quest.Aligned());
    EXPECT_EQ(observations, 0);
    frc::sim::StepTimingAsync(.121_s);
    quest.Periodic();
    EXPECT_FALSE(quest.Connected());
    EXPECT_FALSE(quest.Aligned());
    EXPECT_FALSE(quest.AlignToField(frc::Pose2d{}));
    EXPECT_EQ(observations, 0);
}
