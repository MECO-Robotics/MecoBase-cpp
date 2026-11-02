#pragma once
#include "base/Vision.h"
#include <optional>
#include <string_view>
#include <frc/geometry/Pose3d.h>
namespace base
{
struct QuestNavConfig
{
    std::string name = "QuestNav";
    frc::Transform3d robotToQuest;
    // Illustrative estimator uncertainties; measure/tune for the installed headset.
    std::array<double, 3> stdDevs{.1, .1, .05};
    double fieldLength = 17.548, fieldWidth = 8.052, maxZ = .75;
    double freshnessSeconds = .12;
    double subscriptionPeriodSeconds = .005;
};
// Native QuestNav 2026-2.2.0 frameData receiver. NT receives at 5 ms by default; Periodic
// drains the timestamped queue on the scheduler thread. No headset reset command.
std::unique_ptr<VisionIO> MakeQuestNavVisionIO(QuestNavConfig config);
class QuestNav : public VisionSystem
{
  public:
    static constexpr double kSubscriptionPeriod = .005;
    static constexpr std::string_view kFrameType = "proto:questnav.protos.data.ProtobufQuestNavFrameData";
    QuestNav(QuestNavConfig config, Consumer consumer);
    bool Connected() const
    {
        return Inputs().connected;
    }
    bool Tracking() const
    {
        return Inputs().connected && Inputs().tracking;
    }
    bool Aligned() const
    {
        return Tracking() && Inputs().aligned;
    }
    bool AlignToField(const frc::Pose2d &robotPose);
};
} // namespace base
