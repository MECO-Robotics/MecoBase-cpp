#include "base/QuestNav.h"
#include <networktables/RawTopic.h>
#include "base/Replay.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pb_decode.h>
#include <frc/RobotController.h>
#include <frc/geometry/proto/Pose3dProto.h>
#include <networktables/NetworkTableInstance.h>
namespace base
{
class QuestNavIO final : public VisionIO
{
  public:
    explicit QuestNavIO(QuestNavConfig config);
    void UpdateInputs(VisionInputs &) override;
    void SetQuestFilter(const QuestFilterTuning &) override;
    bool AlignToField(const frc::Pose2d &) override;

  private:
    bool Tracking() const
    {
        return connected_ && tracking_;
    }
    QuestNavConfig config_;
    nt::RawSubscriber frames_;
    std::optional<frc::Pose3d> rawRobot_;
    frc::Pose3d rawAnchor_, fieldAnchor_;
    int64_t lastReceive_{}, lastTimestamp_{};
    double lastAppTimestamp_{};
    uint64_t lastFrame_{};
    bool haveFrame_{}, connected_{}, tracking_{}, aligned_{};
};
namespace
{
struct Frame
{
    uint64_t count{};
    double uptime{};
    bool tracking{};
    frc::Pose3d pose;
};
bool Finite(const frc::Pose3d &pose)
{
    const auto q = pose.Rotation().GetQuaternion();
    for (double v : {pose.X().value(), pose.Y().value(), pose.Z().value(), q.W(), q.X(), q.Y(), q.Z()})
        if (!std::isfinite(v))
            return false;
    return true;
}
// Outer schema: QuestNav 2026-2.2.0 Data.ProtobufQuestNavFrameData.
// Decode standard protobuf primitives with WPILib's nanopb, and delegate the
// nested wpi.proto.ProtobufPose3d to WPILib. Unknown fields remain compatible.
std::optional<Frame> Decode(std::span<const uint8_t> data)
{
    if (data.empty() || data.size() > 4096)
        return std::nullopt;
    auto stream = pb_istream_from_buffer(data.data(), data.size());
    Frame frame;
    bool havePose = false;
    while (stream.bytes_left)
    {
        pb_wire_type_t wire;
        uint32_t tag;
        bool eof;
        if (!pb_decode_tag(&stream, &wire, &tag, &eof))
            return std::nullopt;
        if (tag == 1 || tag == 4)
        {
            uint64_t value;
            if (wire != PB_WT_VARINT || !pb_decode_varint(&stream, &value))
                return std::nullopt;
            if (tag == 1)
            {
                if (value > INT32_MAX)
                    return std::nullopt;
                frame.count = value;
            }
            else
            {
                if (value > 1)
                    return std::nullopt;
                frame.tracking = value != 0;
            }
        }
        else if (tag == 2)
        {
            if (wire != PB_WT_64BIT || !pb_decode_fixed64(&stream, &frame.uptime))
                return std::nullopt;
        }
        else if (tag == 3)
        {
            pb_istream_t sub;
            if (wire != PB_WT_STRING || !pb_make_string_substream(&stream, &sub))
                return std::nullopt;
            try
            {
                wpi::ProtoInputStream<frc::Pose3d> poseStream{&sub};
                auto pose = wpi::Protobuf<frc::Pose3d>::Unpack(poseStream);
                if (!pose || !Finite(*pose))
                    return std::nullopt;
                frame.pose = *pose;
                havePose = true;
            }
            catch (const std::exception &)
            {
                return std::nullopt;
            }
            if (!pb_close_string_substream(&stream, &sub))
                return std::nullopt;
        }
        else if (!pb_skip_field(&stream, wire))
            return std::nullopt;
    }
    if (!havePose || !std::isfinite(frame.uptime) || frame.uptime < 0)
        return std::nullopt;
    return frame;
}
wpi::json Pose(const frc::Pose3d &p)
{
    auto q = p.Rotation().GetQuaternion();
    return {p.X().value(), p.Y().value(), p.Z().value(), q.W(), q.X(), q.Y(), q.Z()};
}

} // namespace
QuestNavIO::QuestNavIO(QuestNavConfig config) : config_(std::move(config))
{
    for (double v : {config_.fieldLength, config_.fieldWidth, config_.maxZ, config_.freshnessSeconds,
                     config_.stdDevs[0], config_.stdDevs[1], config_.stdDevs[2]})
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Invalid QuestNav configuration");
    if (config_.name.empty() || !Finite(frc::Pose3d{}.TransformBy(config_.robotToQuest)))
        throw std::invalid_argument("Invalid QuestNav mount/name");
    if (replay::Active())
        return;
    nt::PubSubOptions options;
    options.periodic = QuestNav::kSubscriptionPeriod;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.pollStorage = 64;
    frames_ = nt::NetworkTableInstance::GetDefault()
                  .GetTable(config_.name)
                  ->GetRawTopic("frameData")
                  .Subscribe(QuestNav::kFrameType, {}, options);
}
void QuestNavIO::SetQuestFilter(const QuestFilterTuning &tuning)
{
    for (double v : {tuning.maxZ, tuning.freshnessSeconds, tuning.stdDevs[0], tuning.stdDevs[1], tuning.stdDevs[2]})
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Invalid QuestNav runtime filter");
    if (tuning.freshnessSeconds > static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e6)
        throw std::invalid_argument("QuestNav freshness overflows timestamp range");
    config_.maxZ = tuning.maxZ;
    config_.freshnessSeconds = tuning.freshnessSeconds;
    config_.stdDevs = tuning.stdDevs;
}
void QuestNavIO::UpdateInputs(VisionInputs &inputs)
{
    const auto input = replay::ProcessInputs("QuestNav/" + config_.name + "/Frames", [this] {
        auto queue = wpi::json::array();
        for (const auto &sample : frames_.ReadQueue())
        {
            queue.push_back({{"timestampUs", sample.time}, {"data", sample.value}});
        }
        return wpi::json{{"nowUs", frc::RobotController::GetFPGATime()}, {"frames", queue}};
    });
    const auto now = input.at("nowUs").get<int64_t>();
    const auto fresh = static_cast<int64_t>(config_.freshnessSeconds * 1e6);
    // A new packet after a receive gap cannot revive an obsolete field alignment.
    if (haveFrame_ && (now < lastReceive_ || now - lastReceive_ >= fresh))
    {
        connected_ = false;
        tracking_ = false;
        aligned_ = false;
    }
    for (const auto &sample : input.at("frames"))
    {
        const auto time = sample.at("timestampUs").get<int64_t>();
        const auto bytes = sample.at("data").get<std::vector<uint8_t>>();
        const auto decoded = Decode(bytes);
        if (!decoded)
            continue;
        const auto count = decoded->count;
        const auto uptime = decoded->uptime;
        const auto &pose = decoded->pose;
        if (time <= 0 || time <= lastTimestamp_ || time > now || now - time >= fresh || !Finite(pose) ||
            !std::isfinite(uptime) || uptime < 0)
            continue;
        if (haveFrame_ && (count < lastFrame_ || uptime < lastAppTimestamp_))
            aligned_ = false;
        if (haveFrame_ && count == lastFrame_ && uptime == lastAppTimestamp_)
            continue;
        haveFrame_ = true;
        lastTimestamp_ = time;
        lastReceive_ = time;
        lastFrame_ = count;
        lastAppTimestamp_ = uptime;
        tracking_ = decoded->tracking;
        rawRobot_ = pose.TransformBy(config_.robotToQuest.Inverse());
        if (!tracking_)
        {
            aligned_ = false;
            continue;
        }
        if (!aligned_)
            continue;
        auto field = fieldAnchor_.TransformBy(frc::Transform3d{rawAnchor_, *rawRobot_});
        if (!Finite(field) || field.X().value() < 0 || field.X().value() > config_.fieldLength ||
            field.Y().value() < 0 || field.Y().value() > config_.fieldWidth ||
            std::abs(field.Z().value()) > config_.maxZ)
            continue;
        inputs.observations.push_back({field.ToPose2d(), units::second_t{time * 1e-6}, config_.stdDevs});
    }
    connected_ = haveFrame_ && now >= lastReceive_ && now - lastReceive_ < fresh;
    if (!connected_)
    {
        tracking_ = false;
        aligned_ = false;
    }
    inputs.connected = connected_;
    inputs.tracking = tracking_;
    inputs.aligned = aligned_;
}
bool QuestNavIO::AlignToField(const frc::Pose2d &robotPose)
{
    const frc::Pose3d desired{robotPose};
    if (!Finite(desired))
        throw std::invalid_argument("Nonfinite QuestNav field alignment");
    const bool accepted = Tracking() && rawRobot_.has_value();
    replay::RecordOutput("QuestNav/" + config_.name + "/Alignment", {{"pose", Pose(desired)}, {"accepted", accepted}});
    if (!accepted)
        return false;
    rawAnchor_ = *rawRobot_;
    fieldAnchor_ = desired;
    aligned_ = true;
    return true;
}
} // namespace base

namespace base
{
std::unique_ptr<VisionIO> MakeQuestNavVisionIO(QuestNavConfig config)
{
    return std::make_unique<QuestNavIO>(std::move(config));
}
QuestNav::QuestNav(QuestNavConfig config, Consumer consumer)
    : VisionSystem(MakeQuestNavVisionIO(config), std::move(consumer))
{
    SetName(config.name);
}
bool QuestNav::AlignToField(const frc::Pose2d &pose)
{
    return VisionSystem::AlignToField(0, pose);
}
} // namespace base
