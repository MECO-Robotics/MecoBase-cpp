#include "base/AdvantageKitReplayAdapter.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <wpi/struct/Struct.h>

namespace base::akit
{
namespace
{
using Json = wpi::json;

const Record &Require(const Frame &frame, const std::string &key)
{
    if (const auto *record = frame.Find(key))
        return *record;
    throw std::runtime_error("AdvantageKit frame is missing required field: " + key);
}

template <typename T> const T &RequireValue(const Frame &frame, const std::string &key, const std::string &expectedType)
{
    const auto &record = Require(frame, key);
    if (record.type != expectedType)
    {
        throw std::runtime_error("AdvantageKit field " + key + " has schema " + record.type + "; expected " +
                                 expectedType);
    }
    const auto *value = std::get_if<T>(&record.value);
    if (!value)
    {
        throw std::runtime_error("AdvantageKit field " + key + " has an invalid decoded value for schema " +
                                 expectedType);
    }
    return *value;
}

bool Boolean(const Frame &frame, const std::string &key)
{
    return RequireValue<bool>(frame, key, "boolean");
}

int64_t Integer(const Frame &frame, const std::string &key)
{
    return RequireValue<int64_t>(frame, key, "int64");
}

int CheckedInt(int64_t value, const std::string &key)
{
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
    {
        throw std::runtime_error("AdvantageKit field " + key + " is outside int range");
    }
    return static_cast<int>(value);
}

Json DecodeDS(const Frame &frame)
{
    const int64_t station = Integer(frame, "DriverStation/AllianceStation");
    std::string alliance = "unknown";
    int location = 1;
    if (station >= 1 && station <= 3)
    {
        alliance = "red";
        location = static_cast<int>(station);
    }
    else if (station >= 4 && station <= 6)
    {
        alliance = "blue";
        location = static_cast<int>(station - 3);
    }
    else if (station != 0)
    {
        throw std::runtime_error("AdvantageKit DriverStation/AllianceStation is outside 0..6");
    }

    Json sticks = Json::array();
    for (int stick = 0; stick < 6; ++stick)
    {
        const std::string prefix = "DriverStation/Joystick" + std::to_string(stick) + "/";
        const std::string axesKey = prefix + "AxisValues";
        const std::string povsKey = prefix + "POVs";
        const std::string nameKey = prefix + "Name";
        const auto &axisValues = RequireValue<std::vector<float>>(frame, axesKey, "float[]");
        const auto &povValues = RequireValue<std::vector<int64_t>>(frame, povsKey, "int64[]");
        if (axisValues.size() > 12 || povValues.size() > 12)
        {
            throw std::runtime_error("AdvantageKit joystick dimensions exceed native replay limits at " + prefix);
        }
        Json axes = Json::array();
        for (float axis : axisValues)
        {
            if (!std::isfinite(axis))
                throw std::runtime_error("AdvantageKit joystick axis is nonfinite: " + prefix + "AxisValues");
            axes.push_back(static_cast<double>(axis));
        }
        Json povs = Json::array();
        for (int64_t pov : povValues)
        {
            povs.push_back(CheckedInt(pov, prefix + "POVs"));
        }

        const int64_t buttons = Integer(frame, prefix + "ButtonValues");
        if (buttons < std::numeric_limits<int32_t>::min() || buttons > std::numeric_limits<int32_t>::max())
        {
            throw std::runtime_error("AdvantageKit joystick button mask is outside Java int range: " + prefix +
                                     "ButtonValues");
        }
        const int64_t buttonCount = Integer(frame, prefix + "ButtonCount");
        const int64_t joystickType = Integer(frame, prefix + "Type");
        const bool xbox = Boolean(frame, prefix + "Xbox");
        const auto &name = RequireValue<std::string>(frame, nameKey, "string");

        sticks.push_back({{"axes", std::move(axes)},
                          {"povs", std::move(povs)},
                          {"buttons", static_cast<uint32_t>(buttons)},
                          {"buttonCount", CheckedInt(buttonCount, prefix + "ButtonCount")},
                          {"xbox", xbox},
                          {"type", CheckedInt(joystickType, prefix + "Type")},
                          {"name", name}});
    }

    return {{"enabled", Boolean(frame, "DriverStation/Enabled")},
            {"autonomous", Boolean(frame, "DriverStation/Autonomous")},
            {"test", Boolean(frame, "DriverStation/Test")},
            {"estop", Boolean(frame, "DriverStation/EmergencyStop")},
            {"dsAttached", Boolean(frame, "DriverStation/DSAttached")},
            {"fmsAttached", Boolean(frame, "DriverStation/FMSAttached")},
            {"alliance", alliance},
            {"location", location},
            {"joysticks", std::move(sticks)}};
}

template <typename T>
std::vector<uint8_t> RawStruct(const Frame &frame, const std::string &key, const std::string &expectedType,
                               size_t expectedSize)
{
    const auto &bytes = RequireValue<std::vector<uint8_t>>(frame, key, expectedType);
    if (bytes.size() != expectedSize)
    {
        throw std::runtime_error("AdvantageKit field " + key + " has " + std::to_string(bytes.size()) +
                                 " bytes; expected " + std::to_string(expectedSize) + " for " + expectedType);
    }
    return bytes;
}

Json DecodeDrive(const Frame &frame, const std::string &backend)
{
    const auto poseBytes =
        RawStruct<frc::Pose2d>(frame, "DriveState/Pose", "struct:Pose2d", wpi::GetStructSize<frc::Pose2d>());
    const auto speedsBytes = RawStruct<frc::ChassisSpeeds>(frame, "DriveState/Speeds", "struct:ChassisSpeeds",
                                                           wpi::GetStructSize<frc::ChassisSpeeds>());
    constexpr size_t kModuleCount = 4;
    constexpr size_t kModuleBytes = wpi::GetStructSize<frc::SwerveModuleState>();
    const auto statesBytes = RawStruct<std::array<frc::SwerveModuleState, kModuleCount>>(
        frame, "DriveState/ModuleStates", "struct:SwerveModuleState[]", kModuleCount * kModuleBytes);

    const auto pose = wpi::UnpackStruct<frc::Pose2d>(poseBytes);
    const auto speeds = wpi::UnpackStruct<frc::ChassisSpeeds>(speedsBytes);
    const auto states = wpi::UnpackStructArray<frc::SwerveModuleState, 0, kModuleCount>(statesBytes);
    Json poseJson = {pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()};
    Json speedsJson = {speeds.vx.value(), speeds.vy.value(), speeds.omega.value()};
    Json stateJson = Json::array();
    for (const auto &state : states)
    {
        stateJson.push_back(state.speed.value());
        stateJson.push_back(state.angle.Radians().value());
    }
    return {{"backend", backend},
            {"pose", std::move(poseJson)},
            {"speeds", std::move(speedsJson)},
            {"states", std::move(stateJson)}};
}

std::string AutoSelection(const Frame &frame)
{
    if (const auto *found = frame.Find("NetworkInputs/SmartDashboard/Auto Choices"))
    {
        const auto &record = *found;
        if (record.type != "string")
            throw std::runtime_error("AdvantageKit field NetworkInputs/SmartDashboard/Auto Choices has schema " +
                                     record.type + "; expected string");
        const auto *selection = std::get_if<std::string>(&record.value);
        if (!selection)
            throw std::runtime_error(
                "AdvantageKit field NetworkInputs/SmartDashboard/Auto Choices is not a decoded string");
        // The chooser can be null before the first dashboard selection. Treat only
        // that documented absence as the base chooser's explicit default.
        return selection->empty() ? "None" : *selection;
    }
    return "None";
}
} // namespace

std::vector<wpi::json> ConvertJavaAdvantageKitFrames(const std::vector<Frame> &frames, std::string driveBackend)
{
    if (frames.empty())
        throw std::runtime_error("AdvantageKit adapter received no frames");
    if (driveBackend != "phoenix" && driveBackend != "rev")
        throw std::runtime_error("AdvantageKit adapter drive backend must be 'phoenix' or 'rev'");

    std::vector<Json> converted;
    converted.reserve(frames.size());
    int64_t previousTimestamp = -1;
    for (size_t sequence = 0; sequence < frames.size(); ++sequence)
    {
        if (frames[sequence].timestampUs <= previousTimestamp)
            throw std::runtime_error("AdvantageKit frame timestamps must be nonnegative and strictly increasing");
        previousTimestamp = frames[sequence].timestampUs;
        converted.push_back(ConvertJavaAdvantageKitFrame(frames[sequence], sequence, driveBackend));
    }
    return converted;
}

wpi::json ConvertJavaAdvantageKitFrame(const Frame &frame, size_t sequence, const std::string &driveBackend)
{
    if (frame.timestampUs < 0)
        throw std::runtime_error("AdvantageKit frame timestamps must be nonnegative and strictly increasing");
    if (driveBackend != "phoenix" && driveBackend != "rev")
        throw std::runtime_error("AdvantageKit adapter drive backend must be 'phoenix' or 'rev'");
    Json inputs;
    inputs["Drive"] = DecodeDrive(frame, driveBackend);
    inputs["Auto/Selection"] = AutoSelection(frame);
    const auto ds = DecodeDS(frame);
    const bool autonomous = ds.at("autonomous").get<bool>();
    const bool test = ds.at("test").get<bool>();
    inputs["DriverStation/MatchInfo"] = {
        {"enabled", ds.at("enabled")},
        {"autonomous", autonomous},
        {"teleop", !autonomous && !test},
        {"test", test},
        {"alliance", ds.at("alliance")},
        // Java AdvantageKit's supported DS schema does not carry match time or
        // game-specific data. Keep those fields explicitly unavailable.
        {"remainingTimeSeconds", nullptr},
        {"gameSpecificMessage", nullptr}};
    return {{"schemaVersion", 1},    {"sequence", sequence},        {"timestampUs", frame.timestampUs},
            {"ds", ds}, {"inputs", std::move(inputs)}, {"outputs", Json::object()}};
}
} // namespace base::akit
