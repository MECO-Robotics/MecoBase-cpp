#include "base/AdvantageKitReplayAdapter.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <gtest/gtest.h>
#include <wpi/DataLogWriter.h>
#include <wpi/struct/Struct.h>

namespace
{
std::filesystem::path TempLog(const char *suffix)
{
    return std::filesystem::temp_directory_path() /
           ("akit-replay-adapter-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
            suffix);
}

void AddBool(wpi::log::DataLogWriter &log, const std::string &name, bool value, int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
        it->second = log.Start(name, "boolean", "", timestamp);
    log.AppendBoolean(it->second, value, timestamp);
}

void AddInt(wpi::log::DataLogWriter &log, const std::string &name, int64_t value, int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
        it->second = log.Start(name, "int64", "", timestamp);
    log.AppendInteger(it->second, value, timestamp);
}

void AddString(wpi::log::DataLogWriter &log, const std::string &name, const std::string &value, int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
        it->second = log.Start(name, "string", "", timestamp);
    log.AppendString(it->second, value, timestamp);
}

void AddIntArray(wpi::log::DataLogWriter &log, const std::string &name, const std::vector<int64_t> &value,
                 int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
        it->second = log.Start(name, "int64[]", "", timestamp);
    log.AppendIntegerArray(it->second, value, timestamp);
}

void AddFloatArray(wpi::log::DataLogWriter &log, const std::string &name, const std::vector<float> &value,
                   int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
        it->second = log.Start(name, "float[]", "", timestamp);
    log.AppendFloatArray(it->second, value, timestamp);
}

template <typename T>
void AddStruct(wpi::log::DataLogWriter &log, const std::string &name, const T &value, int64_t timestamp)
{
    static std::map<std::string, int> ids;
    auto [it, inserted] = ids.try_emplace(name, 0);
    if (inserted)
    {
        log.AddStructSchema<T>(timestamp);
        it->second = log.Start(name, wpi::GetStructTypeString<T>(), "", timestamp);
    }
    std::array<uint8_t, wpi::GetStructSize<T>()> bytes{};
    wpi::PackStruct(std::span<uint8_t>{bytes}, value);
    log.AppendRaw(it->second, bytes, timestamp);
}

void WriteFrame(wpi::log::DataLogWriter &log, int timestampEntry, int64_t timestamp, bool chooser, bool blueAlliance,
                double poseX)
{
    log.AppendInteger(timestampEntry, timestamp, timestamp);
    AddBool(log, "/DriverStation/Enabled", true, timestamp);
    AddBool(log, "/DriverStation/Autonomous", true, timestamp);
    AddBool(log, "/DriverStation/Test", false, timestamp);
    AddBool(log, "/DriverStation/EmergencyStop", false, timestamp);
    AddBool(log, "/DriverStation/DSAttached", true, timestamp);
    AddBool(log, "/DriverStation/FMSAttached", false, timestamp);
    AddInt(log, "/DriverStation/AllianceStation", blueAlliance ? 4 : 2, timestamp);
    for (int port = 0; port < 6; ++port)
    {
        const std::string path = "/DriverStation/Joystick" + std::to_string(port) + "/";
        AddString(log, path + "Name", port == 0 ? "Xbox Controller" : "", timestamp);
        AddInt(log, path + "Type", port == 0 ? 1 : 0, timestamp);
        AddBool(log, path + "Xbox", port == 0, timestamp);
        AddInt(log, path + "ButtonCount", port == 0 ? 10 : 0, timestamp);
        AddInt(log, path + "ButtonValues", port == 0 ? 5 : 0, timestamp);
        AddIntArray(log, path + "POVs", port == 0 ? std::vector<int64_t>{90, -1} : std::vector<int64_t>{}, timestamp);
        AddFloatArray(log, path + "AxisValues", port == 0 ? std::vector<float>{0.25F, -0.5F} : std::vector<float>{},
                      timestamp);
    }

    const std::string autoKey = "/NetworkInputs/SmartDashboard/Auto Choices";
    if (chooser)
        AddString(log, autoKey, "Center Auto", timestamp);

    const frc::Pose2d pose{units::meter_t{poseX}, units::meter_t{-2.0}, frc::Rotation2d{units::radian_t{0.4}}};
    const frc::ChassisSpeeds speeds{units::meters_per_second_t{1.0}, units::meters_per_second_t{-0.25},
                                    units::radians_per_second_t{0.5}};
    const std::array<frc::SwerveModuleState, 4> states{
        frc::SwerveModuleState{units::meters_per_second_t{1.0}, frc::Rotation2d{units::radian_t{0.0}}},
        frc::SwerveModuleState{units::meters_per_second_t{2.0}, frc::Rotation2d{units::radian_t{0.5}}},
        frc::SwerveModuleState{units::meters_per_second_t{3.0}, frc::Rotation2d{units::radian_t{-0.5}}},
        frc::SwerveModuleState{units::meters_per_second_t{4.0}, frc::Rotation2d{units::radian_t{1.0}}}};

    AddStruct(log, "/DriveState/Pose", pose, timestamp);
    AddStruct(log, "/DriveState/Speeds", speeds, timestamp);

    using Module = frc::SwerveModuleState;
    static int moduleStatesEntry = 0;
    if (moduleStatesEntry == 0)
    {
        log.AddStructSchema<Module>(timestamp);
        moduleStatesEntry = log.Start("/DriveState/ModuleStates",
                                      wpi::MakeStructArrayTypeString<Module, std::dynamic_extent>(), "", timestamp);
    }
    std::array<uint8_t, 4 * wpi::GetStructSize<Module>()> statesBytes{};
    for (size_t i = 0; i < states.size(); ++i)
    {
        wpi::PackStruct(std::span<uint8_t>{statesBytes}.subspan(i * wpi::GetStructSize<Module>()), states[i]);
    }
    log.AppendRaw(moduleStatesEntry, statesBytes, timestamp);
}

TEST(AdvantageKitReplayAdapter, MapsJava26TypedDriveAndDriverStationFrames)
{
    const auto path = TempLog("-valid.wpilog");
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(path.string(), ec, "AdvantageKit");
        ASSERT_FALSE(ec);
        const int timestamp = log.Start("/Timestamp", "int64");
        WriteFrame(log, timestamp, 100'000, false, true, 1.25);
        WriteFrame(log, timestamp, 120'000, true, false, 2.5);
        log.Flush();
    }

    const auto frames = base::akit::ReadLog(path.string());
    const auto native = base::akit::ConvertJavaAdvantageKitFrames(frames);
    ASSERT_EQ(native.size(), 2u);
    EXPECT_EQ(native[0]["schemaVersion"], 1);
    EXPECT_EQ(native[0]["sequence"], 0);
    EXPECT_EQ(native[0]["timestampUs"], 100'000);
    ASSERT_EQ(native[0]["ds"]["joysticks"].size(), 6u);
    EXPECT_EQ(native[0]["ds"]["alliance"], "blue");
    EXPECT_EQ(native[0]["ds"]["location"], 1);
    EXPECT_TRUE(native[0]["ds"]["enabled"]);
    EXPECT_TRUE(native[0]["ds"]["autonomous"]);
    EXPECT_EQ(native[0]["ds"]["joysticks"][0]["axes"], (wpi::json{0.25, -0.5}));
    EXPECT_EQ(native[0]["ds"]["joysticks"][0]["povs"], (wpi::json{90, -1}));
    EXPECT_EQ(native[0]["ds"]["joysticks"][0]["buttons"], 5u);
    EXPECT_EQ(native[0]["ds"]["joysticks"][0]["name"], "Xbox Controller");
    EXPECT_EQ(native[0]["inputs"]["Auto/Selection"], "None");
    EXPECT_EQ(native[0]["inputs"]["DriverStation/MatchInfo"]["enabled"], true);
    EXPECT_EQ(native[0]["inputs"]["DriverStation/MatchInfo"]["autonomous"], true);
    EXPECT_EQ(native[0]["inputs"]["DriverStation/MatchInfo"]["teleop"], false);
    EXPECT_EQ(native[0]["inputs"]["DriverStation/MatchInfo"]["test"], false);
    EXPECT_EQ(native[0]["inputs"]["DriverStation/MatchInfo"]["alliance"], "blue");
    EXPECT_TRUE(native[0]["inputs"]["DriverStation/MatchInfo"]["remainingTimeSeconds"].is_null());
    EXPECT_TRUE(native[0]["inputs"]["DriverStation/MatchInfo"]["gameSpecificMessage"].is_null());
    EXPECT_EQ(native[0]["inputs"]["Drive"]["backend"], "phoenix");
    EXPECT_EQ(native[0]["inputs"]["Drive"]["pose"], (wpi::json{1.25, -2.0, 0.4}));
    EXPECT_EQ(native[0]["inputs"]["Drive"]["speeds"], (wpi::json{1.0, -0.25, 0.5}));
    EXPECT_EQ(native[0]["inputs"]["Drive"]["states"], (wpi::json{1.0, 0.0, 2.0, 0.5, 3.0, -0.5, 4.0, 1.0}));
    EXPECT_EQ(native[0]["outputs"], wpi::json::object());

    EXPECT_EQ(native[1]["sequence"], 1);
    EXPECT_EQ(native[1]["timestampUs"], 120'000);
    EXPECT_EQ(native[1]["ds"]["alliance"], "red");
    EXPECT_EQ(native[1]["inputs"]["Auto/Selection"], "Center Auto");
    EXPECT_EQ(native[1]["inputs"]["Drive"]["pose"][0], 2.5);

    auto malformed = frames.front();
    const auto poseSlot = malformed.keyIndex->at("DriveState/Pose");
    auto damagedPose = std::make_shared<base::akit::Record>(*malformed.records.at(poseSlot));
    std::get<std::vector<uint8_t>>(damagedPose->value).pop_back();
    malformed.records[poseSlot] = std::move(damagedPose);
    try
    {
        (void)base::akit::ConvertJavaAdvantageKitFrames({malformed});
        FAIL() << "malformed struct unexpectedly converted";
    }
    catch (const std::runtime_error &error)
    {
        EXPECT_NE(std::string(error.what()).find("DriveState/Pose"), std::string::npos);
        EXPECT_NE(std::string(error.what()).find("expected 24"), std::string::npos);
    }
    std::filesystem::remove(path);
}

TEST(AdvantageKitReplayAdapter, RejectsIncompleteFramesInsteadOfUsingLiveInputs)
{
    base::akit::Frame frame;
    frame.timestampUs = 10;
    try
    {
        (void)base::akit::ConvertJavaAdvantageKitFrames({frame});
        FAIL() << "incomplete frame unexpectedly converted";
    }
    catch (const std::runtime_error &error)
    {
        EXPECT_NE(std::string(error.what()).find("DriveState/Pose"), std::string::npos);
    }
}
} // namespace
