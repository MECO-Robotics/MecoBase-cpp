#include "base/Drive.h"
#include "base/Replay.h"
#include <gtest/gtest.h>
#include <frc/DataLogManager.h>
#include <frc/TimedRobot.h>
#include <wpi/DataLogWriter.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <map>
#include <thread>
#include <frc/simulation/DriverStationSim.h>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
namespace
{
void Environment(const char *key, const std::string &value)
{
#ifdef _WIN32
    _putenv_s(key, value.c_str());
#else
    if (value.empty())
        unsetenv(key);
    else
        setenv(key, value.c_str(), 1);
#endif
}
std::string SavedValue(const char *key)
{
    const auto *value = std::getenv(key);
    return value ? value : "";
}
} // namespace
TEST(DriveWheelRadiusCommand, OwnsDriveStopsOnInterruptionAndReplaysFeedbackFailure)
{
    if (std::getenv("FRC_RADIUS_CHILD"))
    {
        frc::TimedRobot notifier;
        base::replay::InitializeFromEnvironment();
        base::Drive drive;
        auto command = drive.CharacterizeWheelRadius();
        EXPECT_TRUE(command.get()->GetRequirements().contains(&drive));
        auto failing = drive.CharacterizeWheelRadius();
        base::replay::RunReplay([] {},
                                [&] {
                                    const auto frame = base::replay::FrameIndex();
                                    if (frame < 3)
                                    {
                                        if (frame == 0)
                                        {
                                            drive.RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{0},
                                                                                 units::meters_per_second_t{0},
                                                                                 units::radians_per_second_t{.2}});
                                            command.get()->Initialize();
                                            EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 0);
                                        }
                                        command.get()->Execute();
                                        if (frame == 1)
                                            EXPECT_GT(drive.RequestedSpeeds().omega.value(), 0);
                                        if (frame == 2)
                                        {
                                            command.get()->End(true);
                                            EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 0);
                                        }
                                    }
                                    else
                                    {
                                        if (frame == 3)
                                            failing.get()->Initialize();
                                        failing.get()->Execute();
                                        if (frame == 4)
                                        {
                                            EXPECT_TRUE(failing.get()->IsFinished());
                                            EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 0);
                                            failing.get()->End(false);
                                        }
                                    }
                                });
        EXPECT_FALSE(drive.SimulationTruth().has_value());
        base::replay::ShutdownRecording();
        base::replay::FlushOutput();
        frc::DataLogManager::Stop();
        return;
    }
    auto directory = std::filesystem::temp_directory_path() /
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::filesystem::create_directories(directory);
    auto source = directory / "source.wpilog", output = directory / "result.wpilog";
    wpi::json sticks = wpi::json::array();
    for (int i = 0; i < 6; ++i)
        sticks.push_back({{"axes", wpi::json::array()},
                          {"povs", wpi::json::array()},
                          {"buttons", 0},
                          {"buttonCount", 0},
                          {"xbox", false},
                          {"type", 0},
                          {"name", ""}});
    wpi::json frame = {{"schemaVersion", 1}, {"sequence", 0}, {"timestampUs", 100000}};
    frame["ds"] = {{"enabled", true},    {"autonomous", false}, {"test", false},
                   {"estop", false},     {"dsAttached", true},  {"fmsAttached", false},
                   {"alliance", "blue"}, {"location", 1},       {"joysticks", sticks}};
    frame["inputs"]["Drive/WheelRadius"] = {
        {"healthy", true}, {"timestamp", 1.0}, {"rawYaw", 0}, {"angles", {0, 0, 0, 0}}};
    frame["inputs"]["Drive"] = {{"pose", {0, 0, 0}}, {"speeds", {0, 0, 0}}, {"states", {0, 0, 0, 0, 0, 0, 0, 0}}};
    frame["outputs"] = wpi::json::object();
    {
        std::error_code error;
        wpi::log::DataLogWriter log(source.string(), error);
        ASSERT_FALSE(error);
        wpi::log::StringLogEntry entry(log, "Replay/Frames");
        for (int i = 0; i < 5; ++i)
        {
            if (i == 4)
                frame["inputs"]["Drive/WheelRadius"]["healthy"] = false;
            frame["sequence"] = i;
            frame["timestampUs"] = 100000 + i * 20000;
            entry.Append(frame.dump(), 100000 + i * 20000);
        }
        log.Flush();
    }
    auto oldSource = SavedValue("FRC_REPLAY_LOG"), oldOutput = SavedValue("FRC_REPLAY_OUTPUT");
    Environment("FRC_REPLAY_LOG", source.string());
    Environment("FRC_REPLAY_OUTPUT", output.string());
    Environment("FRC_RADIUS_CHILD", "1");
    auto result =
        std::system(("\"" + ::testing::internal::GetArgvs()[0] +
                     "\" --gtest_filter=DriveWheelRadiusCommand.OwnsDriveStopsOnInterruptionAndReplaysFeedbackFailure")
                        .c_str());
    Environment("FRC_REPLAY_LOG", oldSource);
    Environment("FRC_REPLAY_OUTPUT", oldOutput);
    Environment("FRC_RADIUS_CHILD", "");
    ASSERT_EQ(result, 0);
    auto buffer = wpi::MemoryBuffer::GetFile(output.string());
    ASSERT_TRUE(buffer);
    wpi::log::DataLogReader reader(std::move(*buffer));
    std::map<int, std::string> names;
    int results = 0;
    for (const auto &record : reader)
    {
        if (record.IsStart())
        {
            wpi::log::StartRecordData start;
            if (record.GetStartData(&start))
                names[start.entry] = std::string(start.name);
            continue;
        }
        if (names[record.GetEntry()] != "/Replay/Frames" && names[record.GetEntry()] != "Replay/Frames")
            continue;
        std::string_view text;
        if (!record.GetString(&text))
            continue;
        const auto replayed = wpi::json::parse(text);
        if (replayed["outputs"].contains("Drive/WheelRadiusResult"))
        {
            const auto value = replayed["outputs"]["Drive/WheelRadiusResult"];
            EXPECT_FALSE(value["valid"].get<bool>());
            EXPECT_EQ(value["interrupted"].get<bool>(), results == 0);
            ++results;
        }
    }
    EXPECT_EQ(results, 2);
    std::filesystem::remove_all(directory);
}
