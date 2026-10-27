#include "base/MechanismTuning.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLogWriter.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <map>
namespace
{
void TuningEnv(const char *key, const std::string &value)
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
std::string TuningSaved(const char *key)
{
    auto value = std::getenv(key);
    return value ? value : "";
}
} // namespace
TEST(MechanismTuningReplay, ReexecutesRawTransactionsAgainstRecordedEnableState)
{
    if (std::getenv("FRC_TUNING_REPLAY_CHILD"))
    {
        base::replay::InitializeFromEnvironment();
        auto io = std::make_unique<base::InjectedMotorIO>();
        base::PositionJoint joint("ReplayTune", std::move(io), {});
        base::MechanismTuning tuning(joint);
        base::Flywheel retryWheel("RetryProbe", std::make_unique<base::InjectedMotorIO>(), {});
        base::replay::RunReplay([] {},
                                [&] {
                                    joint.Periodic();
                                    tuning.Periodic();
                                    retryWheel.Periodic();
                                    EXPECT_THROW(retryWheel.SetGains({}), std::logic_error);
                                    EXPECT_THROW(retryWheel.SetBrake(true), std::logic_error);
                                    if (base::replay::FrameIndex() == 1)
                                    {
                                        EXPECT_THROW(retryWheel.SetBrake(false), std::logic_error);
                                    }
                                    else
                                    {
                                        retryWheel.SetBrake(false);
                                        EXPECT_THROW(retryWheel.SetBrake(false), std::logic_error);
                                        EXPECT_THROW(retryWheel.SetGains({}), std::logic_error);
                                    }
                                    EXPECT_DOUBLE_EQ(joint.GetGains().p, 5);
                                    EXPECT_EQ(tuning.LastStatus(),
                                              base::replay::FrameIndex() == 0 ? "accepted"
                                              : base::replay::FrameIndex() == 1
                                                  ? "rejected: gains require disabled and unowned mechanism"
                                                  : "applied");
                                    if (base::replay::FrameIndex() == 2)
                                        EXPECT_EQ(tuning.LastRevision(), 2u);
                                });
        base::telemetry::Shutdown();
        base::replay::ShutdownRecording();
        base::replay::FlushOutput();
        frc::DataLogManager::Stop();
        return;
    }
    auto dir = std::filesystem::temp_directory_path() /
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::filesystem::create_directories(dir);
    auto input = dir / "input.wpilog", output = dir / "output.wpilog";
    wpi::json sticks = wpi::json::array();
    for (int i = 0; i < 6; ++i)
        sticks.push_back({{"axes", wpi::json::array()},
                          {"povs", wpi::json::array()},
                          {"buttons", 0},
                          {"buttonCount", 0},
                          {"xbox", false},
                          {"type", 0},
                          {"name", ""}});
    wpi::json gain = {{"p", 5},
                      {"i", 0},
                      {"d", 0},
                      {"s", 0},
                      {"v", 0},
                      {"a", 0},
                      {"gravity", 0},
                      {"maxVelocity", 1},
                      {"maxAcceleration", 1},
                      {"tolerance", .01},
                      {"minimum", 0},
                      {"maximum", 1},
                      {"defaultPosition", 0}};
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(input.string(), ec);
        ASSERT_FALSE(ec);
        wpi::log::StringLogEntry entry(log, "Replay/Frames");
        for (int i = 0; i < 3; ++i)
        {
            gain["p"] = i == 0 ? 5 : 9;
            wpi::json request = {{"revision", i == 0 ? 1 : 2}, {"gains", gain}};
            wpi::json frame = {
                {"schemaVersion", 1},
                {"sequence", i},
                {"timestampUs", 100000 + i * 20000},
                {"ds",
                 {{"enabled", i == 1},
                  {"autonomous", false},
                  {"test", false},
                  {"estop", false},
                  {"dsAttached", true},
                  {"fmsAttached", false},
                  {"alliance", "blue"},
                  {"location", 1},
                  {"joysticks", sticks}}},
                {"inputs", {{"/Tuning/ReplayTune/Requests", {{"requests", wpi::json::array({request.dump()})}}}}},
                {"outputs", {{"/Tuning/ReplayTune/Status", {{"status", "wrong original output"}}}}}};
            frame["inputs"]["Mechanism/RetryProbe"] = {{"position", 0},
                                                       {"velocity", 0},
                                                       {"voltage", 0},
                                                       {"supplyVoltage", 12},
                                                       {"current", 0},
                                                       {"connected", false},
                                                       {"externalEncoder", nullptr},
                                                       {"nativeControl", true},
                                                       {"asynchronousConfiguration", true},
                                                       {"configuration", 2},
                                                       {"configurationFailure", 3}};
            frame["inputs"]["Mechanism/ReplayTune"] = {{"position", 0},
                                                       {"velocity", 0},
                                                       {"voltage", 0},
                                                       {"supplyVoltage", 12},
                                                       {"current", 0},
                                                       {"connected", i != 1},
                                                       {"externalEncoder", nullptr},
                                                       {"nativeControl", true},
                                                       {"asynchronousConfiguration", true},
                                                       {"configuration", i == 1 ? 1 : 0},
                                                       {"configurationFailure", 0}};
            if (i == 2)
                frame["inputs"]["/Tuning/ReplayTune/Requests"]["requests"] = wpi::json::array();
            entry.Append(frame.dump(), 100000 + i * 20000);
        }
        log.Flush();
    }
    auto oldIn = TuningSaved("FRC_REPLAY_LOG"), oldOut = TuningSaved("FRC_REPLAY_OUTPUT");
    TuningEnv("FRC_REPLAY_LOG", input.string());
    TuningEnv("FRC_REPLAY_OUTPUT", output.string());
    TuningEnv("FRC_TUNING_REPLAY_CHILD", "1");
    int result =
        std::system(("\"" + ::testing::internal::GetArgvs().at(0) +
                     "\" --gtest_filter=MechanismTuningReplay.ReexecutesRawTransactionsAgainstRecordedEnableState")
                        .c_str());
    TuningEnv("FRC_REPLAY_LOG", oldIn);
    TuningEnv("FRC_REPLAY_OUTPUT", oldOut);
    TuningEnv("FRC_TUNING_REPLAY_CHILD", "");
    ASSERT_EQ(result, 0);
    auto buffer = wpi::MemoryBuffer::GetFile(output.string());
    ASSERT_TRUE(buffer);
    wpi::log::DataLogReader reader(std::move(*buffer));
    std::map<int, std::string> names;
    int frames = 0;
    for (const auto &record : reader)
    {
        if (record.IsStart())
        {
            wpi::log::StartRecordData start;
            if (record.GetStartData(&start))
                names[start.entry] = std::string(start.name);
            continue;
        }
        std::string_view value;
        if (names[record.GetEntry()] != "/Replay/Frames" && names[record.GetEntry()] != "Replay/Frames")
            continue;
        if (!record.GetString(&value))
            continue;
        auto frame = wpi::json::parse(value);
        EXPECT_NE(frame["outputs"]["/Tuning/ReplayTune/Status"]["status"], "wrong original output");
        if (frames == 2)
        {
            EXPECT_EQ(frame["outputs"]["/Tuning/ReplayTune/Status"]["status"], "applied");
            EXPECT_EQ(frame["outputs"]["/Tuning/ReplayTune/Status"]["revision"], 1);
        }
        ++frames;
    }
    EXPECT_EQ(frames, 3);
    std::filesystem::remove_all(dir);
}
