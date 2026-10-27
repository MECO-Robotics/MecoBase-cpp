#include "base/Replay.h"
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "base/QuestNav.h"
#include "base/PieceDetection.h"
#include "QuestNavFixtures.h"
#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <frc/DataLogManager.h>
#include <frc/TimedRobot.h>
#include <frc/RobotController.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogWriter.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <map>
#include <filesystem>
#include <cstdlib>
#include <chrono>
namespace
{
void Env(const char *key, const std::string &value)
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
std::string Saved(const char *key)
{
    auto v = std::getenv(key);
    return v ? v : "";
}
} // namespace
// Isolate the global robot runtime in a child, so replay never changes another
// native test's IO mode. The source's recorded outputs are deliberately wrong.
TEST(Replay, RecomputesFromInputsWithoutConstructingNativeDrive)
{
    if (std::getenv("FRC_REPLAY_TEST_CHILD"))
    {
        // Match the actual Robot base: its unused notifier must not deadlock the
        // explicit replay clock when StartCompetition bypasses the TimedRobot loop.
        frc::TimedRobot registeredNotifier;
        base::replay::InitializeFromEnvironment();
        base::DriveConfig driveConfig;
        driveConfig.backend = base::DriveBackend::SparkMaxNavX;
        base::Drive drive(driveConfig);
        drive.ConfigureAuto(); // Global AutoBuilder must not own PID sendables at exit.
        base::MotorConfig motor{{51}, {false}, 1, 40, ""};
        auto io = base::MakeMotorIO(base::Mode::Real, base::Vendor::TalonFX, motor);
        auto *injected = dynamic_cast<base::InjectedMotorIO *>(io.get());
        ASSERT_NE(injected, nullptr);
        base::Gains gains;
        gains.p = 3;
        gains.maxAcceleration = 1000;
        gains.maxVelocity = 10;
        base::Flywheel wheel("Probe", std::move(io), gains);
        auto configuredIO = std::make_unique<base::InjectedMotorIO>();
        auto *configured = configuredIO.get();
        base::Flywheel configWheel("ConfigProbe", std::move(configuredIO), gains);
        int observationCount = 0;
        base::VisionConfig config;
        config.name = "ReplayVision";
        base::Vision vision(config, [&](const base::VisionObservation &o) {
            EXPECT_DOUBLE_EQ(o.pose.X().value(), 1);
            EXPECT_DOUBLE_EQ(o.pose.Y().value(), 2);
            EXPECT_TRUE(std::isinf(o.stdDevs[2]));
            drive.AddVision(o.pose, o.timestamp, o.stdDevs);
            ++observationCount;
        });
        int questObservations = 0;
        base::QuestNavConfig questConfig;
        questConfig.name = "ReplayQuest";
        base::QuestNav quest(questConfig, [&](const base::VisionObservation &o) {
            EXPECT_TRUE(std::isfinite(o.pose.X().value()));
            ++questObservations;
        });
        base::PieceDetectionConfig pieceConfig;
        pieceConfig.name = "ReplayPiece";
        pieceConfig.robotToCamera = {frc::Translation3d{units::meter_t{0}, units::meter_t{0}, units::meter_t{1}},
                                     frc::Rotation3d{}};
        base::PhotonPieceDetection pieces(pieceConfig, [] { return frc::Pose3d{}; });
        EXPECT_FALSE(drive.SimulationTruth().has_value());
        base::replay::RunReplay(
            [] {},
            [&] {
                EXPECT_NEAR(frc::RobotController::GetFPGATime(), 100000 + base::replay::FrameIndex() * 20000, 1);
                if (base::replay::FrameIndex() == 1)
                {
                    gains.p = 4;
                    wheel.SetGains(gains);
                }
                wheel.SetVelocity(3);
                wheel.Periodic();
                EXPECT_DOUBLE_EQ(injected->requestedVoltage, base::replay::FrameIndex() == 0   ? 6
                                                             : base::replay::FrameIndex() == 1 ? 8
                                                                                               : 0);
                configWheel.SetVelocity(2);
                configWheel.Periodic();
                EXPECT_EQ(configWheel.ConfigurationState(), base::replay::FrameIndex() == 0
                                                                ? base::MotorConfigurationState::Pending
                                                                : base::MotorConfigurationState::Ready);
                EXPECT_EQ(configured->nativeRequest.has_value(), base::replay::FrameIndex() == 2);
                pieces.Periodic();
                ASSERT_TRUE(pieces.Target());
                EXPECT_NEAR(pieces.Target()->X().value(), 1, 1e-9);
                quest.Periodic();
                EXPECT_TRUE(quest.Connected());
                if (base::replay::FrameIndex() == 0)
                    EXPECT_TRUE(
                        quest.AlignToField(frc::Pose2d{units::meter_t{5}, units::meter_t{4}, frc::Rotation2d{}}));
                vision.Periodic();
                EXPECT_TRUE(vision.Connected());
                EXPECT_DOUBLE_EQ(vision.TargetYawDegrees(), 2);
                EXPECT_NEAR(drive.Pose().Rotation().Radians().value(), 1.5707963267948966, 1e-12);
                drive.FieldDrive(base::replay::FrameIndex() == 0 ? 1 : 0, 0, 0);
                EXPECT_NEAR(drive.RequestedSpeeds().vx.value(), 0, 1e-9);
                EXPECT_NEAR(drive.RequestedSpeeds().vy.value(), base::replay::FrameIndex() == 0 ? -1 : 0, 1e-9);
                drive.ResetPose(frc::Pose2d{});
                EXPECT_NEAR(drive.Pose().Rotation().Radians().value(), 1.5707963267948966, 1e-12);
                EXPECT_THROW(base::replay::ProcessInputs("Missing", [] { return wpi::json{}; }), std::runtime_error);
            });
        EXPECT_EQ(observationCount, 3);
        EXPECT_EQ(questObservations, 2);
        base::telemetry::Flush();
        base::telemetry::Shutdown();
        base::replay::ShutdownRecording();
        base::replay::FlushOutput();
        frc::DataLogManager::Stop();
        return;
    }
    auto dir = std::filesystem::temp_directory_path() /
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::filesystem::create_directories(dir);
    auto source = dir / "input.wpilog", output = dir / "output.wpilog";
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
    frame["inputs"] = {{"Drive",
                        {{"backend", "rev"},
                         {"pose", {0, 0, 1.5707963267948966}},
                         {"speeds", {0, 0, 0}},
                         {"states", {0, 0, 0, 0, 0, 0, 0, 0}}}}};
    frame["inputs"]["Mechanism/Probe"] = {{"position", 0},
                                          {"velocity", 1},
                                          {"voltage", 0},
                                          {"supplyVoltage", 12},
                                          {"current", 0},
                                          {"connected", true},
                                          {"externalEncoder", nullptr}};
    frame["inputs"]["Vision/ReplayVision/Status"] = {{"connected", true}, {"yaw", 2}};
    frame["inputs"]["Vision/ReplayVision/Observations"] =
        wpi::json::array({{{"pose", {1, 2, .3}}, {"timestamp", .1}, {"stdDevs", {.1, .1, nullptr}}}});
    frame["inputs"]["PieceDetection/ReplayPiece"] = {{"now", .1}, {"ray", {.1, 0, -45}}};
    frame["outputs"] = {{"Mechanism/Probe/AppliedVoltageRequest", 999},
                        {"Drive/RequestedChassisSpeeds", {999, 999, 999}}};
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(source.string(), ec);
        ASSERT_FALSE(ec);
        wpi::log::StringLogEntry entry(log, "Replay/Frames");
        for (int i = 0; i < 3; ++i)
        {
            frame["sequence"] = i;
            frame["timestampUs"] = 100000 + 20000 * i;
            auto packet = questfixture::Bytes(i == 0 ? questfixture::first : questfixture::second);
            packet[10] = 7 + i;
            frame["inputs"]["Mechanism/ConfigProbe"] = {{"position", 0},
                                                        {"velocity", 0},
                                                        {"voltage", 0},
                                                        {"supplyVoltage", 12},
                                                        {"current", 0},
                                                        {"connected", i != 0},
                                                        {"externalEncoder", nullptr},
                                                        {"nativeControl", true},
                                                        {"configuration", i == 0 ? 1 : 0}};
            frame["inputs"]["QuestNav/ReplayQuest/Frames"] = {
                {"nowUs", 100000 + 20000 * i},
                {"frames", wpi::json::array({{{"timestampUs", 100000 + 20000 * i}, {"data", packet}}})}};
            if (i == 2)
                frame["inputs"]["Mechanism/Probe"]["connected"] = false;
            entry.Append(frame.dump(), 100000 + 20000 * i);
        }
        log.Flush();
    }
    const auto oldInput = Saved("FRC_REPLAY_LOG"), oldOutput = Saved("FRC_REPLAY_OUTPUT");
    Env("FRC_REPLAY_LOG", source.string());
    Env("FRC_REPLAY_OUTPUT", output.string());
    Env("FRC_REPLAY_TEST_CHILD", "1");
    const auto command = "\"" + ::testing::internal::GetArgvs().at(0) +
                         "\" --gtest_filter=Replay.RecomputesFromInputsWithoutConstructingNativeDrive";
    int result = std::system(command.c_str());
    Env("FRC_REPLAY_LOG", oldInput);
    Env("FRC_REPLAY_OUTPUT", oldOutput);
    Env("FRC_REPLAY_TEST_CHILD", "");
    EXPECT_EQ(result, 0);
    EXPECT_TRUE(std::filesystem::exists(output));
    auto buffer = wpi::MemoryBuffer::GetFile(output.string());
    ASSERT_TRUE(buffer);
    wpi::log::DataLogReader reader(std::move(*buffer));
    std::map<int, std::string> names;
    int count = 0;
    std::array<double, 4> idleAngles{};
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
        auto replayed = wpi::json::parse(value);
        ++count;
        EXPECT_DOUBLE_EQ(replayed["outputs"]["Mechanism/Probe/AppliedVoltageRequest"].get<double>(), count == 1   ? 6
                                                                                                     : count == 2 ? 8
                                                                                                                  : 0);
        EXPECT_NEAR(replayed["outputs"]["Drive/RequestedChassisSpeeds"][1].get<double>(), count == 1 ? -1 : 0, 1e-9);
        const auto &targets = replayed["outputs"]["Drive/ModuleTargets"];
        ASSERT_EQ(targets.size(), 8);
        for (size_t i = 0; i < 4; ++i)
        {
            const double angle = targets[2 * i + 1].get<double>();
            if (count == 1)
            {
                idleAngles[i] = angle;
                EXPECT_NEAR(angle, -1.5707963267948966, 1e-12);
            }
            else
            {
                EXPECT_DOUBLE_EQ(targets[2 * i].get<double>(), 0);
                EXPECT_DOUBLE_EQ(angle, idleAngles[i]);
            }
        }
        EXPECT_TRUE(replayed["outputs"]["Drive/Vision"]["noise"][2].is_null());
    }
    EXPECT_EQ(count, 3);
    std::filesystem::remove_all(dir);
}
