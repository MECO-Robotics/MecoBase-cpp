#include "base/LoggedTunableNumber.h"
#include <gtest/gtest.h>
#include <hal/HAL.h>
#include <networktables/NetworkTableInstance.h>
#include <limits>
#include <frc/DataLogManager.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <chrono>
#include <thread>
#include <filesystem>
#include <map>
#include <cstdlib>

TEST(LoggedTunableNumber, LiveValuesAndDisabledDefaults)
{
    HAL_Initialize(500, 0);
    auto nt = nt::NetworkTableInstance::GetDefault();
    auto liveEntry = nt.GetEntry("/TunableNumbers/TestLiveValue");
    auto offEntry = nt.GetEntry("/TunableNumbers/TestDisabledValue");
    liveEntry.Unpublish();
    offEntry.Unpublish();
    base::LoggedTunableNumber live("TestLiveValue", 2, true);
    base::LoggedTunableNumber off("TestDisabledValue", 3, false);
    EXPECT_DOUBLE_EQ(live.Get(), 2);
    liveEntry.SetDouble(5);
    offEntry.SetDouble(7);
    EXPECT_DOUBLE_EQ(live.Get(), 5);
    EXPECT_DOUBLE_EQ(off.Get(), 3);
    liveEntry.SetDouble(std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(live.Get(), 5);
    EXPECT_FALSE(live.LastInputValid());
    live.ReportEffective(4, "clamped");
    EXPECT_DOUBLE_EQ(nt.GetEntry("/TunableNumbers/TestLiveValue/Effective").GetDouble(-1), 4);
    EXPECT_EQ(nt.GetEntry("/TunableNumbers/TestLiveValue/Status").GetString(""), "clamped");
    off.ReportEffective(3, "applied");
    EXPECT_FALSE(nt.GetEntry("/TunableNumbers/TestDisabledValue/Effective").Exists());
}

TEST(LoggedTunableNumber, DataLogCapturesRequestedEffectiveAndStatus)
{
    const char *childPath = std::getenv("MECO_TUNABLE_LOG_CHILD");
    if (childPath)
    {
        HAL_Initialize(500, 0);
        frc::DataLogManager::Start(childPath, "tuning.wpilog", .01);
        auto nt = nt::NetworkTableInstance::GetDefault();
        base::LoggedTunableNumber number("LogProbe", 1, true);
        nt.GetEntry("/TunableNumbers/LogProbe").SetDouble(7);
        EXPECT_DOUBLE_EQ(number.Get(), 7);
        number.ReportEffective(4, "clamped");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        frc::DataLogManager::Stop();
        return;
    }
    auto dir = std::filesystem::temp_directory_path() /
               ("meco-tuning-log-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);
#ifdef _WIN32
    _putenv_s("MECO_TUNABLE_LOG_CHILD", dir.string().c_str());
#else
    setenv("MECO_TUNABLE_LOG_CHILD", dir.string().c_str(), 1);
#endif
    const int result = std::system(("\"" + ::testing::internal::GetArgvs().at(0) +
                                    "\" --gtest_filter=LoggedTunableNumber.DataLogCapturesRequestedEffectiveAndStatus").c_str());
#ifdef _WIN32
    _putenv_s("MECO_TUNABLE_LOG_CHILD", "");
#else
    unsetenv("MECO_TUNABLE_LOG_CHILD");
#endif
    ASSERT_EQ(result, 0);
    auto buffer = wpi::MemoryBuffer::GetFile((dir / "tuning.wpilog").string());
    ASSERT_TRUE(buffer);
    wpi::log::DataLogReader reader(std::move(*buffer));
    std::map<int, std::string> names;
    bool requested=false, effective=false, status=false;
    for (const auto &record : reader)
    {
        if (record.IsStart())
        {
            wpi::log::StartRecordData start;
            if (record.GetStartData(&start)) names[start.entry]=std::string(start.name);
            continue;
        }
        const auto &name=names[record.GetEntry()];
        double value;
        std::string_view text;
        if (name.ends_with("/TunableNumbers/LogProbe") && record.GetDouble(&value) && value==7) requested=true;
        if (name.ends_with("/TunableNumbers/LogProbe/Effective") && record.GetDouble(&value) && value==4) effective=true;
        if (name.ends_with("/TunableNumbers/LogProbe/Status") && record.GetString(&text) && text=="clamped") status=true;
    }
    EXPECT_TRUE(requested);
    EXPECT_TRUE(effective);
    EXPECT_TRUE(status);
    std::filesystem::remove_all(dir);
}
