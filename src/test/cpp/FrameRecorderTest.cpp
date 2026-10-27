#include "base/FrameRecorder.h"
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>
#include <vector>
TEST(FrameRecorder, SlowSinkPreservesSnapshotsInOrderAndReportsOverflow)
{
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    std::vector<int> values;
    std::vector<int64_t> timestamps;
    std::thread::id worker;
    int faults = 0;
    base::FrameRecorder recorder(
        [&](const wpi::json &snapshot, int64_t timestamp) {
            worker = std::this_thread::get_id();
            if (values.empty())
            {
                entered.set_value();
                released.wait();
            }
            values.push_back(snapshot.at("value").get<int>());
            timestamps.push_back(timestamp);
        },
        [&](auto stats) {
            EXPECT_GT(stats.rejected, 0u);
            ++faults;
        },
        2);
    EXPECT_TRUE(recorder.Submit({{"value", 1}}, 100));
    auto ready = entered.get_future().wait_for(std::chrono::seconds(2));
    if (ready != std::future_status::ready)
    {
        release.set_value();
        FAIL() << "Recorder did not start";
        return;
    }
    wpi::json snapshot = {{"value", 2}};
    EXPECT_TRUE(recorder.Submit(snapshot, 200));
    snapshot["value"] = 99;
    EXPECT_TRUE(recorder.Submit({{"value", 3}}, 300));
    // The sink remains blocked; producers returned without disk/sink progress.
    EXPECT_FALSE(recorder.Submit({{"value", 4}}, 400));
    release.set_value();
    recorder.Flush();
    EXPECT_EQ(values, (std::vector<int>{1, 2, 3}));
    EXPECT_EQ(timestamps, (std::vector<int64_t>{100, 200, 300}));
    EXPECT_NE(worker, std::this_thread::get_id());
    EXPECT_EQ(faults, 1);
    EXPECT_FALSE(recorder.Submit({{"value", 5}}, 500));
    recorder.Stop();
    const auto stats = recorder.Statistics();
    EXPECT_EQ(stats.written, 3u);
    EXPECT_EQ(stats.rejected, 2u);
    EXPECT_EQ(stats.failed, 0u);
}
TEST(FrameRecorder, SinkFailureInvalidatesRecordingWithoutEscapingWorker)
{
    int faults = 0;
    base::FrameRecorder recorder([](auto, int64_t) { throw std::runtime_error("disk adapter failure"); },
                                 [&](auto stats) {
                                     EXPECT_EQ(stats.failed, 1u);
                                     ++faults;
                                 });
    EXPECT_TRUE(recorder.Submit({{"value", 1}}, 100));
    recorder.Flush();
    EXPECT_EQ(faults, 1);
    EXPECT_EQ(recorder.Statistics().failed, 1u);
    EXPECT_FALSE(recorder.Submit({{"value", 2}}, 200));
}
