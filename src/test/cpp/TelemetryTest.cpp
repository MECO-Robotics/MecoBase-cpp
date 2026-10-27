#include "base/Telemetry.h"
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include <vector>
TEST(Telemetry, SlowPublisherDoesNotBlockControlAndCoalescesPendingSnapshots)
{
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    base::telemetry::Post("test/block", [&] {
        entered.set_value();
        released.wait();
    });
    const auto ready = entered.get_future().wait_for(std::chrono::seconds(2));
    if (ready != std::future_status::ready)
    {
        release.set_value();
        FAIL() << "Publisher did not start";
        return;
    }
    std::vector<int> published;
    std::thread::id publisher;
    int snapshot = 1;
    base::telemetry::Post("test/value", [&, snapshot] { published.push_back(snapshot); });
    snapshot = 2;
    base::telemetry::Post("test/value", [&, snapshot] {
        published.push_back(snapshot);
        publisher = std::this_thread::get_id();
    });
    snapshot = 3;
    // Both posts returned while the worker was blocked; controls did not wait for IO.
    release.set_value();
    base::telemetry::Flush();
    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0], 2);
    EXPECT_NE(publisher, std::this_thread::get_id());
    EXPECT_GE(base::telemetry::Statistics().coalesced, 1u);
}
