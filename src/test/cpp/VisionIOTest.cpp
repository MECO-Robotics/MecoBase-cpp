#include "base/Vision.h"
#include <gtest/gtest.h>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <iostream>

TEST(VisionIO, AggregatesOnceInSourceOrderAndStatusReadsArePassive)
{
    std::array<int, 2> calls{};
    std::vector<double> received;
    std::vector<std::unique_ptr<base::VisionIO>> sources;
    for (size_t i = 0; i < 2; ++i)
        sources.push_back(std::make_unique<base::InjectedVisionIO>([&, i](base::VisionInputs &input) {
            ++calls[i];
            input.connected = true;
            input.targetYawDegrees = i + 1;
            input.observations.push_back({frc::Pose2d{}, units::second_t{double(i + 1)}, {.1, .1, .1}});
        }));
    base::VisionSystem system(std::move(sources), [&](const auto &o) { received.push_back(o.timestamp.value()); });
    system.Periodic();
    EXPECT_EQ(calls, (std::array<int, 2>{1, 1}));
    EXPECT_EQ(received, (std::vector<double>{1, 2}));
    const auto capacity = system.Inputs().observations.capacity();
    for (int i = 0; i < 10; ++i)
        EXPECT_TRUE(system.Inputs().connected);
    EXPECT_EQ(calls, (std::array<int, 2>{1, 1}));
    system.Periodic();
    EXPECT_EQ(received, (std::vector<double>{1, 2, 1, 2}));
    EXPECT_EQ(system.Inputs().observations.capacity(), capacity);
    if (std::getenv("FRC_VISION_IO_BENCHMARK"))
    {
        std::vector<double> timings;
        timings.reserve(20000);
        received.reserve(100000);
        for (int i = 0; i < 20000; ++i)
        {
            const auto start = std::chrono::steady_clock::now();
            system.Periodic();
            timings.push_back(
                std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());
        }
        std::sort(timings.begin(), timings.end());
        std::cout << "VisionIO benchmark: 2 injected sources, 1 observation/source, 20000 cycles; median_us="
                  << timings[10000] << " p99_us=" << timings[19800] << std::endl;
    }
}
