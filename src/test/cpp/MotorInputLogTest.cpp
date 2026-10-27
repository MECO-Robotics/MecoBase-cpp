#include "base/MotorInputLog.h"
#include <gtest/gtest.h>
#include <limits>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <wpi/DataLogWriter.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/raw_ostream.h>

TEST(MotorInputLog, NativeLogRoundTripPreservesTimestampAndAllInputsWithoutHardware)
{
    std::string bytes;
    {
        wpi::log::DataLogWriter writer(std::make_unique<wpi::raw_string_ostream>(bytes));
        wpi::log::DoubleArrayLogEntry entry(writer, "Wheel/MotorInputs", base::kMotorInputSchema);
        entry.Append(base::EncodeMotorInputs({1, 2, 3, 11, 5, true}), 4000);
        entry.Append(base::EncodeMotorInputs({6, 7, 8, 10, 9, false}), 8000);
        entry.Append(base::EncodeMotorInputs({std::numeric_limits<double>::quiet_NaN(), 0, 0, 12, 0, true}), 12000);
        writer.Flush();
    }
    auto buffer = wpi::MemoryBuffer::GetMemBufferCopy(
        std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()));
    wpi::log::DataLogReader reader(std::move(buffer));
    auto samples = base::ReadMotorInputLog(reader, "Wheel/MotorInputs");
    ASSERT_EQ(samples.size(), 3u);
    EXPECT_EQ(samples[0].timestampMicroseconds, 4000);
    base::ReplayMotorIO replay(samples);
    replay.AdvanceTo(3999);
    EXPECT_FALSE(replay.Read().connected);
    replay.AdvanceTo(4000);
    EXPECT_EQ(base::EncodeMotorInputs(replay.Read()), (std::array<double, 6>{1, 2, 3, 11, 5, 1}));
    replay.AdvanceTo(8000);
    EXPECT_EQ(base::EncodeMotorInputs(replay.Read()), (std::array<double, 6>{6, 7, 8, 10, 9, 0}));
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto faultIO = std::make_unique<base::ReplayMotorIO>(samples);
    auto *observed = faultIO.get();
    observed->AdvanceTo(12000);
    base::Flywheel wheel("ReplayFaultTest", std::move(faultIO), base::Gains{});
    wheel.SetVoltage(6);
    wheel.Periodic();
    EXPECT_TRUE(std::isnan(wheel.Inputs().position));
    EXPECT_DOUBLE_EQ(observed->requestedVoltage, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}
