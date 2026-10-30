#include "base/Mechanism.h"
#include "base/detail/SparkControl.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/sim/SparkSimFaultManager.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/HAL.h>
#include <gtest/gtest.h>
#include <limits>
#include <chrono>
#include <thread>

TEST(SparkAdapter, DefaultRealAdapterUsesOnboardPhysicalSetpoints)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    base::MotorConfig config{{51}, {true}, 7, 35, "rio"};
    auto io = base::MakeMotorIO(base::Mode::Real, base::Vendor::SparkMax, config);
    base::Gains gains;
    gains.minimum = -2;
    gains.maximum = 3;
    // Real adapters cannot silently select a software PID, even with the legacy
    // Phoenix option absent. Scale .5 represents metres per output revolution.
    ASSERT_TRUE(io->ConfigureControl({gains, .5, base::Gravity::Constant, true, false}));
    frc::sim::SimDeviceSim device{"SPARK MAX [51]"};
    ASSERT_NE(static_cast<HAL_SimDeviceHandle>(device), 0);
    base::NativeControlRequest request{base::NativeControlRequest::Kind::Position, 1.25, 1, 1, 0, 0};
    ASSERT_TRUE(io->SetNative(request));
    EXPECT_DOUBLE_EQ(device.GetDouble("Setpoint").Get(), 2.5);
    request.kind = base::NativeControlRequest::Kind::Velocity;
    request.target = .75;
    ASSERT_TRUE(io->SetNative(request));
    EXPECT_DOUBLE_EQ(device.GetDouble("Setpoint").Get(), 1.5);
    io->SetVoltage(0);
    EXPECT_THROW(io->ConfigureControl({gains, .5, base::Gravity::Constant, true, true}), std::invalid_argument);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    EXPECT_NO_THROW(io->SetBrake(true)); // Default cached brake sends no configuration.
    EXPECT_THROW(io->SetBrake(false), std::logic_error);
    EXPECT_TRUE(io->ConfigureControl({gains, .5, base::Gravity::Constant, true, false}));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (io->ConfigurationState() == base::MotorConfigurationState::Pending &&
           std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    EXPECT_EQ(io->ConfigurationState(), base::MotorConfigurationState::Ready);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
}

TEST(SparkAdapter, GravityModeReconfigurationRetainsSelectedCompensation)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    rev::spark::SparkMax motor{52, rev::spark::SparkLowLevel::MotorType::kBrushless};
    for (auto mode : {base::Gravity::Constant, base::Gravity::Cosine, base::Gravity::Constant})
    {
        rev::spark::SparkMaxConfig config;
        base::detail::ConfigureSparkGravity(config.closedLoop.feedForward, 1.75, .5, mode, true);
        ASSERT_EQ(
            motor.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters),
            rev::REVLibError::kOk);
        auto &feedforward = motor.configAccessor.closedLoop.feedForward;
        // Both accessors expose REV's shared gravity coefficient. A zero cosine
        // ratio selects constant compensation; cosine mode selects the given scale.
        EXPECT_NEAR(feedforward.getkG(), 1.75, 1e-6);
        EXPECT_NEAR(feedforward.getkCosRatio(), mode == base::Gravity::Constant ? 0 : .5, 1e-6);
    }
}

TEST(SparkAdapter, ResetOnEitherDeviceLatchesUnhealthyAfterWarningClears)
{
    HAL_Initialize(500, 0);
    rev::spark::SparkMax leader{53, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax follower{54, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkSimFaultManager faults{&follower};
    bool resetLatched = false;
    ASSERT_TRUE(base::detail::SparkDeviceConfigured(leader, resetLatched));
    ASSERT_TRUE(base::detail::SparkDeviceConfigured(follower, resetLatched));
    faults.SetFaults(rev::spark::SparkBase::Faults{0x4}); // Active sensor fault.
    EXPECT_FALSE(base::detail::SparkDeviceConfigured(follower, resetLatched));
    EXPECT_FALSE(resetLatched);
    faults.SetFaults(rev::spark::SparkBase::Faults{0});
    ASSERT_TRUE(base::detail::SparkDeviceConfigured(follower, resetLatched));
    faults.SetStickyWarnings(rev::spark::SparkBase::Warnings{0x40});
    EXPECT_FALSE(base::detail::SparkDeviceConfigured(follower, resetLatched));
    EXPECT_TRUE(resetLatched);
    // Clearing warning history externally does not reconstruct lost settings.
    faults.SetStickyWarnings(rev::spark::SparkBase::Warnings{0});
    EXPECT_FALSE(base::detail::SparkDeviceConfigured(follower, resetLatched));
    EXPECT_FALSE(base::detail::SparkDeviceConfigured(leader, resetLatched));
}

TEST(SparkAdapter, PidMillisecondsPreservePhysicalVoltage)
{
    HAL_Initialize(500, 0);
    rev::spark::SparkMax motor{55, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMaxConfig config;
    base::Gains gains;
    gains.p = 2;
    gains.i = 3;
    gains.d = .4;
    const double scale = .5; // Metres per output rotation.
    base::detail::ConfigureSparkPid(config.closedLoop, gains, scale);
    ASSERT_EQ(motor.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters),
              rev::REVLibError::kOk);
    auto &pid = motor.configAccessor.closedLoop;
    const double error = .4, previousError = .1, dt = .02;
    // Reconstruct a physical voltage using the actual REV configuration, with
    // error in output rotations and integration/derivative time in milliseconds.
    const double firmwareVolts = 12 * (pid.GetP() * error / scale + pid.GetI() * error / scale * (dt * 1000) +
                                       pid.GetD() * (error - previousError) / scale / (dt * 1000));
    const double physicalVolts = gains.p * error + gains.i * error * dt + gains.d * (error - previousError) / dt;
    EXPECT_NEAR(firmwareVolts, physicalVolts, 1e-5); // Vendor configuration stores floats.
    gains.d = std::numeric_limits<double>::max();
    EXPECT_THROW(base::detail::ConfigureSparkPid(config.closedLoop, gains, scale), std::invalid_argument);
}
