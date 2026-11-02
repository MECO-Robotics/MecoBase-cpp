#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <hal/DriverStationTypes.h>
#include <frc/RobotBase.h>
#include <cmath>
#include <numbers>
#ifndef __FRC_ROBORIO__
#include "base/SimulationRuntime.h"
#endif

namespace
{
class Module final : public base::SwerveModuleIO
{
  public:
    base::SwerveModuleInputs Read() override
    {
        return input;
    }
    void Set(const frc::SwerveModuleState &state, double) override
    {
        target = state;
        stopped = false;
    }
    void SetVoltage(double) override
    {
    }
    void Stop() override
    {
        stopped = true;
    }
    base::SwerveModuleInputs input{0, 0, 0, 0, 0, true};
    frc::SwerveModuleState target{};
    bool stopped = true;
};
class Gyro final : public base::SwerveGyroIO
{
  public:
    base::SwerveGyroInputs Read() override
    {
        return {yaw, true};
    }
    double yaw{};
};
} // namespace

TEST(DriveSystemBoundary, FieldDriveRespectsModeAllianceAndHeading)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed1);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
    base::DriveConfig config;
    config.backend = base::DriveBackend::SparkMaxNavX;
    base::SwerveIO io;
    std::array<Module *, 4> modules{};
    for (size_t i = 0; i < 4; ++i)
    {
        auto m = std::make_unique<Module>();
        modules[i] = m.get();
        io.modules[i] = std::move(m);
    }
    auto gyro = std::make_unique<Gyro>();
    auto *gyroRaw = gyro.get();
    io.gyro = std::move(gyro);
    base::Drive drive(config, std::move(io));
    (void)gyroRaw;
    drive.ResetPose({units::meter_t{0}, units::meter_t{0}, frc::Rotation2d{units::degree_t{90}}});
    drive.Periodic();
    drive.FieldDrive(1, 0, .4);
    const auto red = drive.RequestedSpeeds();
    EXPECT_NEAR(red.vx.value(), 0, .01);
    EXPECT_NEAR(red.vy.value(), 1, .01);
    EXPECT_NEAR(red.omega.value(), .4, .01);
    drive.StopWithX();
    drive.Periodic();
    for (size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(modules[i]->target.angle.Radians().value(),
                    std::atan2(config.modules[i].yMeters, config.modules[i].xMeters), 1e-9);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
    drive.FieldDrive(1, 0, 1);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(), 0);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vy.value(), 0);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().omega.value(), 0);
}

TEST(DriveSystemBoundary, FieldDriveUsesConfiguredControlPeriod)
{
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed1);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();

    auto makeIo = [] {
        base::SwerveIO io;
        for (auto &module : io.modules)
            module = std::make_unique<Module>();
        io.gyro = std::make_unique<Gyro>();
        return io;
    };
    base::DriveConfig fastConfig;
    fastConfig.backend = base::DriveBackend::SparkMaxNavX;
    fastConfig.controlPeriodSeconds = .002;
    base::Drive fast{fastConfig, makeIo()};
    fast.FieldDrive(1, 0, .4);
    const auto fieldRelative = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{-1}, units::meters_per_second_t{0}, units::radians_per_second_t{.4},
        frc::Rotation2d{});
    const auto expectedFast = frc::ChassisSpeeds::Discretize(fieldRelative, units::second_t{.002});
    EXPECT_NEAR(fast.RequestedSpeeds().vx.value(), expectedFast.vx.value(), 1e-12);
    EXPECT_NEAR(fast.RequestedSpeeds().vy.value(), expectedFast.vy.value(), 1e-12);

    base::DriveConfig defaultConfig = fastConfig;
    defaultConfig.controlPeriodSeconds = .02;
    base::Drive normal{defaultConfig, makeIo()};
    normal.FieldDrive(1, 0, .4);
    const auto expectedNormal = frc::ChassisSpeeds::Discretize(fieldRelative, units::second_t{.02});
    EXPECT_NEAR(normal.RequestedSpeeds().vx.value(), expectedNormal.vx.value(), 1e-12);
    EXPECT_NEAR(normal.RequestedSpeeds().vy.value(), expectedNormal.vy.value(), 1e-12);
    EXPECT_GT(std::abs(fast.RequestedSpeeds().vy.value() - normal.RequestedSpeeds().vy.value()), 1e-4);
}

#ifndef __FRC_ROBORIO__
TEST(DriveSystemBoundary, ExternalSimulationSchedulingStepsOnlyWhenOwnerCallsIt)
{
    auto &runtime = base::simulation::GetSimulationRuntime();
    runtime.Reset();
    base::DriveConfig config;
    config.backend = base::DriveBackend::SparkMaxNavX;
    config.externalSimulationScheduling = true;
    base::SwerveIO io;
    for (auto &module : io.modules)
        module = std::make_unique<Module>();
    io.gyro = std::make_unique<Gyro>();
    base::Drive drive{config, std::move(io)};

    drive.SimulationPeriodic();
    EXPECT_DOUBLE_EQ(runtime.Time(), 0.0);
    EXPECT_EQ(drive.SimulationStepCount(), 0u);
    drive.StepSimulationPeriod();
    EXPECT_DOUBLE_EQ(runtime.Time(), .02);
    EXPECT_EQ(drive.SimulationStepCount(), 1u);
    drive.SimulationPeriodic();
    EXPECT_DOUBLE_EQ(runtime.Time(), .02);
    EXPECT_EQ(drive.SimulationStepCount(), 1u);
    runtime.Reset();
}
#endif
