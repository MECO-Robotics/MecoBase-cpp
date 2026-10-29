#include "base/PortableSwerve.h"
#include "base/Drive.h"
#include "base/LiveDriveMotorTuning.h"
#include <frc/RobotController.h>
#include <hal/HAL.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <limits>
#include <future>
#include <functional>
#include <thread>
#include <chrono>
#include <networktables/NetworkTableInstance.h>
namespace
{
class Module final : public base::SwerveModuleIO
{
  public:
    base::SwerveModuleInputs input{0, 0, 0, 0, 0, true};
    std::function<void()> beforeRead;
    std::function<bool()> configure;
    int reads = 0;
    bool stopped = true;
    double force{};
    frc::SwerveModuleState target{};
    base::SwerveModuleInputs Read() override
    {
        ++reads;
        if (beforeRead)
            beforeRead();
        return input;
    }
    void Set(const frc::SwerveModuleState &s, double ff) override
    {
        stopped = false;
        target = s;
        force = ff;
    }
    void SetVoltage(double v) override
    {
        stopped = v == 0;
    }
    void Stop() override
    {
        stopped = true;
    }
    bool ConfigureGains(const base::SparkSwerveGains &) override
    {
        return configure ? configure() : true;
    }
};
class Gyro final : public base::SwerveGyroIO
{
  public:
    int reads = 0;
    double yaw = 0;
    base::SwerveGyroInputs Read() override
    {
        ++reads;
        return {yaw, true};
    }
};
struct Rig
{
    std::array<Module *, 4> modules{};
    Gyro *gyro{};
    base::SwerveIO io;
    Rig()
    {
        for (size_t i = 0; i < 4; ++i)
        {
            auto module = std::make_unique<Module>();
            modules[i] = module.get();
            io.modules[i] = std::move(module);
        }
        auto device = std::make_unique<Gyro>();
        gyro = device.get();
        io.gyro = std::move(device);
    }
};
} // namespace
TEST(PortableSwerve, EstimatorResetAndFailStopRecoveryUseFreshInputsAndRequests)
{
    HAL_Initialize(500, 0);
    base::DriveConfig c;
    c.backend = base::DriveBackend::SparkMaxNavX;
    Rig rig;
    base::PortableSwerve drive(c, std::move(rig.io), false);
    const double now = frc::RobotController::GetFPGATime() * 1e-6;
    drive.Sample(now - .02, true);
    drive.Update();
    drive.Velocity({units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
    for (auto *m : rig.modules)
    {
        m->input.distance = 1;
        m->input.velocity = 1;
    }
    drive.Sample(now, true);
    drive.Update();
    EXPECT_NEAR(drive.State().pose.X().value(), 1, 1e-8);
    EXPECT_FALSE(rig.modules[0]->stopped);
    drive.ResetPose({units::meter_t{5}, units::meter_t{2}, frc::Rotation2d{}});
    for (auto *m : rig.modules)
        m->input.distance = 1.2;
    drive.Sample(now + .01, false);
    drive.Update();
    EXPECT_NEAR(drive.State().pose.X().value(), 5.2, 1e-8);
    EXPECT_TRUE(rig.modules[0]->stopped);
    drive.Sample(now + .02, true);
    EXPECT_TRUE(rig.modules[0]->stopped);
    drive.Velocity({units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
    drive.Sample(now + .03, true);
    EXPECT_FALSE(rig.modules[0]->stopped);
    rig.modules[1]->input.healthy = false;
    drive.Sample(now + .04, true);
    EXPECT_TRUE(rig.modules[0]->stopped);
    rig.modules[1]->input.healthy = true;
    drive.Sample(now + .05, true);
    EXPECT_TRUE(rig.modules[0]->stopped);
    drive.Velocity({units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
    EXPECT_THROW(drive.Velocity({units::meters_per_second_t{0}, units::meters_per_second_t{0},
                                 units::radians_per_second_t{std::numeric_limits<double>::quiet_NaN()}}),
                 std::invalid_argument);
    drive.Sample(now + .06, true);
    EXPECT_TRUE(rig.modules[0]->stopped);
}
TEST(PortableSwerve, WheelForceAddsOnlyProjectedTorqueVoltage)
{
    HAL_Initialize(500, 0);
    base::DriveConfig c;
    c.backend = base::DriveBackend::SparkMaxNavX;
    Rig rig;
    base::PortableSwerve drive(c, std::move(rig.io), false);
    drive.Sample(frc::RobotController::GetFPGATime() * 1e-6, true);
    drive.Velocity({units::meters_per_second_t{2}, units::meters_per_second_t{0}, units::radians_per_second_t{0}},
                   {10, 10, 10, 10}, {20, 20, 20, 20});
    drive.Sample(frc::RobotController::GetFPGATime() * 1e-6, true);
    // NEO stall torque 2.6 N m at 12 V; no velocity back EMF in this term.
    EXPECT_NEAR(rig.modules[0]->force, 10 * c.wheelRadius / c.driveRatio * 12 / 2.6, 1e-6);
    EXPECT_NEAR(rig.modules[0]->target.speed.value(), 2, 1e-10);
    std::array<double, 4> huge;
    huge.fill(std::numeric_limits<double>::max());
    EXPECT_THROW(
        drive.Velocity({units::meters_per_second_t{1}, units::meters_per_second_t{1}, units::radians_per_second_t{0}},
                       huge, huge),
        std::invalid_argument);
}

TEST(PortableSwerve, ConfigurationRejectsUnsupportedBusAndInvalidAbsoluteCalibration)
{
    base::DriveConfig config;
    config.backend = base::DriveBackend::SparkMaxNavX;
    for (auto &module : config.modules)
        module.encoder = 0;
    config.pigeon = -1;
    EXPECT_NO_THROW(config.Validate());
    config.bus = "canivore";
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.bus = "rio";
    config.modules[0].offsetTurns = 1;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.modules[0].offsetTurns = .75;
    EXPECT_NO_THROW(config.Validate());
    config.driveCurrent = 1e300;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.driveCurrent = 40.5;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.driveCurrent = 40;
    for (bool steering : {false, true})
    {
        auto invalid = config;
        (steering ? invalid.spark.steerD : invalid.spark.driveD) = std::numeric_limits<double>::max() / 500;
        EXPECT_THROW(invalid.Validate(), std::invalid_argument);
    }
    EXPECT_NO_THROW(config.Validate());
}

TEST(PortableSwerve, BlockedDeviceReadDoesNotBlockCommandsOrCrossEstimatorReset)
{
    HAL_Initialize(500, 0);
    base::DriveConfig config;
    config.backend = base::DriveBackend::SparkMaxNavX;
    Rig rig;
    base::PortableSwerve drive(config, std::move(rig.io), false);
    const double now = frc::RobotController::GetFPGATime() * 1e-6;
    drive.Sample(now - .02, false);
    drive.Update();
    for (auto *module : rig.modules)
        module->input.distance = 1;
    std::promise<void> reading, release;
    auto started = reading.get_future();
    auto released = release.get_future().share();
    rig.modules[0]->beforeRead = [&] {
        reading.set_value();
        released.wait();
    };
    auto sample = std::async(std::launch::async, [&] { drive.Sample(now, true); });
    EXPECT_EQ(started.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    // These calls run while the device read is held. They must not acquire its IO lock.
    auto publication = std::async(std::launch::async, [&] {
        drive.Velocity({units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
        drive.ResetPose({units::meter_t{5}, units::meter_t{0}, frc::Rotation2d{}});
    });
    const auto ready = publication.wait_for(std::chrono::milliseconds(100));
    release.set_value();
    publication.get();
    sample.get();
    EXPECT_EQ(ready, std::future_status::ready);
    rig.modules[0]->beforeRead = {};
    drive.Update();
    EXPECT_NEAR(drive.State().pose.X().value(), 5, 1e-10);
    EXPECT_FALSE(rig.modules[0]->stopped); // newest request survived the blocked read
    drive.Sample(now + .01, true);
    EXPECT_FALSE(rig.modules[0]->stopped);
}

TEST(PortableSwerve, InjectedDriveAcquiresAndExecutesThroughPublicPeriodic)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEStop(false);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
    base::DriveConfig config;
    config.backend = base::DriveBackend::SparkMaxNavX;
    Rig rig;
    for (auto *module : rig.modules)
        module->input.distance = 10;
    {
        base::Drive drive(config, std::move(rig.io));
        EXPECT_FALSE(drive.SimulationTruth().has_value());
        drive.Periodic();
        EXPECT_NEAR(drive.Pose().X().value(), 0, 1e-8); // historical encoder travel is not a new displacement
        for (auto *module : rig.modules)
        {
            EXPECT_EQ(module->reads, 1);
            module->input.distance = 10.2;
            module->input.velocity = .5;
        }
        rig.gyro->yaw = .4;
        drive.RunVelocity(
            {units::meters_per_second_t{.8}, units::meters_per_second_t{0}, units::radians_per_second_t{0}});
        drive.Periodic();
        EXPECT_EQ(rig.gyro->reads, 2);
        EXPECT_NEAR(drive.Pose().Rotation().Radians().value(), .4, 1e-8);
        EXPECT_GT(drive.Pose().X().value(), .18);
        EXPECT_LT(drive.Pose().X().value(), .21);
        EXPECT_NEAR(drive.Speeds().vx.value(), .5, 1e-8);
        for (auto *module : rig.modules)
        {
            EXPECT_EQ(module->reads, 2);
            EXPECT_FALSE(module->stopped);
            EXPECT_NEAR(module->target.speed.value(), .8, 1e-8);
        }
        frc::sim::DriverStationSim::SetEnabled(false);
        frc::sim::DriverStationSim::NotifyNewData();
        frc::DriverStation::RefreshData();
        drive.Periodic();
        for (auto *module : rig.modules)
            EXPECT_TRUE(module->stopped);
    }
}

TEST(PortableSwerve, LiveGainConfigurationInhibitsAndRecoversFromFault)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
    base::DriveConfig config; config.backend=base::DriveBackend::SparkMaxNavX;
    Rig rig;
    std::promise<void> entered, release;
    auto released=release.get_future().share();
    rig.modules[0]->configure=[&] { entered.set_value(); released.wait(); return false; };
    base::Drive drive(config,std::move(rig.io));
    drive.EnableLiveMotorTuning();
    drive.Periodic();
    const frc::ChassisSpeeds moving{units::meters_per_second_t{1},units::meters_per_second_t{0},units::radians_per_second_t{0}};
    drive.RunVelocity(moving); drive.Periodic();
    auto gains=drive.MotorGains(); gains.driveP+=.1;
    ASSERT_TRUE(drive.SetMotorGains(gains));
    const auto enteredStatus=entered.get_future().wait_for(std::chrono::seconds(2));
    if (enteredStatus!=std::future_status::ready) {release.set_value(); FAIL() << "Configuration worker did not start";}
    EXPECT_EQ(drive.MotorGainState(),base::DriveGainState::Pending);
    drive.RunVelocity(moving);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    auto update=std::async(std::launch::async,[&] {drive.Periodic();});
    const auto updateStatus=update.wait_for(std::chrono::milliseconds(200));
    release.set_value();
    update.wait();
    EXPECT_EQ(updateStatus,std::future_status::ready); // Scheduler did not wait for controller IO.
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (drive.MotorGainState()==base::DriveGainState::Pending && std::chrono::steady_clock::now()<deadline) {
        drive.Periodic(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(drive.MotorGainState(),base::DriveGainState::Fault);
    drive.Periodic(); // Consume the completed failed configuration before retrying.
    EXPECT_DOUBLE_EQ(drive.MotorGains().driveP,config.spark.driveP);
    drive.RunVelocity(moving);
    EXPECT_DOUBLE_EQ(drive.RequestedSpeeds().vx.value(),0);
    rig.modules[0]->configure=[] {return true;};
    ASSERT_TRUE(drive.SetMotorGains(gains));
    const auto retryDeadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (drive.MotorGainState()==base::DriveGainState::Pending && std::chrono::steady_clock::now()<retryDeadline) {
        drive.Periodic(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(drive.MotorGainState(),base::DriveGainState::Ready);
    EXPECT_DOUBLE_EQ(drive.MotorGains().driveP,gains.driveP);
    drive.Periodic();
    for (auto *module : rig.modules) EXPECT_TRUE(module->stopped); // Old request is not replayed.
    drive.RunVelocity(moving); drive.Periodic();
    for (auto *module : rig.modules) EXPECT_FALSE(module->stopped);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
}

TEST(PortableSwerve, SimulatedSparkUsesUpdatedControllerGain)
{
    HAL_Initialize(500, 0);
    base::DriveConfig config; config.backend=base::DriveBackend::SparkMaxNavX;
    auto io=base::MakeSparkNavXSwerveIO(config,true);
    auto& module=*io.modules[0];
    const frc::SwerveModuleState request{units::meters_per_second_t{1},frc::Rotation2d{}};
    module.Set(request,0);
    const auto before=module.SimVoltages(.02,12)[0];
    auto changed=config.spark; changed.driveP+=.1;
    ASSERT_TRUE(module.ConfigureGains(changed));
    const auto after=module.SimVoltages(.02,12)[0];
    EXPECT_GT(after,before);
}

TEST(PortableSwerve, SparkLiveTunerAppliesIndividualGains)
{
    HAL_Initialize(500, 0);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
    auto nt=nt::NetworkTableInstance::GetDefault();
    auto p=nt.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kP");
    p.Unpublish();
    base::DriveConfig config; config.backend=base::DriveBackend::SparkMaxNavX;
    Rig rig;
    base::Drive drive(config,std::move(rig.io));
    base::LiveDriveMotorTuning tuning(drive,true);
    p.SetDouble(.2);
    tuning.Periodic();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (drive.MotorGainState()==base::DriveGainState::Pending && std::chrono::steady_clock::now()<deadline) {
        drive.Periodic(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(drive.MotorGainState(),base::DriveGainState::Ready);
    tuning.Periodic();
    EXPECT_DOUBLE_EQ(drive.MotorGains().driveP,.2);
    EXPECT_DOUBLE_EQ(nt.GetEntry("/TunableNumbers/Drive/MotorGains/Drive/kP/Effective").GetDouble(-1),.2);
    EXPECT_EQ(nt.GetEntry("/Tuning/Drive/MotorStatus").GetString(""),"applied");
    p.Unpublish();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData(); frc::DriverStation::RefreshData();
}
