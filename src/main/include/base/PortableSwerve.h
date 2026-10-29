#pragma once
#include <array>
#include <memory>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
namespace base
{
struct DriveConfig;
struct SparkSwerveGains;
// Calibrated physical coordinates: metres, radians, SI rates and applied volts.
struct SwerveModuleInputs
{
    double distance{}, velocity{}, angle{}, angularVelocity{}, voltage{};
    bool healthy{false};
};
struct SwerveGyroInputs
{
    double yaw{};
    bool healthy{false};
};
class SwerveModuleIO
{
  public:
    virtual ~SwerveModuleIO() = default;
    virtual SwerveModuleInputs Read() = 0;
    virtual void Set(const frc::SwerveModuleState &, double forceVolts) = 0;
    virtual void SetVoltage(double volts) = 0;
    virtual void Stop() = 0;
    // Simulation controllers share the same requests but never construct vendor devices.
    virtual std::array<double, 2> SimVoltages(double, double)
    {
        return {};
    }
    virtual void SetSimInputs(SwerveModuleInputs)
    {
    }
    // Called only by the configuration worker while IO is inhibited.
    virtual bool ConfigureGains(const SparkSwerveGains &) { return false; }
};
class SwerveGyroIO
{
  public:
    virtual ~SwerveGyroIO() = default;
    virtual SwerveGyroInputs Read() = 0;
    virtual void SetSimYaw(double)
    {
    }
};
struct SwerveIO
{
    std::array<std::unique_ptr<SwerveModuleIO>, 4> modules;
    std::unique_ptr<SwerveGyroIO> gyro;
};
SwerveIO MakeSparkNavXSwerveIO(const DriveConfig &, bool simulation);
// The real sampler owns IO calls; the robot thread owns estimator updates. The
// bounded queue drops oldest samples on overruns rather than blocking controls.
class PortableSwerve
{
  public:
    struct Snapshot
    {
        frc::Pose2d pose{};
        frc::ChassisSpeeds speeds{};
        std::array<frc::SwerveModuleState, 4> states{}, targets{};
        std::array<frc::SwerveModulePosition, 4> positions{};
        std::array<double, 4> voltages{};
        double timestamp{}, rawYaw{};
        bool healthy{};
    };
    PortableSwerve(const DriveConfig &, SwerveIO, bool threaded);
    ~PortableSwerve();
    void Velocity(const frc::ChassisSpeeds &, const std::array<double, 4> &fx = {},
                  const std::array<double, 4> &fy = {});
    void Voltage(double);
    void Brake();
    void Update();
    void Sample(double timestamp, bool enabled); // explicit simulation / injected IO acquisition
    Snapshot State() const;
    void ResetPose(const frc::Pose2d &);
    void AddVision(const frc::Pose2d &, units::second_t, std::array<double, 3>);
    std::array<double, 2> SimVoltages(size_t, double, double);
    void SetSimInputs(size_t, SwerveModuleInputs);
    void SetSimYaw(double);
    // Nonblocking stop attempt; configuration runs on a worker after it succeeds.
    bool InhibitForConfiguration();
    bool ConfigureGains(const SparkSwerveGains &);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace base
