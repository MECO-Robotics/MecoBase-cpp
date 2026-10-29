#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/Replay.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
using namespace units::literals;
namespace p = ctre::phoenix6;
namespace s = p::swerve;
namespace base
{
namespace
{
class BorrowedPathController final : public pathplanner::PathFollowingController
{
  public:
    explicit BorrowedPathController(std::weak_ptr<std::shared_ptr<pathplanner::PathFollowingController>> owner)
        : owner_(std::move(owner))
    {
    }
    frc::ChassisSpeeds calculateRobotRelativeSpeeds(const frc::Pose2d &pose,
                                                    const pathplanner::PathPlannerTrajectoryState &target) override
    {
        return Require()->calculateRobotRelativeSpeeds(pose, target);
    }
    void reset(const frc::Pose2d &pose, const frc::ChassisSpeeds &speeds) override
    {
        Require()->reset(pose, speeds);
    }
    units::meter_t getPositionalError() override
    {
        return Require()->getPositionalError();
    }
    bool isHolonomic() override
    {
        return Require()->isHolonomic();
    }

  private:
    std::shared_ptr<pathplanner::PathFollowingController> Require() const
    {
        auto owner = owner_.lock();
        if (!owner)
            throw std::runtime_error("PathPlanner controller used after its Drive owner was destroyed");
        if (!*owner)
            throw std::runtime_error("PathPlanner controller is unavailable");
        return *owner;
    }
    std::weak_ptr<std::shared_ptr<pathplanner::PathFollowingController>> owner_;
};
} // namespace
void Drive::FieldDrive(double x, double y, double omega)
{
    if (!frc::DriverStation::IsTeleopEnabled())
    {
        Stop();
        return;
    }
    auto alliance = frc::DriverStation::GetAlliance();
    double sign = alliance && *alliance == frc::DriverStation::Alliance::kRed ? -1 : 1;
    const auto robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{x * sign}, units::meters_per_second_t{y * sign}, units::radians_per_second_t{omega},
        Pose().Rotation());
    // Operator commands update every 20 ms; ApplyRobotSpeeds does not discretize.
    RunVelocity(frc::ChassisSpeeds::Discretize(robotSpeeds, .02_s));
}
void Drive::RunHeading(double x, double y, frc::Rotation2d heading, double kp, double maxOmega)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading.Radians().value()) || !std::isfinite(kp) ||
        kp < 0 || !std::isfinite(maxOmega) || maxOmega < 0)
    {
        Stop();
        throw std::invalid_argument("RunHeading requires finite translation, heading and nonnegative gains/limits");
    }
    const double error = (heading - Pose().Rotation()).Radians().value();
    RunVelocity({units::meters_per_second_t{x}, units::meters_per_second_t{y},
                 units::radians_per_second_t{std::clamp(kp * error, -maxOmega, maxOmega)}});
}
void Drive::Stop()
{
    RunVelocity(frc::ChassisSpeeds{});
}
void Drive::StopWithX()
{
    if (MotorGainState() != DriveGainState::Ready) { requestedSpeeds_ = {}; return; }
    requestedSpeeds_ = {};
    if (native_)
        native_->SetControl(s::requests::SwerveDriveBrake{});
    else if (portable_)
        portable_->Brake();
    else
        for (size_t i = 0; i < 4; ++i)
            replayTargets_[i] = {0_mps, frc::Rotation2d{units::radian_t{
                                            std::atan2(config_.modules[i].yMeters, config_.modules[i].xMeters)}}};
    replay::RecordOutput("Drive/Brake", true);
    RecordDriveOutput();
}
void Drive::ConfigureAuto()
{
    std::vector<frc::Translation2d> locations;
    for (const auto &m : config_.modules)
        locations.emplace_back(units::meter_t{m.xMeters}, units::meter_t{m.yMeters});
    pathplanner::RobotConfig rc{
        units::kilogram_t{config_.massKg}, units::kilogram_square_meter_t{config_.moi},
        pathplanner::ModuleConfig{
            units::meter_t{config_.wheelRadius}, units::meters_per_second_t{config_.maxSpeed}, config_.cof,
            (config_.backend == DriveBackend::SparkMaxNavX ? frc::DCMotor::NEO(1) : frc::DCMotor::KrakenX60FOC(1))
                .WithReduction(config_.driveRatio),
            units::ampere_t{config_.driveCurrent}, 1},
        locations};
    if (config_.autoConfigFromGUI)
        rc = pathplanner::RobotConfig::fromGUISettings();
    autoController_ = std::make_shared<std::shared_ptr<pathplanner::PathFollowingController>>(
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants{config_.autoTranslationKP, 0, 0},
            pathplanner::PIDConstants{config_.autoRotationKP, 0, 0}));
    pathplanner::AutoBuilder::configure(
        [this] { return Pose(); }, [this](const auto &pose) { ResetPose(pose); }, [this] { return Speeds(); },
        [this](const frc::ChassisSpeeds &speed, const pathplanner::DriveFeedforwards &ff) { RunVelocity(speed, ff); },
        std::make_shared<BorrowedPathController>(autoController_), rc,
        [] {
            auto a = frc::DriverStation::GetAlliance();
            return a && *a == frc::DriverStation::Alliance::kRed;
        },
        this);
}
void Drive::SetAutoGains(double translationP, double rotationP)
{
    if (!std::isfinite(translationP) || translationP < 0 || !std::isfinite(rotationP) || rotationP < 0)
        throw std::invalid_argument("Auto gains must be finite and nonnegative");
    if (translationP == config_.autoTranslationKP && rotationP == config_.autoRotationKP)
        return;
    if (autoController_)
    {
        auto controller = std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants{translationP, 0, 0}, pathplanner::PIDConstants{rotationP, 0, 0});
        controller->reset(Pose(), Speeds());
        *autoController_ = std::move(controller);
    }
    config_.autoTranslationKP = translationP;
    config_.autoRotationKP = rotationP;
    replay::RecordOutput("Drive/AutoGains", {{"translationP", translationP}, {"rotationP", rotationP}});
}
static double Deadband(double value)
{
    return std::abs(value) < .1 ? 0 : std::copysign((std::abs(value) - .1) / .9, value);
}
frc2::CommandPtr Drive::Joystick(std::function<double()> x, std::function<double()> y, std::function<double()> omega)
{
    return frc2::cmd::RunEnd(
        [=, this] {
            FieldDrive(Deadband(x()) * config_.maxSpeed, Deadband(y()) * config_.maxSpeed,
                       Deadband(omega()) * config_.maxAngularRate);
        },
        [this] { Stop(); }, {this});
}
frc2::CommandPtr Drive::Aim(std::function<double()> x, std::function<double()> y,
                            std::function<frc::Rotation2d()> heading)
{
    return frc2::cmd::RunEnd(
        [=, this] {
            double error = (heading() - Pose().Rotation()).Radians().value();
            FieldDrive(Deadband(x()) * config_.maxSpeed, Deadband(y()) * config_.maxSpeed,
                       std::clamp(5 * error, -config_.maxAngularRate, config_.maxAngularRate));
        },
        [this] { Stop(); }, {this});
}
} // namespace base
