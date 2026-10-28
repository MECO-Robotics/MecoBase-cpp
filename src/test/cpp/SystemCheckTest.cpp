#include "base/SystemCheck.h"
#include <gtest/gtest.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HAL.h>
#include <stdexcept>

namespace {
void ConfigureDS(bool enabled) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetFmsAttached(false);
  frc::sim::DriverStationSim::SetEStop(false);
  frc::sim::DriverStationSim::SetTest(true);
  frc::sim::DriverStationSim::SetEnabled(enabled);
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12});
  frc::sim::DriverStationSim::NotifyNewData();
}
}
TEST(SystemCheckSafety, RefusesDisabledAndDoesNotRunActions) {
  ConfigureDS(false);
  int actions=0,stops=0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  base::SystemCheck check([&]{++stops;},"/tmp/frc-systemcheck-disabled.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock",1,[&]{++actions;},[]{return base::CheckEvaluation{true,"ok",{}};}});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed",true);
  EXPECT_FALSE(check.Start());
  EXPECT_EQ(actions,0);
}
TEST(SystemCheckSafety, OwnershipInterruptionCancelsDuringCountdown) {
  ConfigureDS(true);
  int actions=0,stops=0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  base::SystemCheck check([&]{++stops;},"/tmp/frc-systemcheck-interrupted.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock",1,[&]{++actions;},[]{return base::CheckEvaluation{true,"ok",{}};}});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed",true);
  ASSERT_TRUE(check.Start());
  EXPECT_TRUE(check.Active());
  EXPECT_EQ(actions,0);
  auto conflicting=frc2::cmd::Run([]{}, {&mechanism});
  conflicting.Schedule();
  EXPECT_FALSE(check.Active());
  EXPECT_EQ(actions,0);
  EXPECT_GE(stops,2);
  EXPECT_NE(check.ReportJson().find("ABORTED"),std::string::npos);
  conflicting.Cancel();
  ConfigureDS(false);
}
TEST(SystemCheckSafety, UsesConfiguredStartingVoltage) {
  ConfigureDS(true);
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  base::SystemCheck check([] {}, "/tmp/frc-systemcheck-voltage.json",
                          {.startingVoltage = 13});
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock", 1, [] {}, [] { return base::CheckEvaluation{true, "ok", {}}; }});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
  EXPECT_FALSE(check.Start());
  EXPECT_NE(check.ReportJson().find("Battery voltage too low"), std::string::npos);
  ConfigureDS(false);
}
TEST(SystemCheckSafety, AbortOverridesSimultaneousStart) {
  ConfigureDS(true);
  int actions = 0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  base::SystemCheck check([] {}, "/tmp/frc-systemcheck-abort.json",
                          {.countdownSeconds = 0});
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock", 1, [&] { ++actions; },
                  [] { return base::CheckEvaluation{true, "ok", {}}; }});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/Start", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/Abort", true);
  check.Periodic();
  check.Periodic();
  EXPECT_FALSE(check.Active());
  EXPECT_EQ(actions, 0);
  EXPECT_FALSE(frc::SmartDashboard::GetBoolean("SystemCheck/Start", true));
  ConfigureDS(false);
}
TEST(SystemCheckSafety, DestructionNeutralizesAnActiveCheck) {
  ConfigureDS(true);
  int stops = 0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  {
    base::SystemCheck check([&] { ++stops; }, "/tmp/frc-systemcheck-destroyed.json");
    check.ConfigureRequirements({&mechanism});
    check.AddStage({"mock", 1, [] {}, [] { return base::CheckEvaluation{true, "ok", {}}; }});
    frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
    frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
    ASSERT_TRUE(check.Start());
    stops = 0;
  }
  EXPECT_GT(stops, 0);
  ConfigureDS(false);
}
TEST(SystemCheckSafety, RejectedReservationDoesNotTouchOwnedSubsystem) {
  ConfigureDS(true);
  int stops = 0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  auto owner = frc2::cmd::Run([] {}, {&mechanism})
      .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
  owner.Schedule();
  ASSERT_TRUE(owner.IsScheduled());
  base::SystemCheck check([&] { ++stops; }, "/tmp/frc-systemcheck-busy.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock", 1, [] {}, [] { return base::CheckEvaluation{true, "ok", {}}; }});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
  EXPECT_FALSE(check.Start());
  EXPECT_FALSE(check.Active());
  EXPECT_TRUE(owner.IsScheduled());
  EXPECT_EQ(stops, 0);
  EXPECT_NE(check.ReportJson().find("Unable to reserve"), std::string::npos);
  check.Cancel();
  frc::SmartDashboard::PutBoolean("SystemCheck/Abort", true);
  check.Periodic();
  EXPECT_TRUE(owner.IsScheduled());
  EXPECT_EQ(stops, 0);
  owner.Cancel();
  ConfigureDS(false);
}
TEST(SystemCheckSafety, NeutralizationFailureReleasesReservation) {
  ConfigureDS(true);
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  bool fail = true;
  base::SystemCheck check([&] { if (fail) throw std::runtime_error("stop failed"); },
                          "/tmp/frc-systemcheck-neutralize-error.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock", 1, [] {}, [] { return base::CheckEvaluation{true, "ok", {}}; }});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
  EXPECT_FALSE(check.Start());
  EXPECT_FALSE(check.Active());
  EXPECT_NE(check.ReportJson().find("neutralization failed"), std::string::npos);
  auto owner = frc2::cmd::Run([] {}, {&mechanism})
      .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
  owner.Schedule();
  EXPECT_TRUE(owner.IsScheduled());
  owner.Cancel();
  fail = false;
  ASSERT_TRUE(check.Start());
  fail = true;
  check.Cancel();
  EXPECT_FALSE(check.Active());
  EXPECT_NE(check.ReportJson().find("neutralization failed"), std::string::npos);
  owner.Schedule();
  EXPECT_TRUE(owner.IsScheduled());
  owner.Cancel();
  {
    fail = false;
    base::SystemCheck destroying([&] { if (fail) throw std::runtime_error("stop failed"); },
                                 "/tmp/frc-systemcheck-destroy-error.json");
    destroying.ConfigureRequirements({&mechanism});
    destroying.AddStage({"mock", 1, [] {}, [] { return base::CheckEvaluation{true, "ok", {}}; }});
    frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
    frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
    ASSERT_TRUE(destroying.Start());
    fail = true;
  }
  owner.Schedule();
  EXPECT_TRUE(owner.IsScheduled());
  owner.Cancel();
  ConfigureDS(false);
}
TEST(SystemCheckSafety, UnknownStageExceptionAbortsAndReleasesReservation) {
  ConfigureDS(true);
  int stops = 0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  base::SystemCheck check([&] { ++stops; }, "/tmp/frc-systemcheck-stage-error.json",
                          {.countdownSeconds = 0});
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock", 1, [] { throw 42; },
                  [] { return base::CheckEvaluation{true, "ok", {}}; }});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed", true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed", true);
  ASSERT_TRUE(check.Start());
  check.Periodic();  // Advance out of the countdown.
  stops = 0;
  check.Periodic();  // The action throws.
  EXPECT_FALSE(check.Active());
  EXPECT_GT(stops, 0);
  EXPECT_NE(check.ReportJson().find("unknown exception"), std::string::npos);
  auto owner = frc2::cmd::Run([] {}, {&mechanism});
  owner.Schedule();
  EXPECT_TRUE(owner.IsScheduled());
  owner.Cancel();
  ConfigureDS(false);
}
