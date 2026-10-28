#pragma once
#include <functional>
#include <map>
#include <optional>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Requirements.h>
#include <string>
#include <vector>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

namespace base
{
  struct CheckEvaluation
  {
    bool passed = false;
    std::string explanation;
    std::map<std::string, double> measurements;
  };
  struct CheckStage
  {
    std::string name;
    double seconds = 3;
    std::function<void()> action;
    std::function<CheckEvaluation()> evaluate;
    std::vector<std::string> prerequisites;
    double stableSeconds = .25;
  };
  struct SystemCheckOptions
  {
    double startingVoltage = 11.5;
    double runningVoltage = 9.5;
    double countdownSeconds = 3;
    std::string setupPrompt = "Confirm the robot is safe for testing";
  };
  // Dashboard operator explicitly arms and confirms setup; test mode and safety interlocks
  // remain enforced during countdown and every active stage. Call Cancel() at mode transitions.
  // Controlled subsystems and callback captures must outlive this object; destruction
  // aborts and neutralizes an active check.
  // Outputs are neutralized only after subsystem ownership is acquired, and when
  // an active check ends or is cancelled.
  // Neutralization callbacks should not throw. If one does, the check releases
  // ownership and reports failure; it cannot guarantee that outputs stopped.
  class SystemCheck final : public frc2::SubsystemBase
  {
  public:
    explicit SystemCheck(std::function<void()> neutralize,
                         std::string reportPath = "/home/lvuser/systemcheck-latest.json",
                         SystemCheckOptions options = {});
    ~SystemCheck() override;
    void ConfigureRequirements(frc2::Requirements requirements);
    void AddStage(CheckStage stage);
    bool Start();
    void Cancel(std::string reason = "Operator cancelled");
    void Periodic() override;
    bool Active() const { return active_; }
    std::string ReportJson() const;

  private:
    struct Result
    {
      std::string name, status = "NOT_RUN", explanation;
      double elapsed = 0;
      std::map<std::string, double> measurements;
    };
    std::string Interlock(bool starting) const;
    void Advance();
    void SaveReport() const;
    std::function<void()> neutralize_;
    std::string reportPath_, state_ = "DISARMED", reason_;
    SystemCheckOptions options_;
    std::vector<CheckStage> stages_;
    std::vector<Result> results_;
    frc::Timer runTimer_, stageTimer_;
    int stage_ = -1;
    double stableSince_ = -1;
    bool active_ = false, countdown_ = false;
    std::optional<frc2::CommandPtr> reservation_;
    bool reservationEnding_ = false;
  };
}
