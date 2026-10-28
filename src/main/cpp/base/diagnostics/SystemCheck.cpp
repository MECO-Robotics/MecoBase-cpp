#include "base/SystemCheck.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/json.h>
#include <frc2/command/Commands.h>

namespace base {
SystemCheck::SystemCheck(std::function<void()> neutralize,std::string reportPath,SystemCheckOptions options)
    : neutralize_(std::move(neutralize)),reportPath_(std::move(reportPath)),options_(std::move(options)) {
  if (!std::isfinite(options_.startingVoltage) || options_.startingVoltage < 0 ||
      !std::isfinite(options_.runningVoltage) || options_.runningVoltage < 0 ||
      !std::isfinite(options_.countdownSeconds) || options_.countdownSeconds < 0 || options_.setupPrompt.empty())
    throw std::invalid_argument("Invalid system check options");
  if(!neutralize_) throw std::invalid_argument("System check requires emergency neutralization");
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",false);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed",false);
  frc::SmartDashboard::PutBoolean("SystemCheck/Start",false);
  frc::SmartDashboard::PutBoolean("SystemCheck/Abort",false);
}
SystemCheck::~SystemCheck() {
  if(active_) Cancel("System check destroyed");
  if(reservation_ && !reservationEnding_) reservation_->Cancel();
}
void SystemCheck::ConfigureRequirements(frc2::Requirements requirements) {
  if(active_ || requirements.begin()==requirements.end())
    throw std::invalid_argument("Diagnostic requirements must include controlled subsystems");
  reservation_.emplace(frc2::cmd::RunEnd([]{},[this]{
    reservationEnding_=true;
    if(active_) Cancel("Diagnostic subsystem ownership interrupted");
    reservationEnding_=false;
  },requirements).WithName("SystemCheckReservation"));
}
void SystemCheck::AddStage(CheckStage stage) {
  if(active_ || stage.name.empty() || !stage.action || !stage.evaluate ||
     !std::isfinite(stage.seconds) || stage.seconds<=0 || !std::isfinite(stage.stableSeconds) || stage.stableSeconds<0 || stage.stableSeconds>stage.seconds)
    throw std::invalid_argument("Invalid diagnostic stage");
  if(std::any_of(stages_.begin(),stages_.end(),[&](const auto& other){return other.name==stage.name;}))
    throw std::invalid_argument("Duplicate diagnostic stage");
  stages_.push_back(std::move(stage));
}
std::string SystemCheck::Interlock(bool starting) const {
  if(!frc::DriverStation::IsDSAttached()) return "Driver Station disconnected";
  if(!frc::DriverStation::IsTestEnabled()) return "Enable Test mode";
  if(frc::DriverStation::IsFMSAttached()) return "FMS attached";
  if(frc::DriverStation::IsEStopped()) return "Emergency stopped";
  if(frc::RobotController::IsBrownedOut()) return "Brownout";
  double battery=frc::RobotController::GetBatteryVoltage().value();
  if(!std::isfinite(battery) || battery<(starting ? options_.startingVoltage : options_.runningVoltage)) return "Battery voltage too low";
  if(!frc::SmartDashboard::GetBoolean("SystemCheck/Armed",false)) return "Arm the check";
  if(!frc::SmartDashboard::GetBoolean("SystemCheck/SetupConfirmed",false)) return options_.setupPrompt;
  return {};
}
bool SystemCheck::Start() {
  if(active_) return false;
  reason_=Interlock(true);
  if(stages_.empty()) reason_="No diagnostic stages configured";
  if(!reservation_) reason_="No diagnostic subsystem requirements configured";
  if(!reason_.empty()) {state_="BLOCKED"; return false;}
  reservation_->Schedule();
  if(!reservation_->IsScheduled()) {
    reason_="Unable to reserve diagnostic subsystems"; state_="BLOCKED";
    return false;
  }
  neutralize_(); results_.clear();
  for(const auto& stage:stages_) results_.push_back({stage.name});
  stage_=-1; countdown_=true; active_=true; state_="COUNTDOWN";
  runTimer_.Restart(); stageTimer_.Restart();
  return true;
}
void SystemCheck::Cancel(std::string reason) {
  if(!active_) return;
  neutralize_();
  reason_=std::move(reason); state_="ABORTED"; active_=false;
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",false);
  if(stage_>=0 && stage_<static_cast<int>(results_.size())) {
    auto& result=results_[stage_]; result.status="ABORTED"; result.explanation=reason_; result.elapsed=stageTimer_.Get().value();
  }
  runTimer_.Stop(); stageTimer_.Stop();
  if(reservation_ && !reservationEnding_) reservation_->Cancel();
  SaveReport();
}
void SystemCheck::Advance() {
  neutralize_();
  while(++stage_<static_cast<int>(stages_.size())) {
    auto& result=results_[stage_]; auto& stage=stages_[stage_];
    bool allowed=std::all_of(stage.prerequisites.begin(),stage.prerequisites.end(),[&](const std::string& name){
      return std::any_of(results_.begin(),results_.begin()+stage_,[&](const Result& previous){return previous.name==name && previous.status=="PASS";});
    });
    if(!allowed) {result.status="SKIPPED";result.explanation="Prerequisite failed";continue;}
    result.status="RUNNING"; stageTimer_.Restart(); stableSince_=-1; return;
  }
  active_=false; state_="COMPLETE"; runTimer_.Stop(); stageTimer_.Stop();
  if(reservation_ && !reservationEnding_) reservation_->Cancel();
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",false); SaveReport();
}
void SystemCheck::Periodic() {
  if(frc::SmartDashboard::GetBoolean("SystemCheck/Abort",false)) {
    frc::SmartDashboard::PutBoolean("SystemCheck/Abort",false);
    // An abort takes precedence over any simultaneous or stale start request.
    frc::SmartDashboard::PutBoolean("SystemCheck/Start",false);
    Cancel();
  }
  if(frc::SmartDashboard::GetBoolean("SystemCheck/Start",false)) {
    frc::SmartDashboard::PutBoolean("SystemCheck/Start",false); Start();
  }
  if(active_) {
    auto failure=Interlock(false);
    if(!failure.empty()) Cancel(failure);
    else if(countdown_) {
      if(stageTimer_.Get().value()>=options_.countdownSeconds) {countdown_=false;state_="RUNNING";Advance();}
    } else {
      try {
        auto& stage=stages_.at(stage_); auto& result=results_.at(stage_);
        stage.action(); auto evaluation=stage.evaluate();
        result.elapsed=stageTimer_.Get().value(); result.measurements=evaluation.measurements;
        bool finite=std::all_of(result.measurements.begin(),result.measurements.end(),[](const auto& entry){return std::isfinite(entry.second);});
        if(!finite) {Cancel("Nonfinite diagnostic measurement");return;}
        if(evaluation.passed) {if(stableSince_<0)stableSince_=result.elapsed;}
        else stableSince_=-1;
        for(const auto& [name,value]:result.measurements) frc::SmartDashboard::PutNumber("SystemCheck/Live/"+name,value);
        if(result.elapsed>=stage.seconds) {
          result.status=evaluation.passed && stableSince_>=0 && result.elapsed-stableSince_>=stage.stableSeconds ? "PASS" : "FAIL";
          result.explanation=evaluation.explanation; Advance();
        }
      } catch(const std::exception& exception) {Cancel(exception.what());}
    }
  }
  frc::SmartDashboard::PutString("SystemCheck/State",state_);
  frc::SmartDashboard::PutString("SystemCheck/Reason",reason_);
  frc::SmartDashboard::PutString("SystemCheck/Stage",active_ && stage_>=0 ? stages_[stage_].name : "");
  frc::SmartDashboard::PutString("SystemCheck/ReportJSON",ReportJson());
}
std::string SystemCheck::ReportJson() const {
  wpi::json stages=wpi::json::array(); bool failed=state_=="ABORTED";
  for(const auto& result:results_) {
    failed|=result.status=="FAIL" || result.status=="ABORTED" || result.status=="SKIPPED";
    stages.push_back({{"name",result.name},{"status",result.status},{"explanation",result.explanation},{"elapsedSeconds",result.elapsed},{"measurements",result.measurements}});
  }
  return wpi::json{{"state",state_},{"reason",reason_},{"elapsedSeconds",runTimer_.Get().value()},
                   {"overallStatus",failed ? "FAIL" : state_=="COMPLETE" ? "PASS" : "NOT_RUN"},{"results",stages}}.dump(2);
}
void SystemCheck::SaveReport() const {
  std::ofstream output(reportPath_); if(output) output<<ReportJson()<<'\n';
  else FRC_ReportError(frc::warn::Warning, "Could not write system check report: {}", reportPath_);
}
}
