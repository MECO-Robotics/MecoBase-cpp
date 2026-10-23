#include "MechanismHardwareInternal.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#ifndef __FRC_ROBORIO__
#include "base/SimulationRuntime.h"
#endif

namespace base::detail
{
namespace
{
// Output-shaft radians internally, output rotations at the MotorIO boundary.
class SimMotorIO final : public MotorIO
{
  public:
    SimMotorIO(MotorConfig config, Vendor vendor, MotorSimulationConfig model)
        : config_(std::move(config)), model_(model), inertia_(model.outputInertia.value()),
          resistance_(vendor == Vendor::SparkMax ? 12. / 105 : 12. / 366),
          kt_(vendor == Vendor::SparkMax ? 2.6 / 105 : 7.09 / 366),
          kv_(vendor == Vendor::SparkMax ? 5676 * 2 * std::numbers::pi / 60 / 12
                                         : 6000 * 2 * std::numbers::pi / 60 / 12),
          position_(model.initialPosition.value())
    {
        model_.Validate();
        if (model_.load == MotorSimulationConfig::Load::Elevator)
            inertia_ += model_.mass.value() * std::pow(model_.drumRadius.value(), 2);
#ifndef __FRC_ROBORIO__
        registration_ =
            simulation::GetSimulationRuntime().Register({[this](double dt) {
                                                      busVoltage_ = simulation::GetSimulationRuntime().BusVoltage();
                                                      StepSimulation(units::second_t{dt});
                                                  },
                                                  {},
                                                  {},
                                                  [this] {
                                                      position_ = model_.initialPosition.value();
                                                      velocity_ = voltage_ = requestedVoltage_ = current_ = 0;
                                                      busVoltage_ = 12;
                                                  },
                                                  [this] { return SupplyCurrent(); }});
#endif
    }
    MotorInputs Read() override
    {
        return {position_ / (2 * std::numbers::pi),
                velocity_ / (2 * std::numbers::pi),
                voltage_,
                busVoltage_,
                SupplyCurrent(),
                true};
    }
    void StepSimulation(units::second_t duration) override
    {
        const double dt = duration.value();
        if (!std::isfinite(dt) || dt < 0 || dt > 1)
            throw std::invalid_argument("Simulation step must be finite and between zero and one second");
        // Bound nonlinear gravity error and stop penetration independently of caller cadence.
        const int steps = static_cast<int>(std::ceil(dt / .001));
        for (int n = 0; n < steps; ++n)
            Advance(dt / steps);
    }
    void SetVoltage(double volts) override
    {
        if (!std::isfinite(volts))
            throw std::invalid_argument("Nonfinite simulated motor voltage");
        requestedVoltage_ = volts;
    }
    void SetBrake(bool enabled) override
    {
        brake_ = enabled;
    }

  private:
    double SupplyCurrent() const
    {
        return busVoltage_ > 0 ? std::max(0., voltage_ * current_) * config_.ids.size() / busVoltage_ : 0;
    }
    void Advance(double dt)
    {
        voltage_ = std::clamp(requestedVoltage_, -busVoltage_, busVoltage_);
        const double drag = .0001 / inertia_;
        const double torque = kt_ * config_.ids.size() * config_.gearRatio / inertia_;
        const double emf = config_.gearRatio / kv_;
        const double limit = config_.currentLimit;
        double load = 0;
        if (model_.load == MotorSimulationConfig::Load::Arm)
            load = -model_.mass.value() * 9.80665 * model_.armCOM.value() *
                   std::cos(position_ + model_.armAngleOffset.value()) / inertia_;
        else if (model_.load == MotorSimulationConfig::Load::Elevator)
            load = -model_.mass.value() * 9.80665 * model_.drumRadius.value() / inertia_;
        auto advance = [&](double duration, double decay, double equilibrium) {
            const double integral = -std::expm1(-decay * duration) / decay;
            position_ += equilibrium * duration + (velocity_ - equilibrium) * integral;
            velocity_ += (equilibrium - velocity_) * (-std::expm1(-decay * duration));
        };
        if (!brake_ && std::abs(voltage_) < 1e-9)
        {
            advance(dt, drag, load / drag);
            current_ = 0;
        }
        else
        {
            double remaining = dt;
            // Split at every electrical region crossing. Gravity can drive a motor
            // through both current limits, so one crossing is not sufficient.
            for (int region = 0; remaining > 1e-14 && region < 4; ++region)
            {
                const double raw = (voltage_ - emf * velocity_) / resistance_;
                const double linearDecay = torque * emf / resistance_ + drag;
                const double linearEquilibrium = (torque * voltage_ / resistance_ + load) / linearDecay;
                double decay = linearDecay, equilibrium = linearEquilibrium;
                double boundary = 0;
                bool crossingPossible = false;
                const double acceleration = torque * std::clamp(raw, -limit, limit) + load - drag * velocity_;
                const bool positiveLimited = raw > limit + 1e-9 || (raw >= limit - 1e-9 && acceleration < 0);
                const bool negativeLimited = raw < -limit - 1e-9 || (raw <= -limit + 1e-9 && acceleration > 0);
                if (positiveLimited || negativeLimited)
                {
                    const double current = positiveLimited ? limit : -limit;
                    decay = drag;
                    equilibrium = (torque * current + load) / drag;
                    boundary = (voltage_ - resistance_ * current) / emf;
                    crossingPossible = (boundary - velocity_) * (equilibrium - boundary) > 0;
                }
                else
                {
                    const double low = (voltage_ - resistance_ * limit) / emf;
                    const double high = (voltage_ + resistance_ * limit) / emf;
                    boundary = linearEquilibrium < low ? low : high;
                    crossingPossible = linearEquilibrium < low || linearEquilibrium > high;
                }
                double segment = remaining;
                if (crossingPossible && std::abs(equilibrium - velocity_) > 1e-14)
                {
                    const double fraction = (boundary - velocity_) / (equilibrium - velocity_);
                    if (fraction > 1e-12 && fraction < 1)
                        segment = std::min(remaining, -std::log1p(-fraction) / decay);
                }
                advance(segment, decay, equilibrium);
                remaining -= segment;
            }
            current_ = std::clamp((voltage_ - emf * velocity_) / resistance_, -limit, limit);
        }
        if (model_.minimum && position_ < model_.minimum->value())
        {
            position_ = model_.minimum->value();
            velocity_ = std::max(0., velocity_);
        }
        if (model_.maximum && position_ > model_.maximum->value())
        {
            position_ = model_.maximum->value();
            velocity_ = std::min(0., velocity_);
        }
        // Stops constrain the shaft before reporting its electrical state: a
        // powered motor held against a stop draws stall/current-limited current.
        current_ = !brake_ && std::abs(voltage_) < 1e-9
                       ? 0
                       : std::clamp((voltage_ - emf * velocity_) / resistance_, -limit, limit);
        voltage_ = std::clamp(resistance_ * current_ + emf * velocity_, -std::abs(voltage_), std::abs(voltage_));
    }
    MotorConfig config_;
    MotorSimulationConfig model_;
    double inertia_, resistance_, kt_, kv_, position_, velocity_ = 0, voltage_ = 0, current_ = 0, requestedVoltage_ = 0,
                                                       busVoltage_ = 12;
    bool brake_ = true;
#ifndef __FRC_ROBORIO__
    // Destroy registration before any captured state.
    simulation::Registration registration_;
#endif
};

} // namespace

std::unique_ptr<MotorIO> MakeSimMotorIO(const MotorConfig &config, Vendor vendor,
                                        const MotorSimulationConfig &simulation)
{
    return std::make_unique<SimMotorIO>(config, vendor, simulation);
}

} // namespace base::detail
