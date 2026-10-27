#include "base/ModelFeedforward.h"
#include <gtest/gtest.h>
#include <numbers>
#include <limits>
TEST(ModelFeedforward, MatchesMotorVoltageInAngularAndLinearCoordinates)
{
    auto motor = frc::DCMotor::Falcon500(2);
    constexpr double gear = 8, inertia = .04, radius = .03, omega = 3, alpha = 2;
    auto angular = base::ModelFeedforward::Motor(motor, units::kilogram_square_meter_t{inertia}, gear);
    const double expected =
        motor.Voltage(units::newton_meter_t{inertia * alpha / gear}, units::radians_per_second_t{omega * gear}).value();
    EXPECT_NEAR(angular.v * omega / (2 * std::numbers::pi) + angular.a * alpha / (2 * std::numbers::pi), expected,
                1e-12);
    auto linear = base::ModelFeedforward::Elevator(motor, units::kilogram_square_meter_t{inertia}, gear,
                                                   units::kilogram_t{4}, units::meter_t{radius});
    EXPECT_NEAR(linear.v * omega * radius + linear.a * alpha * radius, expected, 1e-12);
    EXPECT_NEAR(
        linear.gravity,
        motor.Voltage(units::newton_meter_t{4 * 9.80665 * radius / gear}, units::radians_per_second_t{0}).value(),
        1e-12);
    auto arm = base::ModelFeedforward::Arm(motor, units::kilogram_square_meter_t{inertia}, gear, units::kilogram_t{4},
                                           units::meter_t{.2});
    EXPECT_NEAR(arm.gravity,
                motor.Voltage(units::newton_meter_t{4 * 9.80665 * .2 / gear}, units::radians_per_second_t{0}).value(),
                1e-12);
    base::Gains gains;
    gains.p = 7;
    gains.s = .3;
    gains.maximum = 5;
    auto applied = linear.ApplyTo(gains);
    EXPECT_EQ(applied.p, 7);
    EXPECT_EQ(applied.s, .3);
    EXPECT_EQ(applied.maximum, 5);
    EXPECT_EQ(applied.v, linear.v);
}
TEST(ModelFeedforward, RejectsInvalidPhysicalParameters)
{
    auto motor = frc::DCMotor::Falcon500(1);
    EXPECT_THROW(base::ModelFeedforward::Motor(motor, units::kilogram_square_meter_t{0}, 2), std::invalid_argument);
    EXPECT_THROW(base::ModelFeedforward::Motor(motor, units::kilogram_square_meter_t{1}, 2,
                                               std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_THROW(base::ModelFeedforward::Elevator(motor, units::kilogram_square_meter_t{1}, 2, units::kilogram_t{1},
                                                  units::meter_t{-1}),
                 std::invalid_argument);
}
