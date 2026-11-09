#include "base/Drive.h"
#include "base/RuntimeTuning.h"
#include <gtest/gtest.h>
#include <limits>

TEST(DriveConfig, RejectsUnsafeConfiguration)
{
    base::DriveConfig good;
    EXPECT_NO_THROW(good.Validate());
    good.controlPeriodSeconds = .001;
    EXPECT_NO_THROW(good.Validate());
    good.controlPeriodSeconds = .02;
    EXPECT_NO_THROW(good.Validate());
    // Different CAN device types may use the same numeric address.
    good.modules[0].encoder = good.modules[0].drive;
    good.pigeon = good.modules[0].drive;
    good.bus = ""; // Phoenix default-bus alias is supported.
    EXPECT_NO_THROW(good.Validate());
    for (int fault = 0; fault < 12; ++fault)
    {
        base::DriveConfig bad;
        switch (fault)
        {
        case 0:
            bad.driveRatio = 0;
            break;
        case 1:
            bad.simulation.suspensionDamping = std::numeric_limits<double>::quiet_NaN();
            break;
        case 2:
            bad.modules[1].steer = bad.modules[0].drive;
            break;
        case 3:
            bad.modules[1].encoder = bad.modules[0].encoder;
            break;
        case 4:
            for (auto &m : bad.modules)
                m.yMeters = 0;
            break;
        case 5:
            bad.pigeon = 63;
            break;
        case 6:
            bad.odometryHz = -1;
            break;
        case 7:
            bad.autoTranslationKP = -1;
            break;
        case 8:
            bad.autoRotationKP = std::numeric_limits<double>::infinity();
            break;
        case 9:
            bad.odometryHz = std::numeric_limits<double>::quiet_NaN();
            break;
        case 10:
            bad.controlPeriodSeconds = .0009;
            break;
        case 11:
            bad.controlPeriodSeconds = .0201;
            break;
        }
        EXPECT_THROW(base::Drive{bad}, std::invalid_argument) << fault;
    }
}

TEST(DriveConfig, RuntimeTuningKeepsPlantAndConfiguredGainsSeparate)
{
    base::DriveConfig initial;
    auto configured = base::ConfigureRuntimeDrive(initial, [](base::DriveConfig config) {
        config.massKg = 42;
        return config;
    });
    EXPECT_DOUBLE_EQ(configured.massKg, 42);
    EXPECT_DOUBLE_EQ(configured.autoTranslationKP, initial.autoTranslationKP);
    base::Gains gains;
    gains.gravity = 1.5;
    EXPECT_DOUBLE_EQ(base::ConfigureRuntimeJointGains(gains, base::SimulationTuning::Automatic,
        base::SimulationJointGravity::Unmodeled).gravity, 0);
    EXPECT_DOUBLE_EQ(base::ConfigureRuntimeJointGains(gains, base::SimulationTuning::Configured,
        base::SimulationJointGravity::Unmodeled).gravity, 1.5);
    EXPECT_DOUBLE_EQ(base::ConfigureRuntimeJointGains(gains, base::SimulationTuning::Automatic,
        base::SimulationJointGravity::Modeled).gravity, 1.5);
}

TEST(DriveConfig, ValidatesExplicitDriveSimulationProfiles)
{
    base::DriveConfig unconfigured;
    EXPECT_NO_THROW(unconfigured.Validate());

    auto configured = unconfigured;
    configured.simulation.bumperCornerRadius = .04;
    configured.simulation.bumperFriction = .65;
    configured.simulation.wheelWidth = .05;
    configured.simulation.contactPatchLength = .02;
    configured.simulation.contactShearDeflection = .001;
    configured.simulation.driveMotor = {.resistanceOhms = .1,
                                        .torqueConstantNmPerAmp = .02,
                                        .velocityConstantRadPerSecPerVolt = 50,
                                        .currentLimitAmps = 60,
                                        .frictionVoltage = .1};
    configured.simulation.steerMotor = {.resistanceOhms = .2,
                                        .torqueConstantNmPerAmp = .03,
                                        .velocityConstantRadPerSecPerVolt = 40,
                                        .currentLimitAmps = 30,
                                        .frictionVoltage = 0};
    EXPECT_NO_THROW(configured.Validate());

    auto partialMotor = unconfigured;
    partialMotor.simulation.driveMotor.resistanceOhms = .1;
    EXPECT_THROW(partialMotor.Validate(), std::invalid_argument);

    auto partialContact = unconfigured;
    partialContact.simulation.wheelWidth = .05;
    EXPECT_THROW(partialContact.Validate(), std::invalid_argument);

    auto oversizedCorner = unconfigured;
    oversizedCorner.simulation.bumperCornerRadius = .36;
    EXPECT_THROW(oversizedCorner.Validate(), std::invalid_argument);

    auto invalidFriction = unconfigured;
    invalidFriction.simulation.bumperFriction = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(invalidFriction.Validate(), std::invalid_argument);
}
