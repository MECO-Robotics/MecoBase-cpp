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
