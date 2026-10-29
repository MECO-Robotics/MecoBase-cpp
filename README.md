# MECO Base C++

Reusable WPILib 2026 C++ libraries for MECO Robotics. This repository builds and validates the `MecoBase` vendordep; season-specific robot entry points, hardware maps, autos, and game behavior belong in a consuming robot project such as [2026-Rebuilt-cpp](https://github.com/MECO-Robotics/2026-Rebuilt-cpp).

## Install

In a WPILib robot project, choose **Manage Vendor Libraries → Install new libraries (online)** and add [`MecoBase.json`](https://meco-robotics.github.io/MecoBase-cpp/MecoBase.json). MECO Base contains robot framework code and remains usable without a physics simulation package. Install the matching-year [MECO Simulation vendordep](https://github.com/MECO-Robotics/MecoSimulation-cpp) for desktop physics.

MecoBase 2026.2.1 adds per-module simulation device mounting directions, separate from electrical motor inversion. The public API uses headers such as `base/Drive.h` and `base/Mechanism.h`. The consuming robot owns its `Robot` and `RobotContainer` entry points.

Version 2026.2.3 removes redundant test forwarding functions while retaining the existing safety and behavior checks. Public APIs and robot behavior are unchanged.

The 2026.2.9 release candidate adds individual logged live tuning for mechanisms and Phoenix or Spark/navX swerve. Version 2026.2.8 adds reusable terrain-region mapping. Version 2026.2.7 adds optional footprint planning and moving-target replanning. Version 2026.2.6 stops native planning when the navgrid is invalid. Version 2026.2.5 adds native AutoDrive and object perception; 2026.2.4 added reusable Choreo trajectory following and an operator-armed system check runner.

## Build and validate

Install WPILib 2026.2.1 with its C++ toolchains and Java 17 or newer. For development on Linux, build and run desktop Debug tests from the repository root:

```sh
./gradlew runMecoBaseTestLinuxx86-64DebugGoogleTestExe -Ptoolchain-optional-roboRio --max-workers=2
```

After building, use the installed runner for focused checks without invoking Gradle again:

```sh
build/install/mecoBaseTest/linuxx86-64/debug/mecoBaseTest --gtest_filter='DriveTarget.*'
```

The runner sets the shared-library search path. Rebuild after source changes, and choose the relevant GoogleTest suite or case. `./gradlew build --max-workers=2` remains the broader local desktop/roboRIO check. Public headers use ordinary includes; the unused DriveTarget named-module experiment and its custom BMI build were removed.

Pull requests run desktop Debug tests. Main-branch and manually dispatched integration builds produce Linux, roboRIO, and Windows archives, assemble a local Maven repository, and link clean consumer projects. GitHub Pages publication requires manual dispatch from a reviewed `v*` release tag with `publish` enabled. Published Maven versions are retained when the site is updated. Update `vendor/MecoBase.json` and the artifact version together.

Run standalone physics model checks from the matching-year [MECO Simulation repository](https://github.com/MECO-Robotics/MecoSimulation-cpp):

```sh
git clone --branch wpilib-2026 https://github.com/MECO-Robotics/MecoSimulation-cpp.git
cmake -S MecoSimulation-cpp -B MecoSimulation-cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build MecoSimulation-cpp/build --parallel 2
ctest --test-dir MecoSimulation-cpp/build --output-on-failure
```

A successful desktop build or simulation test does not establish hardware calibration or commissioning.

## Modules

Each module groups implementation under `src/main/cpp/base/` and exposes its reusable API under `src/main/include/base/`. Start with the public header before changing behavior.

| Module | Use it for | Start here |
| --- | --- | --- |
| `drive/` | Swerve control, drive commands, hardware adapters | [`Drive.h`](src/main/include/base/Drive.h), [`source`](src/main/cpp/base/drive/) |
| `mechanism/` | Flywheels, joints, motor IO, mechanism tuning | [`Mechanism.h`](src/main/include/base/Mechanism.h), [`source`](src/main/cpp/base/mechanism/) |
| `vision/` | Pose cameras, QuestNav, game-piece detection | [`Vision.h`](src/main/include/base/Vision.h), [`source`](src/main/cpp/base/vision/) |
| `logging/` | WPILOG, input recording, replay | [`Replay.h`](src/main/include/base/Replay.h), [`source`](src/main/cpp/base/logging/) |
| `telemetry/` | NetworkTables and AdvantageScope publishing | [`Telemetry.h`](src/main/include/base/Telemetry.h), [`source`](src/main/cpp/base/telemetry/) |
| Simulation (optional external package) | Year-specific desktop physics, arena, and WPILib integration | [MECO Simulation](https://github.com/MECO-Robotics/MecoSimulation-cpp) |

For Choreo trajectories, install ChoreoLib separately and include `base/ChoreoAuto.h`; `base::FollowChoreo<2026>` accepts the drive, trajectory name, controller gains, and period. The command resets odometry at startup and stops the drive on completion or interruption.

`base/SystemCheck.h` provides an operator-armed diagnostic runner. Supply subsystem requirements, neutralization, stages, and a setup prompt appropriate to the robot.

Keep hardware access in IO adapters. Subsystems own goals and safety; adapters own vendor calls and device units. Keep units explicit: drive geometry uses metres and radians; mechanism coordinates use configured output rotations or metres. Follow the focused test guidance in [`AGENTS.md`](AGENTS.md).

## Documentation

- [Mechanism control](docs/mechanism-control.md) and [safe tuning](docs/mechanism-tuning.md)
- [Portable swerve](docs/portable-swerve.md) and [wheel-radius characterization](docs/wheel-radius-characterization.md)
- [Vision configuration](docs/vision-configuration.md), [QuestNav](docs/questnav.md), and [piece detection](docs/piece-detection.md)
- [Replay boundaries and workflow](docs/replay.md) and [AdvantageScope topics](docs/advantagescope.md)
- [Optional simulation setup and limits](docs/simulation.md) and [battery model](docs/simulation-power.md)

For C++ IntelliSense, run `./gradlew generateCompileCommands` and select a database under `build/TargetedCompileCommands/`.

Native goal driving on roboRIO and desktop is provided by [`base/AutoDrive.h`](src/main/include/base/AutoDrive.h); see [AutoDrive setup and ownership](docs/autodrive.md). No desktop companion is required.
