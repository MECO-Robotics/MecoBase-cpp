# MECO Base C++

Reusable WPILib 2026 C++ libraries for MECO Robotics. This repository builds and validates the `MecoBase` vendordep; season-specific robot entry points, hardware maps, autos, and game behavior belong in a consuming robot project such as [2026-Rebuilt-cpp](https://github.com/MECO-Robotics/2026-Rebuilt-cpp).

## Install

In a WPILib robot project, choose **Manage Vendor Libraries → Install new libraries (online)** and add [`MecoBase.json`](https://meco-robotics.github.io/MecoBase-cpp/MecoBase.json). MECO Base contains robot framework code and remains usable without a physics simulation package. This checkout prepares the split MecoBase 2026.2.0 artifact, which removes Jolt; the currently published online descriptor is still 2026.1.0 and includes Jolt. Install the matching-year [MECO Simulation vendordep](https://github.com/MECO-Robotics/MecoSimulation-cpp) for desktop physics after both split artifacts are published.

The public API uses headers such as `base/Drive.h` and `base/Mechanism.h`. The consuming robot owns its `Robot` and `RobotContainer` entry points.

## Build and validate

Install WPILib 2026.2.1 with its C++ toolchains and Java 17 or newer. From the repository root:

```sh
./gradlew build --max-workers=2
./gradlew check --max-workers=2
```

`build` compiles the static library for roboRIO and desktop targets. `check` runs the desktop C++ tests. The package workflow builds Linux, roboRIO, and Windows archives, assembles a local Maven repository, and links clean roboRIO/Linux/Windows consumer projects. It publishes to GitHub Pages only when manually dispatched from a reviewed `v*` release tag with `publish` enabled. Update `vendor/MecoBase.json` and the artifact version together.

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

Keep hardware access in IO adapters. Subsystems own goals and safety; adapters own vendor calls and device units. Keep units explicit: drive geometry uses metres and radians; mechanism coordinates use configured output rotations or metres. Follow the focused test guidance in [`AGENTS.md`](AGENTS.md).

## Documentation

- [Mechanism control](docs/mechanism-control.md) and [safe tuning](docs/mechanism-tuning.md)
- [Portable swerve](docs/portable-swerve.md) and [wheel-radius characterization](docs/wheel-radius-characterization.md)
- [Vision configuration](docs/vision-configuration.md), [QuestNav](docs/questnav.md), and [piece detection](docs/piece-detection.md)
- [Replay boundaries and workflow](docs/replay.md) and [AdvantageScope topics](docs/advantagescope.md)
- [Optional simulation setup and limits](docs/simulation.md) and [battery model](docs/simulation-power.md)

For C++ IntelliSense, run `./gradlew generateCompileCommands` and select a database under `build/TargetedCompileCommands/`.
