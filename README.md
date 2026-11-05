# MECO Base C++

Reusable WPILib 2026 C++ libraries for MECO Robotics. This repository builds and validates the `MecoBase` vendordep; season-specific robot entry points, hardware maps, autos, and game behavior belong in a consuming robot project such as [2026-Rebuilt-cpp](https://github.com/MECO-Robotics/2026-Rebuilt-cpp).

## Find a guide

| I want to… | Start here | More detail |
| --- | --- | --- |
| Drive to a target | [AutoDrive basics](docs/autodrive-basics.md) | [Command contract](docs/autodrive.md) and [footprint configuration](docs/footprint-configuration.md) |
| Control a motor or mechanism | [Mechanism quick start](docs/mechanism-control.md#quick-start) | [Motor adapters and safety](docs/mechanism-control.md), [tuning](docs/mechanism-tuning.md) |
| Add localization | [Localization quick start](docs/vision-configuration.md#quick-start) | [Vision configuration](docs/vision-configuration.md), [QuestNav](docs/questnav.md) |
| Run desktop physics | [Simulation overview](docs/simulation.md) | [MecoSimulation guides](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/simulation-basics.md) |
| Record or replay data | [Replay](docs/replay.md) | [AdvantageScope](docs/advantagescope.md) |

## Install

In a WPILib robot project, choose **Manage Vendor Libraries → Install new libraries (online)** and add [`MecoBase.json`](https://meco-robotics.github.io/MecoBase-cpp/MecoBase.json). MECO Base contains robot framework code and remains usable without a physics simulation package. Install the matching-year [MECO Simulation vendordep](https://github.com/MECO-Robotics/MecoSimulation-cpp) for desktop physics.

Version 2026.2.15 adds runtime tuning, driver input arbitration, and an optional lifecycle facade. Version 2026.2.14 adds runtime-selected motor, vision, and detection IO, replay initialization before IO construction, and CAN rate profile selection. Version 2026.2.13 adds the NT gamepad reader and footprint outcome names. Version 2026.2.12 adds configurable drive control timing and motor/CAN status rates, optional NetworkTables detection and QuestNav delivery periods, and externally scheduled desktop drive simulation. Version 2026.2.10 provides `base/season2026/MatchContext.h` for 2026 match phases and hub activity. It reads Driver Station data or accepts explicit inputs; unavailable alliance and match data remain unknown. Version 2026.2.9 adds a footprint configuration parser and a cross-platform simulation latency facade.

`base/SimulationLifecycle.h` provides the scheduler and scene hooks across platforms.
With MECO Simulation installed, desktop hooks run its lifecycle; without it and on
roboRIO they are no-ops. Replay remains guarded by the Simulation lifecycle.
Header availability selects the lifecycle type, so use the same Simulation
vendordep configuration for every translation unit that shares that type. A
Simulation package without `meco/sim/SimulationLifecycle.h` uses no-op hooks.
`base/SimulationPractice.h` similarly supplies an optional practice-match command
only in live desktop simulation. On roboRIO, replay, or desktop without the
Simulation vendordep, it returns no command. Use the same Simulation vendordep
configuration across translation units that include this header.

`base/RuntimeScene.h` provides an optional, platform-neutral scene lifecycle.
Supply a lazy desktop provider to `MakeRuntimeScene`; real and replay use an
inert scene without invoking it. A desktop provider can add robot-specific
callbacks and must not outlive the robot objects it references. Base has no
Simulation dependency; the application wires the provider at composition time.

Applications can include `base/SimulationLatency.h` on desktop and roboRIO to
instrument the robot loop and joystick input. Its desktop implementation needs
MECO Simulation; its roboRIO functions are no-ops and need no Simulation headers.

`base/FootprintConfiguration.h` parses version 1 footprint planner JSON into
Base planner types. Applications supply measured dimensions, field bounds,
obstacles and limits, and check robot limits before using the result. Invalid
input returns no configuration. See [the format contract](docs/footprint-configuration.md).

`base/NavigationGrid.h` loads a PathPlanner navgrid and finds the nearest
reachable point accepted by a caller's predicate using eight-neighbour path
distance without cutting blocked corners. The caller owns goal selection and
the response to a missing or invalid grid. `IsNavigationGridPointFree` checks
a field point against the grid, returning false for invalid or outside points.

MecoBase 2026.2.1 adds per-module simulation device mounting directions, separate from electrical motor inversion. The public API uses headers such as `base/Drive.h` and `base/Mechanism.h`. The consuming robot owns its `Robot` and `RobotContainer` entry points.

Version 2026.2.3 removes redundant test forwarding functions while retaining the existing safety and behavior checks. Public APIs and robot behavior are unchanged.

Version 2026.2.8 adds reusable terrain-region mapping. Version 2026.2.7 adds optional footprint planning and moving-target replanning. Version 2026.2.6 stops native planning when the navgrid is invalid. Version 2026.2.5 adds native AutoDrive and object perception; 2026.2.4 added reusable Choreo trajectory following and an operator-armed system check runner.

## Learn AutoDrive

Start with [AutoDrive basics](docs/autodrive-basics.md), then use the [full command contract](docs/autodrive.md) for configuration and limits.

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

Pull requests run desktop Debug tests. Main-branch and manually dispatched integration builds produce Linux, roboRIO, and Windows archives, assemble a local Maven repository, and link clean consumer projects. GitHub Pages publication uses the **Bounded MECO Base release** workflow, dispatched from `main` with `release_ref` set to a reviewed `v*` tag and `publish` enabled. Published Maven versions are retained when the site is updated. It packages that existing immutable release tag from the `release_ref` input. Run a packaging check and publish with:

```sh
gh workflow run package-vendordep-bounded.yml --ref main -f release_ref=v2026.2.15 -f publish=false
gh workflow run package-vendordep-bounded.yml --ref main -f release_ref=v2026.2.15 -f publish=true
```

Pull-request desktop checks prepare SHA-verified Studica Maven and Gradle distribution artifacts once. Each compiler shard and the desktop test reverify both artifacts before use.

For 2026.2.15, dispatch the dedicated Windows recovery workflow from the immutable release tag once for Debug and once for Release. Supply both successful run IDs to the bounded package workflow with the same release tag; it verifies their source, coverage, and artifact digests before packaging.

Each Linux and Windows archive builds separately; the workflow verifies the staged archives against the tagged source before assembling the package. Update `vendor/MecoBase.json` and the artifact version together. For the one-time 2026.2.13 recovery, `reuse_run_id=41962900942` reuses pinned Linux artifacts and a separately verified bounded Windows Release archive, then rebuilds Windows Debug in source-verified shards. The Windows recovery enforces a three-minute total budget and embeds Debug symbols in the objects; staged and deployed consumer checks still run before release completion. For the one-time 2026.2.14 recovery, use the dedicated Windows recovery workflow once per configuration at the pinned workflow commit, and provide both successful run IDs together with `reuse_run_id=42036443481` to the bounded release workflow. The recovery verifies archive digests and source provenance; staged and deployed consumers remain required. If Pages deployment completes but the release job is cancelled during verification, use the `Verify deployed Base 2026.2.14` workflow to independently verify public checksums and link the six deployed consumers without republishing.

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
| Input | Validated NT4 gamepad axes and Driver Station arbitration | [`DriverInput.h`](src/main/include/base/DriverInput.h), [`NtGamepad.h`](src/main/include/base/NtGamepad.h) |
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

`base::NtGamepad` reads `/Input/Gamepad` arrays in the form `[1, sequence, source timestamp in microseconds, leftY, leftX, rightX]`. It rejects malformed, stale, or regressed samples and returns neutral axes when input is unavailable. It provides axes only. `base::ReadDriverInput` selects NT or Driver Station axes with the configured fallback policy and returns neutral drive axes unless teleop is enabled and emergency stop is clear. Buttons and fallback preference remain robot-owned.

### Simulation tuning policy

`base::SimulationTuning` lets a robot select `Automatic` simulation-derived gains or `Configured` gains. `ConfigureRuntimeDrive` lazily invokes the supplied simulation configuration only in simulation; hardware and replay retain the supplied configuration. `ConfigureRuntimeJointGains` removes gravity feedforward only for automatic tuning of an explicitly gravity-free simulated plant. Configured mode retains the robot's gains. This policy does not configure or disable the simulation's physical model.

For a bounded 2026.2.15 Windows recovery after the original eight-shard run fails, the separately reviewed recovery tooling supplies a prewarmed Gradle cache and 16 exact-tag source shards. Preserve the tooling commit at an immutable branch, dispatch the prewarm and recovery workflows from that same commit, and run Debug and Release separately. The publisher independently checks the pinned tooling SHA, full shard set, exact tagged source, archive provenance, and each run's three-minute queue-plus-execution limit. A completed or cancelled job is not proof of a successful recovery.

If all 16 Windows recovery shards completed successfully within their original deadline but the concurrent assembly did not, dispatch the separately pinned 2026.2.15 assembly workflow with that shard run ID and configuration. It verifies each shard and its artifact, then performs a new independently timed archive assembly. The publisher checks both runs and their recorded digests; the earlier assembly's cancelled or failed result is not counted as successful.
