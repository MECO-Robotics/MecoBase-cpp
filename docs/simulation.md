# Optional desktop simulation

MECO Base keeps deterministic motor simulation and the fixed-step callback runtime. Jolt world physics, drive dynamics, arena support, and the WPILib desktop adapter are maintained in the year-specific [MECO Simulation repository](https://github.com/MECO-Robotics/MecoSimulation-cpp).

Install the Simulation vendordep that matches the current Base/WPILib year when that year's artifacts are released. The `wpilib-2026` branch contains the 2026 standalone physics library. The 2027/SystemCore branch is in development and does not claim compatibility until its target build and consumer checks pass. No cross-year binary compatibility is promised.

Base projects can build without the optional simulation package. The Jolt models support Linux and Windows desktop builds; they are not a roboRIO runtime dependency. Set each module's `simulation.modules[i]` device orientations to describe physical rotor-positive direction relative to the chassis. These fields are independent of motor `driveInverted` / `steerInverted` and encoder `encoderInverted` settings. Defaults are counterclockwise-positive.

Applications must construct `base::Drive` with an explicit robot hardware configuration. `base::DriveConfig{}` contains illustrative/example values and is intended for tests or deliberate simulation examples, not a robot hardware map.

`DriveSimulationConfig` keeps robot-specific physical values explicit. Set separate `driveMotor` and `steerMotor` profiles with winding resistance (ohms), torque constant (N·m/A), velocity constant (rad/s/V), and current limit (A). A profile with all values zero is unconfigured; otherwise resistance, both constants, and current limit must be positive. `frictionVoltage` is a nonnegative motor-side voltage loss. Set `bumperCornerRadius`, `bumperFriction`, `wheelWidth`, `contactPatchLength`, and `contactShearDeflection` when the selected physics model uses those properties; zero means unconfigured for these optional values. The optional physics adapter rejects configurations missing parameters required by its selected model rather than assuming a motor or contact profile for the robot.

`RuntimeScene` also exposes no-op `BeforeScheduler`, `AfterScheduler`, `BeforeScene`, and `AfterScene` hooks. Applications can override them to coordinate shared runtime callbacks around command scheduling and scene updates; real and replay scenes need no implementation. A desktop simulation result is not hardware calibration or commissioning evidence.
