# Explicit-time mechanism control

`MotorIO::Read()` is observational. Normal robot simulation advances registered
plants exactly once through the shared world clock (five 4 ms steps per period).
No mechanism `SimulationPeriodic` or separate Notifier advances the same plant.

`Mechanism::Control(dt)` uses the supplied interval for bounded profiles and
passes it to motor IO. Simulated and injected IO use that interval for their
controller emulator; real IO sends setpoints to onboard firmware. The interval must be finite, positive and at
most 100 ms; invalid intervals stop before throwing. The ordinary `Periodic()`
retains 20 ms control and publishes cached telemetry through the background worker.
`Control(dt, enabled)` also checks DriverStation enable; an injected true flag
cannot authorize a disabled robot.

`UseExternalScheduling()` suppresses automatic control, while `Periodic()` still
publishes cached telemetry. A single caller may then own command scheduling,
`Control(dt)` and standalone plant `StepSimulation(dt)` (also available through
the compatibility name `AdvanceSimulation`). These APIs require the same owner
thread and must not race the command scheduler. External scheduling does **not**
disable the shared simulation world: do not manually advance a plant while that
world is running. The existing plant-step interval permits 0–1 s, independently
of the tighter control-timestep bound.

`DeterministicSimulation::StepWithSensors` is an offline harness: deliver delayed
sensors/DS events first, schedule commands every fifth integer 4 ms tick, then
control and physics. HAL time is caller-owned. `LatencyChannel` bounds pending
sensor delivery; `ReplayMotorIO` consumes timestamped immutable samples without
hardware. The legacy six-field `MotorInputLog` codec is offline-only and omits
external-encoder/native-control metadata. It does not add a duplicate live logger
or replace [full frame replay](replay.md). The normal frame recorder remains
20 ms; calling explicit control at 4 ms does not automatically create a separate
recorded frame for every substep.

The original branch evidence remains in `artifacts/time-step-*`; it describes
its historical implementation and is not a fresh validation of current source.
