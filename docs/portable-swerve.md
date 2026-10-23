# Portable Spark / NavX swerve

`DriveConfig.backend = DriveBackend::SparkMaxNavX` selects four Spark MAX drive /
steering pairs, NEO motors and a NavX on MXP SPI. Phoenix native swerve remains
the default and retains its native synchronized acquisition. Desktop launches may
select `FRC_DRIVE_BACKEND=rev`, or configure the consuming robot to select its portable REV backend
with Jolt planar or 3D physics. Real hardware selection is explicit C++ config.
Full robot builds retain Phoenix headers and libraries for the default backend.

Each steering Spark uses its attached duty-cycle absolute encoder. Module
`encoder` CAN IDs and `pigeon` are unused by this backend. Only the roboRIO CAN
bus is supported. `offsetTurns` in [0,1) rotations is the REV absolute
encoder zero offset, `encoderInverted` its direction; motor inversion flags are
applied separately. Confirm positive drive, positive azimuth, wheel distance and
CCW gyro directions during hardware commissioning. Coupling correction uses
steering integrated position/rate and the configured coupling ratio.

`SparkSwerveGains` is separate from Phoenix rotor-turn gains: drive P/D use duty
cycle per metre/second and its derivative, steering P/D use duty cycle per radian
and radian/second, under 12 V compensation. Drive KS and KV are volts and volts
per metre/second. Public derivative gains use seconds; the real adapter multiplies
D by 1000 for REV's millisecond derivative timebase, while simulation differentiates
using seconds. Nonfinite converted gains are rejected before device construction.
Real closed-loop requests execute on each Spark using
`SetSetpoint`, absolute steering feedback and continuous position wrapping.
PathPlanner wheel forces are projected onto each optimized wheel heading and
converted to additional winding-drop voltage, without duplicating velocity FF.
The adapter accepts integer current limits from 1 to 200 A and rejects unsupported
values before device construction; choose motor-safe limits for the actual robot.
Illustrative gains and physical constants require robot-specific tuning.

A 100 Hz default sampling thread performs nonblocking cached reads and command
writes. A fixed 64-sample queue feeds WPILib kinematics and pose estimation on the
robot thread. This is software timestamped acquisition, not hardware-coherent
Phoenix odometry. Module liveness is based on REV cached-read status/errors;
unlike NavX, module feedback has no independent source-timestamp watchdog.
Configuration transactions run only during construction;
periodic CAN timeout is zero. NavX disconnect, calibration, frozen timestamp,
module read/status faults, disabled state and requests older than 100 ms stop
outputs and invalidate the request. Recovery requires a fresh command. Robot-side request publication and snapshots
use a separate short mutex, never held across vendor calls. Device-reset warnings
are cleared after installing checked status-frame periods and before applying
full startup configuration, with a disabled-only
250 ms bounded readback to establish the cleared cache baseline; a subsequent sticky reset
warning latches unhealthy until robot-code restart, since nonpersistent device
configuration may have been lost. No enabled reconfiguration is attempted. Stop and
join precede device destruction. No telemetry is published by the sampler.

Jolt uses vendor-free simulated module/gyro adapters, NEO electrical parameters,
the existing world clock and shared battery model. Emulated controllers consume
requests before each 4 ms world step. Sensor coordinates are calibrated physical
units and sample timestamps follow the physics clock. Encoder inversion/offset
firmware and actual REV controller firmware are not simulated or validated by
this model. Real device calibration remains a hardware acceptance step.

The existing Drive input schema remains readable. New portable recordings mark
the source backend and retain the same measured pose/speed/module-state boundary;
replay retains the WPILib kinematics azimuth cache across requests, including
zero-speed requests and estimator resets. It reconstructs requests and optimized
module targets without constructing
vendor devices or physics. It does not rerun the estimator from raw hardware
batches or emulate vendor firmware. Vision pose resets remain estimator operations;
`ResetSimulation` explicitly places the physical robot and synchronizes sensors.

Injected `Drive(config, SwerveIO)` requires `SparkMaxNavX` configuration. Real
injected IO uses the same default 100 Hz sampler thread; desktop injected IO
is acquired and commanded once per `Drive::Periodic` (normally 50 Hz), without
constructing a vendor adapter or automatically registering Jolt physics. Replay
creates no sampler and performs no IO reads or writes. The normal desktop
factory keeps its existing Jolt substep sampling owner. The estimator takes its
initial module/gyro baseline from the first accepted acquisition, preserving its
initial or explicitly reset field pose; historical encoder travel is not newly
integrated motion. Ordinary
`ResetPose` uses the latest acquired sensor baseline and rejects acquisitions
that crossed the reset; it does not move physics or defer the baseline to a later
sample.
