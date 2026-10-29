# Optional mechanism model and dashboard tuning

`ModelFeedforward::Motor` derives `v` and `a` from WPILib's `LinearSystemId::DCMotorSystem`. Inertia is at the output shaft; gearing is motor/output speed. The coordinate scale is one unit per rotation for rotary mechanisms, or drum circumference for metres. `Arm` adds horizontal-arm gravity amplitude (use `Gravity::Cosine` and an appropriate physical zero); `Elevator` adds constant gravity voltage and uses drum circumference. Elevator inertia must already include `mass * radius²`. `ApplyTo(existingGains)` preserves PID, friction, profiles and limits. Models assume ideal gearing and omit friction, compliance and efficiency losses; coefficients require validation against measured hardware.

Instantiate `MechanismTuning tuning{joint};` only for mechanisms you intend to tune. It registers a subsystem that consumes one JSON string topic `/Tuning/<mechanism name>/Request`; `/Status` reports revision and acceptance/rejection via the low-priority snapshot worker. No per-gain aliases are created. The tuner and mechanism must outlive commands created from `tuning.TestCommand()`.

A gain transaction requires the complete gain object and a strictly increasing positive unsigned revision:

```json
{"revision":1,"gains":{"p":1,"i":0,"d":0,"s":0,"v":0,"a":0,"gravity":0,"maxVelocity":1,"maxAcceleration":1,"tolerance":0.01,"minimum":0,"maximum":1,"defaultPosition":0}}
```

The complete gain transaction is validated before submission, only while disabled and with no command owning the mechanism. Firmware configuration can partially fail; disabled-only updates and output inhibition protect control safety, including in simulation. A rejected revision is consumed; resending it later cannot unexpectedly apply it.

Setpoints are separate transactions, e.g. `{"revision":2,"setpoint":0.5}`. They apply only while enabled in Test mode with the explicit `TestCommand` scheduled. That command requires the mechanism and stops it on interruption/end. Position setpoints must respect physical configured limits; flywheel setpoints must respect configured maximum velocity. Both gains and setpoints in one transaction cannot satisfy the mutually exclusive disabled/enabled requirements and are rejected. Requests outside the permitted state are discarded, never deferred until the next mode or command. A setpoint holds only for the active tuning command; no automatic button binding is installed.

Raw queued request strings are recorded through replay input substitution; re-executed tuner decisions use the recorded Driver Station state. Replay does not construct an NT subscriber or publisher. Transaction status is recorded as output for comparison. This supports code traceability; it does not reproduce motor firmware internals.

Real runtime gain updates are asynchronous: `accepted` acknowledges submission,
not firmware completion. Subsequent tuning ticks report `pending`, `applied`, or
`fault` from the mechanism's control-tick input snapshot. Completion status retains
the accepted configuration revision even when later revisions were rejected. A fast update may reach
`applied` without an intervening `pending` tick. The dashboard must wait for
`applied` and issue a fresh setpoint after configuration. Disabled-only and
unowned-mechanism checks still apply. Software simulation applies gains locally.

## Live individual gain tuning

`LiveMechanismTuning{mechanism, enabled}` exposes each gain as an individual
number under `/TunableNumbers/<name>/Gains/` when enabled. Construct it after the
mechanism and keep it alive while the scheduler runs. Disabled instances create
no tuning entries and ignore dashboard writes. Values default to the mechanism's
configured gains; nonfinite values and entries whose NetworkTables type is not
double are rejected. The complete gain set is
validated before submission, including limits and profile constraints.

Updates are accepted while enabled and while commands own the mechanism. Real
controllers apply them asynchronously: output is stopped while pending or faulted,
and a fresh command request is needed after completion. `/Tuning/<name>/LiveStatus`
reports submission, pending, applied, rejection or fault. Inputs and applied
values are recorded in the replay frame when recording is active, in addition
to the standard NetworkTables log. A fault may represent partially applied
firmware settings; output remains inhibited until gains are successfully applied.
Set `/Tuning/<name>/Retry` to true to retry the same gain set after a fault;
the tuner clears it after reading. The older JSON transaction tuner remains
available with its existing restrictions.

`LiveDriveMotorTuning` provides individual Phoenix swerve drive and steering
slot values under `/TunableNumbers/Drive/MotorGains/`. It is opt-in and must be
constructed before the scheduler runs. Updates stop all modules and run on a
worker; `/Tuning/Drive/MotorStatus` reports completion or fault, and
`/Tuning/Drive/MotorRetry` retries the same values after a fault. A fresh drive
request is required after completion. Updates can partially apply; a fault
keeps drive output inhibited. This API currently supports Phoenix swerve only;
Spark/navX gain configuration remains construction-time.
