# Runtime parity decisions

Drive configuration is validated before any vendor construction, including
replay. Physical dimensions, inertias, gearing and current limits must be finite
and positive; suspension damping must be positive; friction/gains must be nonnegative. Module locations must be
distinct and span a plane. CAN IDs are 0–62 and unique within device type on the
configured bus (including Phoenix’s empty default-bus alias): TalonFX drive/steer identities share one namespace, CANcoders use
another, and the Pigeon may share a numeric address with either.

These checks establish usable numerical inputs, not physical calibration.
Phoenix continues to own native odometry/control timing (250 Hz on CAN FD,
100 Hz on CAN 2.0 by its pinned default). The separate [Spark/NavX backend](portable-swerve.md)
samples at 100 Hz by default and uses WPILib estimation; it does not add a second
loop to Phoenix.

## Translation SysId

`drive.Characterize(false, frc2::sysid::Direction::kForward)` creates a
quasistatic command; `true` selects a dynamic step. Reverse uses `kReverse`.
The returned owning command requires the drivetrain. WPILib's default ramp,
step voltage and timeout apply. Phoenix's native `SysIdSwerveTranslation` and the
portable module adapters hold all azimuths forward while applying voltage. Normal drivetrain commands must not
run concurrently; interruption and completion apply zero then stop.

Each module logs measured motor voltage, coupling-corrected wheel distance in
metres, and wheel speed in metres/second through WPILib SysId. Recorded
`Drive/Characterization` inputs let replay recompute command voltage requests
without constructing vendor devices or physics adapters. Old recordings without those inputs can
still replay normal driving; adding a SysId command to an old recording fails
explicitly for the missing input rather than inventing measurements.

Translation characterization does not add steer/rotation characterization or
change native odometry timing. Hardware SysId requires a safe test area and
separate operator authorization; no physical tuning is claimed by native tests.

Phoenix characterization requests 50 Hz voltage status updates without waiting on CAN.
It requires valid native odometry and voltage status, with timestamps no older
than 100 ms. Portable characterization uses its sampler's measured voltage,
distance, speed and health snapshot. The health result is recorded with measured inputs, so replay also
reproduces feedback loss even when stale numeric values remain finite. Unhealthy
or malformed feedback applies zero voltage before failing the routine. This is
a conservative characterization gate, not a general CAN fault detector.
