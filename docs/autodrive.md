# Native AutoDrive

`base/AutoDrive.h` supplies `AutoDriveToPose` on both roboRIO and desktop. Call
`ConfigureNativeAutoDrivePathfinder()` before `Drive::ConfigureAuto()`, then bind the returned command to a controller or
an autonomous chooser. Planning and trajectory following execute in C++, using
the native AD* implementation adapted from PathPlanner. No Python interpreter, command server,
or desktop companion participates in driving.

Pass a supplier returning an optional absolute blue-origin `frc::Pose2d`. The goal
is sampled when scheduled and never alliance-flipped. A missing or invalid goal
reserves the drivetrain while commanding zero. A goal which becomes unavailable
or invalid cancels path following; release, interruption and disable stop output.
The drive and anything captured by the supplier must outlive the command. Release
and press again to choose another goal. The supplier must enforce freshness for
camera-derived targets; `ObjectPerception` provides fresh clustered detections.

`AutoDriveTrackTarget` is an opt-in command for suppliers whose targets move.
It stops when translation changes by the configured threshold, waits for a fresh
observation and measured pose and speed to settle, and starts another pathfinding
attempt. Missing or nonfinite feedback or a wait timeout stops the command. Replanning
is bounded to three attempts; invalid or missing observations end the command.
Heading-only changes do not trigger replanning. The target supplier remains
responsible for enforcing freshness. These requests still require the configured
navgrid and robot-specific validation of stopping and localization.

`AutoDriveConfig` supplies field bounds and trajectory constraints: 2 m/s,
3 m/s², 3 rad/s and 6 rad/s² by default. These are illustrative settings, not
measured hardware tuning. DriveConfig still owns hardware limits and the configured
path-following controller. Deploy `pathplanner/navgrid.json` with the robot; it
must describe obstacle clearances for the actual footprint. Configure localization
and verify the field pose before commanding an absolute target on hardware.
An absent or invalid navgrid prevents all native path generation.
For a goal less than 0.5 m away, a bounded final approach runs only while the
inflated navgrid and current dynamic obstacles leave its centerline clear. A
missing grid or blocked segment causes a stop; the grid must account for the
robot's footprint.

During AutoDrive, PathPlanner feedback output is bounded by the configured
translation and rotation limits, including feedback corrections. The limit is
released when the command ends or is interrupted. If a timed path ends short,
AutoDrive makes up to three pathfinding attempts in total, using the same
obstacle-aware planner. It then uses the grid-checked final approach only for a
goal within 1 m. Arrival requires position, heading and measured speed to remain
within tolerance for five scheduler ticks. If the goal is still farther away or
the final segment is blocked, it stops without claiming arrival.

This uses a native planner adapted from the upstream AutoDrive planner.
It does not claim parity with that project's additional Python SE(2) repair,
terrain speed profiling or training policies. There is no fallback to a Python
planner. MecoSimulation supplies optional synthetic camera observations through
the `ObjectDetectionIO` contract; hardware uses its NetworkTables IO adapter.

`base/FootprintClearance.h` supplies a reusable, conservative swept rectangle
check and bounded orientation-aware grid search for field boundaries and
rectangular obstacles. Sweeps use the shortest rotation between poses and fail
closed on invalid geometry. The existing `AutoDriveToPose` planner does not use
this search; its inflated navgrid remains necessary.
When direct travel is blocked, the grid search can start from nearby reachable
cell centers if the containing cell center is obstructed; each connection is
checked as a swept footprint. A failed bounded search is not proof that no
continuous route exists.
`base/FootprintFollower.h` provides a field-relative feedback prototype for
these routes. It settles at each waypoint and latches a zero request on invalid
feedback or blocked geometry. An optional separate final-settling sample count
can hold the last pose longer than intermediate waypoints. `base/FootprintCommand.h` wraps a supplied route
in a drivetrain-owning command, samples field obstacles each 20 ms scheduler
cycle, and stops on completion, interruption, disable, or a failed supplier.
By default, within the heading tolerance the follower requests zero angular
speed and still requires its speed and dwell checks before reporting arrival.
An optional narrower angular stop tolerance permits correction farther inside
the arrival band; it does not change the arrival or dwell thresholds.
Optional pose-derived angular damping and settling use successive estimated
headings, with a separate wheel-speed sanity limit. They are disabled by default;
only enable them after validating localization freshness and timing for the robot.
Route search must run off the scheduler thread. Callers must account for latency,
tracking error, and braking distance; the geometric prediction alone is not a
physical collision guarantee. These prototypes do not replace `AutoDriveToPose`.
`base/FootprintAutoDrive.h` combines a goal supplier, cancellable background
search, and this follower in an alternate drivetrain-owning command. It uses a
snapshot for planning, rechecks the current obstacle supplier while following,
and makes up to three attempts at the original goal when a route is blocked;
the supplier must be fast on the scheduler thread. Only one footprint search can
run at a time. Before each search the command requests zero and waits for both
wheel speeds and estimated pose changes to remain below configurable limits;
it ends stopped if the settling timeout expires or feedback is invalid. This
cannot detect motion missing from both feedback sources or guarantee braking
distance. The robot application must opt in with measured chassis and field
geometry; no hardware or season simulation validation has been completed.
By default the target remains the initial sampled pose. A positive
`targetTranslationReplanThreshold` instead stops and replans toward a fresh
target when its translation moves by that distance. Target changes share the
three-attempt bound with obstacle replans; heading-only changes do not trigger
replanning, and target freshness remains the supplier's responsibility. Before
replanning after a target change, the command holds zero until the observed
target remains within the threshold for `targetStabilitySeconds`; a missing
target or `targetStabilityTimeout` ends stopped. This reduces retry churn but
does not invent observations while a target is missing.

`base/FootprintSpeedProfile.h` provides a separate experimental per-waypoint
speed calculation with caller-supplied segment grade and rolling resistance,
turn limits, and forward acceleration and backward braking bounds. An empty
result means the inputs are invalid or the initial speed cannot stop in the
available route. An optional terrain supplier enables its use by the footprint
follower; without it, the follower still settles at every waypoint. With it,
nonzero-speed intermediate waypoints can be passed, while zero-speed waypoints
and the final pose still require settling. The supplier receives each directed
segment on the scheduler thread and must describe its grade and rolling
resistance. A segment endpoint overshoot outside position tolerance blocks the
route so the owning command can replan. This prototype does not validate a
robot's traction and braking.
An optional boundary supplier can split each planned segment at ordered
fractions before the terrain supplier is called. The split follows the same
linear position and shortest-angle heading interpolation; invalid or excessive
boundaries fail closed. The supplier must identify every terrain transition
relevant to its model and be fast enough for scheduler setup.
The profile speed envelope overrides the request slew limit if estimated pose
changes suddenly. That limits the commanded velocity but cannot guarantee the
physical robot can decelerate in the remaining distance.
An optional translation progress watchdog stops the follower if its estimated
distance to the current waypoint fails to improve by the configured amount for
the timeout while a substantial translation is requested. It is disabled by
default. It requires a fresh, trustworthy pose estimate; wheel odometry can
report false progress during slip. The watchdog reports a distinct stalled
result, and the owning AutoDrive command ends stopped instead of retrying it.
Blocked geometry can still be replanned up to the existing attempt limit.
An optional scheduler-thread outcome distinguishes arrival, stall, exhausted
blocked attempts, no route returned, a busy planner, invalid feedback, source
failure, unavailable target, failed settling, and interruption. A no-route
result can mean the search budget was exhausted or the worker failed; it does not prove no route
exists. The shared outcome must be read with appropriate synchronization if a
consumer accesses it outside the scheduler thread.
`FootprintSegmentSpeed` computes a bounded reference at a distance along one
segment, including the acceleration and braking limits for its terrain. It
rejects endpoint speeds that cannot be connected within the segment.
Its numerical approach is informed by `speed_profile` and
`_segment_accelerations` in AutoDrive revision `d74b0a15`; it does not yet
model the original module-angle braking adjustment or reproduce its complete
path reference behavior.

The `2026.2.8` candidate package is intended to include desktop and roboRIO
Debug/Release archives. Windows is omitted only with the explicit development
flag `-PallowMissingWindows=true`; a public release still requires all supported
platforms and clean-consumer verification. No hardware was deployed by this change.

`base/FootprintTerrainMap.h` supplies reusable static rectangular terrain regions
and segment-boundary splitting. Callers provide field-frame grade components and
rolling resistance; segments must be split at the reported boundaries before
sampling. Invalid or overlapping regions and travel exactly along a shared region
boundary fail closed. This is a centerline model, not a whole-chassis contact
model or physical traction guarantee.
