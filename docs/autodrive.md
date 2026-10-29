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
check for field boundaries and rectangular obstacles. It accounts for the shortest
rotation between poses and fails closed on invalid geometry. The current AutoDrive
planner does not yet use this check or perform orientation-aware route repair;
its inflated navgrid remains necessary.

MecoBase `2026.2.6` includes desktop and roboRIO
Debug/Release archives. Windows is omitted only with the explicit development
flag `-PallowMissingWindows=true`; a public release still requires all supported
platforms and clean-consumer verification. No hardware was deployed by this change.
