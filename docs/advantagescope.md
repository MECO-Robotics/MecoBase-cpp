# AdvantageScope

Use NT4 on localhost port 5810 for the owned desktop simulator, or open recorded
WPILOG files. [Simulator build and launch](../README.md#quickstart) and
[code replay](replay.md) describe the current launch and recording workflows.

| Canonical topic | Type / use |
| --- | --- |
| `/AdvantageScope/Drive/Pose` | Estimated `Pose2d` |
| `/AdvantageScope/Drive/ChassisSpeeds` | Measured robot-relative `ChassisSpeeds` |
| `/AdvantageScope/Drive/RequestedChassisSpeeds` | Requested robot-relative motion |
| `/AdvantageScope/Drive/ModuleStates`, `ModuleTargets` | `SwerveModuleState[]`, FL/FR/BL/BR |
| `/Simulation/RobotTruth`, `/Simulation/Pieces` | Optional physical `Pose3d` / `Pose3d[]` topics from a matching-year simulation integration |
| `/Metadata/*` | Project, language, WPILib and telemetry format strings |

No legacy `Speeds`, `MeasuredStates`, `TargetStates` or lifted planar `RobotPose`
alias is restored. Physical truth remains distinct from estimated pose. Telemetry
snapshots and metadata publish through the low-priority worker. Linear units are
metres and metres/second; geometry angles are radians. Mechanisms retain their
configured output rotations or metres, as documented in their APIs.

Validate a log and generate a layout with the installed AdvantageScope parser:

```sh
node scripts/validate_advantagescope.cjs --log logs/example.wpilog \
  --output artifacts/advantagescope-validation.json \
  --layout artifacts/advantagescope-layout.json
```

The tool reads the installed ASAR worker/WASM without modifying the application.
It requires decoded drivetrain types and project/WPILib metadata; the optional
Simulation integration supplies 3D robot truth. Historical branch sample logs/layouts in
`artifacts/advantagescope-base-*` use older topic names and remain historical
parser evidence. The validator accepts those older geometry locations as well.
Parser validation does not establish GUI rendering, hardware calibration or
physical field accuracy.
