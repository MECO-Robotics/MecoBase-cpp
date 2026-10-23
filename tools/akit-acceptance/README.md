# AdvantageKit WPILOG acceptance probes

These probes use the actual AdvantageKit 26.0.2 writer and reader from the 2026
Java robot's GradleRIO runtime, plus the built C++ robot executable.
GradleRIO supplies the same version-matched WPILib Java and desktop JNI runtime
as the Java project. The init script adds this test source only for the command;
it does not edit the Java repository.

From the C++ base repository root, the complete C++ writer to Java reader check
generates a two-cycle fixture using the C++ logger and then reads it with the
actual AdvantageKit reader:

```bash
./tools/akit-acceptance/verify-cpp-writer.sh
```

The helper runs the `mecoBaseTest` task with
`AKIT_ACCEPTANCE_CPP_OUTPUT` set, enabling the gated fixture in
`AdvantageKitAcceptanceFixtureTest.cpp`. It then checks these stable paths and
types with the Java reader: `ReplayInputs/Drive/backend` (string),
`ReplayInputs/Drive/pose/0` (double), and
`ReplayOutputs/Drive/voltage` (double). It also verifies that the reader
recovers timestamped table values. The probe additionally inspects raw WPILOG
records for the two output values because AdvantageKit's replay reader filters
`ReplayOutputs` from its LogTable. The raw WPILib reader confirms both timestamped
voltage samples are present. The complete command passed on 2026-10-21, with
AdvantageKit recovering both input cycles and the raw reader recovering both
output samples.

To check a different C++ log, call `run.sh` directly with its path and optional
typed key assertions:

```bash
./tools/akit-acceptance/run.sh /path/to/cpp-generated.wpilog \
  'ReplayInputs/Drive/backend=String' 'ReplayInputs/Drive/pose/0=Double'
```

Types use AdvantageKit's enum names (`Boolean`, `Integer`, `Float`, `Double`,
`String`, arrays, or `Raw`). Without path assertions, the probe only proves
that some timestamped values were recovered.

Optional environment variable:

- `REBUILT_DIR`: path to the Java Rebuilt repository (default:
  `../2026-Rebuilt`).
