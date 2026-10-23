# Recording and replay

The library records one versioned `Replay/Frames` JSON record per robot cycle in the primary WPILOG. The file uses the AdvantageKit WPILOG header and includes timestamped typed `ReplayInputs` and `ReplayOutputs` entries for AdvantageScope and other WPILOG readers.

A consuming robot integrates replay at its lifecycle boundary: install recorded Driver Station state and FPGA timestamps before running its normal mode callbacks, scheduler, commands, and IO-owned mechanism controller emulation. The framework does not feed recorded actuator outputs back into the code. New outputs go to a new WPILOG.

The replay adapters accept Java AdvantageKit 26.0.2 logs for the documented drivetrain and Driver Station schemas. The Java log does not identify the C++ drivetrain backend, so configure `FRC_DRIVE_BACKEND` when it differs from the consuming robot's default. Other Java mechanism, vision, and device IO schemas must be mapped by the consuming robot; replay must not read live devices as a fallback.

AdvantageScope can open either log for normal playback; code replay is a separate operation. WPILib NT topics are stored with an `NT:` prefix in WPILOG. Each recorded cycle also exposes timestamp-aligned JSON snapshots at `Replay/Inputs` and `Replay/Outputs`; `Replay/Frames` remains the authoritative replay source. `Replay/RecordingStatus` is sampled once per second and finalized at orderly shutdown. Periodic snapshots may lead or trail persisted frames because status and frame records are not synchronized with the lower-priority writer; readers require an exact count from finalized status records.

For the C++ to Java format-compatibility probe, run `tools/akit-acceptance/verify-cpp-writer.sh`. It generates a fixture through the library's native test and reads it with the Java AdvantageKit 26.0.2 reader from the sibling Java project. This checks selected typed values and timestamps; it does not establish full robot replay equivalence or hardware behavior.
