# Optional desktop simulation

MECO Base keeps deterministic motor simulation and the fixed-step callback runtime. Jolt world physics, drive dynamics, arena support, and the WPILib desktop adapter are maintained in the year-specific [MECO Simulation repository](https://github.com/MECO-Robotics/MecoSimulation-cpp).

Install the Simulation vendordep that matches the current Base/WPILib year when that year's artifacts are released. The `wpilib-2026` branch contains the 2026 standalone physics library. The 2027/SystemCore branch is in development and does not claim compatibility until its target build and consumer checks pass. No cross-year binary compatibility is promised.

Base projects can build without the optional simulation package. The Jolt models support Linux and Windows desktop builds; they are not a roboRIO runtime dependency. Set each module's `simulation.modules[i]` device orientations to describe physical rotor-positive direction relative to the chassis. These fields are independent of motor `driveInverted` / `steerInverted` and encoder `encoderInverted` settings. Defaults are counterclockwise-positive. A desktop simulation result is not hardware calibration or commissioning evidence.
