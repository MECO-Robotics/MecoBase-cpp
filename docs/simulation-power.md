# Simulation battery model

The consuming robot can sample each registered plant's battery-side current after advancing the physics clock, then use WPILib `BatterySim` to set the next period's model bus voltage and HAL roboRIO input voltage. Reads never step a plant. Real and replay modes do not run this path.
