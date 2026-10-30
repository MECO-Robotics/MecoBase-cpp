# Footprint configuration format

`base::ParseFootprintConfiguration` accepts a JSON object with `version: 1`
and returns no value when the input is invalid. Lengths and positions are in
metres, linear speeds in m/s, linear accelerations in m/s², angular speeds in
rad/s, and angular accelerations in rad/s². All numeric values must be finite.

Required members are:

- `footprint`: positive `length_m` and `width_m`, and nonnegative `margin_m`.
  The margin is added to both sides of each dimension.
- `field`: a rectangle with `min_x`, `min_y`, `max_x`, and `max_y`. Each
  minimum must be smaller than its maximum; the footprint must fit the field.
- `obstacles`: 1–1024 valid rectangles in the same format. Each must intersect
  the field. An empty list is rejected; the application must provide an
  explicit obstacle map.
- `planner`: `resolution_m` in [0.05, 1] and integer `expansion_budget` in
  [1, 200000].
- `limits`: positive `max_speed_mps`, `max_omega_radps`,
  `max_acceleration_mps2`, and `max_alpha_radps2`.

An optional `terrain` object requires 1–128 `regions`, each with a `bounds`
rectangle entirely within the field, finite `grade_x` and `grade_y` (rise/run),
and nonnegative `rolling_resistance`. Regions cannot overlap in area. The
object also requires positive `lateral_acceleration_mps2`, `steering_rate_radps`,
`stall_timeout_s`, `stall_progress_m`, and `stall_request_mps`; the stall
request must be less than `max_speed_mps`. Without terrain, no terrain callbacks
are installed. If supplied, the result owns the immutable map through its
callbacks.

The caller owns the choice of geometry and must compare configured motion
limits with the robot's actual drive limits before issuing a command. Parsing
does not establish that the obstacle map is complete or safe for a real field.
