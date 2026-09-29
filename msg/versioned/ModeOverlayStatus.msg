# Firmware-owned mode-overlay selection and health. Published even while disabled.
uint32 MESSAGE_VERSION = 0
uint64 timestamp
uint64 session_id
uint64 accepted_sequence
uint32 applicable_modes
uint32 reset_counter
uint32 rejected_outputs
uint8 nav_state
bool enabled
bool registered
bool applicable
bool engaged # A valid replacement is actually selected by the controller
bool ready # A fresh, correlated response reports ready
bool braking # Firmware brake reference is actually selected
bool failed # Latches on loss/invalid output in flight until disarm
float32 max_deviation # Granted local reference radius, m
float32 max_velocity_xy # Horizontal speed limit, m/s
float32 max_velocity_up # Climb rate limit, m/s
float32 max_velocity_down # Descent rate limit, m/s
float32 max_acceleration # Acceleration norm accepted in every direction, m/s^2
float32 max_acceleration_xy # Horizontal acceleration norm limit, m/s^2
float32 max_acceleration_up # Upward (NED -z) acceleration limit, m/s^2
float32 max_acceleration_down # Downward (NED +z) acceleration limit, m/s^2
float32 max_jerk # Jerk norm limit, m/s^3
bool yaw_authority # REPLACE heading is in effect; false keeps the source mode heading
float32 max_yawspeed # Heading rate limit for granted headings, rad/s
float32 max_yaw_acceleration # Heading acceleration limit for granted headings, rad/s^2
float32 timeout # Maximum input and response age before braking, s
