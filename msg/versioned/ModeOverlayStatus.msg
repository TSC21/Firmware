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
float32 max_deviation
float32 max_velocity_xy
float32 max_velocity_up
float32 max_velocity_down
float32 max_acceleration
float32 max_jerk
