# Unmodified intent sampled before multicopter position-control overlay selection.
# intent_id is an opaque FMU token, NOT a timestamp. Echo it unchanged in the output.
uint32 MESSAGE_VERSION = 0
uint64 timestamp
uint64 session_id
uint64 intent_id
uint32 reset_counter # Changes when the estimator reference changes
uint8 nav_state
bool applicable
bool armed
TrajectorySetpoint intent

# Destination before the source mode performs trajectory smoothing, when available.
bool position_goal_valid
float32[3] position_goal # Local NED destination, m; Navigator or active GotoControl
