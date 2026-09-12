# A response to an FMU-issued ModeOverlayInput, all references in local NED.
uint32 MESSAGE_VERSION = 0
uint64 timestamp # PX4 receive time; ROS publishers set zero
uint64 session_id
uint64 intent_id # Echo the input token unchanged
uint64 sequence # Strictly increasing within a session; replayed outputs are rejected
uint8 ACTION_PASSTHROUGH = 0
uint8 ACTION_REPLACE = 1
uint8 ACTION_STOP = 2
uint8 action
bool ready # Companion perception/state are fresh; false requests a stop
TrajectorySetpoint setpoint # Full finite position, velocity, acceleration for REPLACE
