# Register a single multicopter setpoint overlay. Registration is disarmed-only.
uint32 MESSAGE_VERSION = 0
uint64 timestamp # PX4 receive time; ROS publishers set zero
uint64 request_id # Nonzero transaction ID, echoed in the reply
uint64 session_id # Nonzero random ID for one companion process lifetime
uint8 ACTION_REGISTER = 0
uint8 ACTION_UNREGISTER = 1
uint8 action
char[25] name
uint32 applicable_modes # VehicleStatus navigation-state bitmask; nonzero
float32 max_deviation # Requested local reference radius from vehicle position, m
uint8 ORB_QUEUE_LENGTH = 4
