# Reply to a mode overlay registration transaction.
uint32 MESSAGE_VERSION = 0
uint64 timestamp
uint64 request_id
uint64 session_id # Echoes the requesting session, including on rejection
uint8 RESULT_ACCEPTED = 0
uint8 RESULT_DISABLED = 1
uint8 RESULT_ARMED = 2
uint8 RESULT_BUSY = 3
uint8 RESULT_INVALID = 4
uint8 result
uint32 applicable_modes # Granted modes; valid on acceptance
float32 max_deviation # Granted local reference radius, m
uint8 ORB_QUEUE_LENGTH = 4
