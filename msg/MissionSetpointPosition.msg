uint64 timestamp

uint32 mission_id
int32 current_seq

uint16 mission_item_count
uint16 setpoint_count

bool reference_valid
bool home_alt_valid
bool truncated
uint8[1] padding0

uint8 MAX_SETPOINTS = 128

uint16[128] sequence
float32[128] north
float32[128] east
float32[128] down
