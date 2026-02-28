uint64 timestamp

uint32 mission_id
int32 current_seq

uint16 mission_item_count
uint16 total_setpoint_count
uint16 setpoint_count
uint8 chunk_index
uint8 chunk_count

bool reference_valid
bool home_alt_valid
bool truncated
uint8[2] padding0

uint8 MAX_SETPOINTS = 32
uint8 MAX_CHUNKS = 4

uint16[32] sequence
float32[32] north
float32[32] east
float32[32] down
