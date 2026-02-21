uint64 timestamp

bool mpc_allowed
bool mpc_active
bool obstacle_data_fresh
bool obstacle_triggered
uint8 obstacle_count
uint8[3] padding0

float32 nearest_obstacle_distance
float32 trigger_distance
float32 vehicle_speed

bool solve_success
uint8[3] padding1
float32 objective_value
int32 qp_iterations
float32 qp_solve_time_us

int32 last_qp_status
