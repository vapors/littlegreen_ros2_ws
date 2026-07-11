std_msgs/Header header

uint64 cycle_index
int64 cycle_start_monotonic_ns
int64 cycle_end_monotonic_ns
uint32 read_start_index

bool command_valid
bool command_stale
uint64 command_sequence
int64 command_receipt_monotonic_ns
float64 command_age_ms
float64[12] command_target_rad

bool target_valid
float64[12] target_rad_from_steps
int32[12] target_steps

# Static per-joint ST3215 motion-profile values used by every SyncWrite.
# v2.4.2 Track 1 baseline ships with speed=0 and acceleration=0 for all joints.
uint16[12] configured_speed_steps_s
uint8[12] configured_acceleration_units

bool write_due
bool write_attempted
bool write_ok
uint64 written_command_sequence
int64 sync_write_start_monotonic_ns
int64 sync_write_end_monotonic_ns
float64 sync_write_us

float64 feedback_sweep_us
float64 cycle_work_us

float64[12] q_meas_rad
float64[12] qdot_meas_rad_s
int32[12] raw_position_steps
int32[12] raw_speed
int32[12] raw_load
float64[12] load_ratio
float64[12] voltage_v
int32[12] temperature_c
uint8[12] servo_status
bool[12] moving
int32[12] raw_current
float64[12] current_a
int64[12] sample_monotonic_ns
float64[12] feedback_age_ms_at_cycle_end
bool[12] read_ok
uint8[12] status_error

uint64 telemetry_dropped_count

# Bus-reported torque command state: -1 unknown, 0 disabled, 1 enabled.
int8 torque_enabled_state
