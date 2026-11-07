std_msgs/Header header

uint32 tick_ms
uint8 state
uint8 status
uint8 is_busy
uint8 has_fault
uint8 active_cmd
uint8 active_source
uint8 active_seq
uint8 diag_code
# Cartesian current/target XYZ use the arm_base command reference (m).
float32 current_x
float32 current_y
float32 current_z
float32 target_x
float32 target_y
float32 target_z
float32 current_gripper_rad
float32 target_gripper_rad
uint32 fault_flags
