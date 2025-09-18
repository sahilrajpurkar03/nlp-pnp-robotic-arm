# High-level state of the pick & place pipeline
int32 state
# 0 = IDLE
# 1 = RECEIVED_TARGET
# 2 = MOVING_TO_PICK
# 3 = PICKING
# 4 = MOVING_TO_PLACE
# 5 = PLACING
# 6 = COMPLETED
# -1 = ERROR

# Flag whether the last cycle succeeded
bool success

# Human-readable event description
string event_msg

# Target information
float64 target_x
float64 target_y
float64 target_yaw
int32   target_box

# Current end effector pose
geometry_msgs/PoseStamped ee_pose

# Gripper state
int32 gripper_state
# 0 = UNKNOWN
# 1 = OPEN
# 2 = CLOSED
# 3 = MOVING
# -1 = ERROR
