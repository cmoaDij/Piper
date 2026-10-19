# Arm status feedback message
uint8 ctrl_mode             # Control mode
uint8 arm_status            # Arm status
uint8 mode_feedback         # Mode feedback
uint8 teach_status          # Teach mode status
uint8 motion_status         # Motion status: 0=reached, 1=not reached
uint8 trajectory_num        # Current trajectory point number
int64 err_status            # Error status
bool[] joint_angle_limit    # Joint angle limit flags
bool[] communication_status_joint  # Joint communication status
