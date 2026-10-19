# This message contains the joint position, velocity, and torque control values for a single joint.

int32[] joint_index     # Index of the joint to control
float64[] p_des         # Desired joint position in radians
float64[] v_des         # Desired joint velocity in radians/second
float64[] kp            # Position gain
float64[] kd            # Velocity gain
float64[] torque        # Desired joint torque in N/m
