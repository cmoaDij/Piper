# AgxGripper status feedback message

std_msgs/Header header

# Gripper status
float64 width           # Current gripper opening width, unit: m
float64 force           # Current gripping force, unit: N

# Driver status
bool voltage_too_low        # Voltage too low
bool motor_overheating      # Motor overheating
bool driver_overcurrent     # Driver overcurrent
bool driver_overheating     # Driver overheating
bool sensor_status          # Sensor status
bool driver_error_status    # Driver error status
bool driver_enable_status   # Driver enable status
bool homing_status          # Homing/zero position status
