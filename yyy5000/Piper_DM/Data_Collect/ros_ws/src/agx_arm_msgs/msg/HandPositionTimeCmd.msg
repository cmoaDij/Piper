# Revo2 dexterous hand position/time hybrid control message
# For position_time_ctrl(), sends position and arrival time together

# Target positions for each finger [0, 100]
int8 thumb_tip_pos          # Thumb tip position
int8 thumb_base_pos         # Thumb base position
int8 index_finger_pos       # Index finger position
int8 middle_finger_pos      # Middle finger position
int8 ring_finger_pos        # Ring finger position
int8 pinky_finger_pos       # Pinky finger position

# Arrival time for each finger (unit: 10ms, range: [0, 255], e.g. 200 = 2 seconds)
uint8 thumb_tip_time        # Thumb tip arrival time
uint8 thumb_base_time       # Thumb base arrival time
uint8 index_finger_time     # Index finger arrival time
uint8 middle_finger_time    # Middle finger arrival time
uint8 ring_finger_time      # Ring finger arrival time
uint8 pinky_finger_time     # Pinky finger arrival time
