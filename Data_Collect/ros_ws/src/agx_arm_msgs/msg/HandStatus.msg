# Revo2 dexterous hand status feedback message

std_msgs/Header header

# Hand type flag: 1=left hand, 2=right hand
uint8 left_or_right

# Finger positions [0, 100]
uint8 thumb_tip_pos         # Thumb tip position
uint8 thumb_base_pos        # Thumb base position
uint8 index_finger_pos      # Index finger position
uint8 middle_finger_pos     # Middle finger position
uint8 ring_finger_pos       # Ring finger position
uint8 pinky_finger_pos      # Pinky finger position

# Finger motor status: 0=idle, 1=running, 2=blocked
uint8 thumb_tip_status      # Thumb tip motor status
uint8 thumb_base_status     # Thumb base motor status
uint8 index_finger_status   # Index finger motor status
uint8 middle_finger_status  # Middle finger motor status
uint8 ring_finger_status    # Ring finger motor status
uint8 pinky_finger_status   # Pinky finger motor status
