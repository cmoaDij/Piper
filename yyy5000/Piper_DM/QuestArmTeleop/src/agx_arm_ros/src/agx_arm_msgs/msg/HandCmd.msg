# Revo2 dexterous hand control command message

# Control mode: position/speed/current
string mode

# Target values for each finger
# position mode: [0, 100]
# speed mode: [-100, 100]
# current mode: [-100, 100]
int8 thumb_tip          # Thumb tip
int8 thumb_base         # Thumb base
int8 index_finger       # Index finger
int8 middle_finger      # Middle finger
int8 ring_finger        # Ring finger
int8 pinky_finger       # Pinky finger
