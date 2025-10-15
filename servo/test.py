"""Basic sweep."""

import time
from adafruit_servokit import ServoKit

# Setup
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(min_pulse=500, max_pulse=2400)

# Move the servo 
kit.servo[0].angle = 45
time.sleep(1.5)
kit.servo[0].angle = 90
