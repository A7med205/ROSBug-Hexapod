"""Parameterized servo sweep from 0 to 180 and back to 0."""

import time
from adafruit_servokit import ServoKit

# === USER VARIABLES ===
num_steps = 180        # Total number of steps in the sweep (half-cycle)
time_interval = 0.005  # Delay between steps in seconds
step_size = 1          # Size of each angle step in degrees
# =======================

# Setup
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(min_pulse=500, max_pulse=2400)

# Generate angle range: 0 to 180, then back to 0
angles_up = list(range(0, 181, step_size))
angles_down = list(range(179, -1, -step_size))  # Avoid repeating 180
angles = angles_up + angles_down

# Move the servo through the generated angles
for angle in angles:
    kit.servo[0].angle = angle
    time.sleep(time_interval)
