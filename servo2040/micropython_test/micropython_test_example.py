import time
import math
from servo import Servo, servo2040

"""
Demonstrates how to create a Servo object and control it.
"""

# Create a servo on pin 0
s = Servo(servo2040.SERVO_1)

# Move the servo using pulse width
s.pulse(500)
time.sleep(0.5)

s.pulse(1500)
time.sleep(0.5)

s.pulse(2500)
time.sleep(0.5)

s.pulse(500)