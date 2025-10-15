import time
from adafruit_servokit import ServoKit
import math

def trapezoidal_angle(t, T, angle_total):
    if T <= 0:
        return angle_total if t > 0 else 0.0

    ta = T / 3.0
    vmax = 1.5 * angle_total / T
    a = vmax / ta

    if t <= 0.0:
        return 0.0
    elif t < ta:
        return 0.5 * a * t * t
    elif t < 2.0 * ta:
        return 0.5 * a * ta * ta + vmax * (t - ta)
    elif t <= T:
        return angle_total - 0.5 * a * (T - t) * (T - t)
    else:
        return angle_total

# === Parameters ===
angle_total = 180         # Total degrees of motion per sweep
T = 1.0                   # Total time of each full sweep in seconds
dt = 0.002                # Time step in seconds
angle_threshold = 1       # Minimum angle change to send new signal
channel = 0               # Servo channel

# === Setup ===
kit = ServoKit(channels=16)
kit.servo[channel].set_pulse_width_range(min_pulse=350, max_pulse=2250)

# === Sweep 1 ===
t = 0.0
last_sent_angle = -999  # Force initial send

while t <= T:
    progress = trapezoidal_angle(t, T, angle_total)
    current_angle = progress
    current_angle = max(0, min(180, current_angle))  # Clamp to 0-180
    rounded_angle = int(current_angle)  # Round down

    if abs(rounded_angle - last_sent_angle) >= angle_threshold:
        kit.servo[channel].angle = rounded_angle
        last_sent_angle = rounded_angle

    time.sleep(dt)
    t += dt

# === Sleep ===
time.sleep(0.1)

# === Sweep 2 ===
t = 0.0
last_sent_angle = 999  # Force initial send

while t <= T:
    progress = 180 - trapezoidal_angle(t, T, angle_total)
    current_angle = progress
    current_angle = max(0, min(180, current_angle))  # Clamp to 0-180
    rounded_angle = int(current_angle)  # Round down

    if abs(last_sent_angle - rounded_angle) >= angle_threshold:
        kit.servo[channel].angle = rounded_angle
        last_sent_angle = rounded_angle

    time.sleep(dt)
    t += dt
