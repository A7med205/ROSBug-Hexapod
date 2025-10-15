import time
import sys
import tty
import termios
from adafruit_servokit import ServoKit

# === USER VARIABLES ===
step_size = 30            # Size of each angle step in degrees
min_pulse = 500          # Minimum allowed pulse
max_pulse = 2400         # Maximum allowed pulse
# =======================

# Setup
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(min_pulse=min_pulse, max_pulse=max_pulse)

# Start servo in neutral position
current_pulse = 1500
kit.servo[0].set_pulse_width_range(min_pulse=min_pulse, max_pulse=max_pulse)
kit.servo[0].angle = None  # Disable angle mode so we can use raw pulses
kit.servo[0].set_pulse_width_range(min_pulse, max_pulse)
kit.servo[0].fraction = (current_pulse - min_pulse) / (max_pulse - min_pulse)


def get_key():
    """Read a single keypress from stdin and return it."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)                 # set terminal to raw mode
        ch = sys.stdin.read(1)         # read one char (blocking)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # restore settings
    return ch


if __name__ == "__main__":
    print("Control the servo with keys:")
    print("  P = move up")
    print("  I = move down")
    print("  O = reset to 1500 µs")
    print("  Q = quit")

    while True:
        key = get_key().lower()

        if key == "q":
            print("Exiting...")
            break
        elif key == "p":  # Increase pulse
            current_pulse = min(current_pulse + step_size, max_pulse)
            kit.servo[0].fraction = (current_pulse - min_pulse) / (max_pulse - min_pulse)
            print(f"Pulse increased: {current_pulse} µs")
        elif key == "i":  # Decrease pulse
            current_pulse = max(current_pulse - step_size, min_pulse)
            kit.servo[0].fraction = (current_pulse - min_pulse) / (max_pulse - min_pulse)
            print(f"Pulse decreased: {current_pulse} µs")
        elif key == "o":  # Reset to center gradually
            target_pulse = 1500
            step = 10 if target_pulse > current_pulse else -10  # choose direction
            for pulse in range(current_pulse, target_pulse, step):
                kit.servo[0].fraction = (pulse - min_pulse) / (max_pulse - min_pulse)
                time.sleep(0.01)  # adjust speed (smaller = faster)
            current_pulse = target_pulse
            print(f"Pulse gradually reset to: {current_pulse} µs")
        else:
            print(f"Unmapped key pressed: {repr(key)}")
