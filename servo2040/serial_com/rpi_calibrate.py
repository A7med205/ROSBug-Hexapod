# pip install pyserial
import time
import serial
import sys
import tty
import termios

# -------------------------
# Servo board configuration
# -------------------------
PORT = "/dev/ttyACM0"  # adjust with ls /dev/ttyACM*
BAUD = 115200
SERVO_CHANNEL = 1      # hard-coded servo channel
START_PULSE = 1500     # initial pulse width
STEP = 5               # increment/decrement step

# -------------------------
# Serial communication setup
# -------------------------
ser = serial.Serial(PORT, BAUD, timeout=1)

def send(cmd: str) -> str:
    ser.write((cmd + "\n").encode("utf-8"))
    line = ser.readline().decode(errors="ignore").strip()
    return line

def do_pulse(ch: int, pulse_us: int, delay_s: float = 0.1):
    cmd = f"PULSE {ch} {int(pulse_us)}"
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

# -------------------------
# Input handling
# -------------------------
def get_key():
    """Read a single keypress from stdin and return it."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)                 
        ch = sys.stdin.read(1)         
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  
    return ch

# -------------------------
# Main logic
# -------------------------
print("Waiting for READY...")
start = time.time()
ready = False
while time.time() - start < 5:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print("<<", line)
    if line == "READY":
        ready = True
        break
if not ready:
    print("Didn't see READY, continuing anyway...")

# Keep PING
print(">> PING")
print("<<", send("PING"))

# Set starting pulse
pulse = START_PULSE
do_pulse(SERVO_CHANNEL, pulse, delay_s=0.15)

print("Control with 'w'=up, 's'=down, 'q'=quit.")

try:
    while True:
        key = get_key()
        if key == "q":
            print(">> OFF")
            print("<<", send("OFF"))
            break
        elif key == "w":
            pulse += STEP
            do_pulse(SERVO_CHANNEL, pulse)
        elif key == "s":
            pulse -= STEP
            do_pulse(SERVO_CHANNEL, pulse)
        else:
            print(f"Ignored key: {repr(key)}")
finally:
    ser.close()
    print("Serial closed, exiting.")

