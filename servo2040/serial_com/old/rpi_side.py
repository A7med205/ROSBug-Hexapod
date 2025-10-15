# pip install pyserial
import time
import serial

# Adjust this if needed; use: ls /dev/ttyACM* to find the device
PORT = "/dev/ttyACM0"
BAUD = 115200  # For USB CDC it's largely ignored but must be set

ser = serial.Serial(PORT, BAUD, timeout=1)

def send(cmd):
    ser.write((cmd + "\n").encode("utf-8"))
    # Read one reply line (protocol is one line per command)
    line = ser.readline().decode(errors="ignore").strip()
    return line

# Wait for board to announce it's ready
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
print(">>", "PING")
print("<<", send("PING"))

def do_pulse(pulse_us, delay_s=1.0):
    cmd = f"PULSE {pulse_us}"
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

# Replace all other commands with PULSE() calls

# Mid position (approx 1500us)
do_pulse(1500, delay_s=1.0)

# Move a bit to one side
do_pulse(1700, delay_s=1.5)

# Move a bit to the other side
do_pulse(1300, delay_s=1.5)

# Simple "sweep-like" demonstration by alternating pulses
for _ in range(4):
    do_pulse(1200, delay_s=0.75)
    do_pulse(1800, delay_s=0.75)

# Disable by sending 0us (board-side .pulse(0) disables)
do_pulse(0, delay_s=0.5)

ser.close()
