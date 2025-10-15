import time
import serial # pip install pyserial

# Adjust this if needed; use: ls /dev/ttyACM* to find the device
PORT = "/dev/ttyACM0"
BAUD = 115200  # For USB CDC it's largely ignored but must be set

ser = serial.Serial(PORT, BAUD, timeout=1)

def send(cmd):
    ser.write((cmd + "\n").encode("utf-8"))
    # Read one reply line (protocol is one line per command)
    line = ser.readline().decode(errors="ignore").strip()
    return line

def do_pulse(pulse_us, delay_s=1.0):
    cmd = f"PULSE {pulse_us}"
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

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

# Set pulse value range
angles_up = list(range(500, 2510, 10))
angles_down = list(range(2490, 490, -10))  # Avoid repeating 180
angles = angles_up + angles_down

# Move the servo through the generated angles
for angle in angles:
    do_pulse(angle, delay_s=0.005)

# Disable by sending 0us (board-side .pulse(0) disables)
do_pulse(0, delay_s=0.5)

ser.close()

