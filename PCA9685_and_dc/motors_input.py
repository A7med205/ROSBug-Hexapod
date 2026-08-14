import sys
import tty
import termios
import select
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ==========================
# KEYBOARD INPUT FUNCTION (non-blocking)
# ==========================
def get_key(timeout=0.1):
    """Read a single keypress from stdin with timeout (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        else:
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ==========================
# MOTOR SETUP
# ==========================
# Left motor
in1 = DigitalOutputDevice(17)
in2 = DigitalOutputDevice(27)
ena = PWMOutputDevice(18)

# Right motor
in3 = DigitalOutputDevice(22)
in4 = DigitalOutputDevice(23)
enb = PWMOutputDevice(13)

SPEED = 0.5  # default speed

def forward(speed=SPEED):
    in1.on();  in2.off();  ena.value = speed + 0.06
    in3.on();  in4.off();  enb.value = speed - 0.06

def backward(speed=SPEED):
    in1.off(); in2.on();   ena.value = speed + 0.06
    in3.off(); in4.on();   enb.value = speed - 0.06

def left(speed=SPEED):
    in1.off(); in2.on();   ena.value = speed + 0.06
    in3.on();  in4.off();  enb.value = speed - 0.06

def right(speed=SPEED):
    in1.on();  in2.off();  ena.value = speed + 0.06
    in3.off(); in4.on();   enb.value = speed - 0.06

def stop():
    ena.value = 0
    enb.value = 0

# ==========================
# MAIN LOOP
# ==========================
if __name__ == "__main__":
    print("Hold W (forward), S (backward), A (left), D (right). Release to stop. Press Q to quit.")

    last_key = None  # remember the last movement key being held

    try:
        while True:
            key = get_key()

            if key is None:
                # No key pressed → stop motors
                stop()
                last_key = None
                continue

            key = key.lower()

            if key == "q":
                print("Exiting...")
                stop()
                break

            # Movement keys
            if key in ("w", "a", "s", "d"):
                last_key = key

            # Apply motor commands
            if last_key == "w":
                backward()
            elif last_key == "s":
                forward()
            elif last_key == "a":
                right()
            elif last_key == "d":
                left()
            else:
                stop()

    except KeyboardInterrupt:
        stop()
        print("\nInterrupted, motors stopped.")
