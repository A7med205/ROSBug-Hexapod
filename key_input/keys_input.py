import sys
import tty
import termios

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
    print("Press keys (press 'q' to quit)...")
    while True:
        key = get_key()
        if key == "q":
            print("Exiting...")
            break
        elif key == "\x1b":  # ESC or start of arrow key sequence
            # Read two more chars for arrow keys
            next1 = get_key()
            next2 = get_key()
            if next1 == "[":
                if next2 == "A":
                    print("Up arrow")
                elif next2 == "B":
                    print("Down arrow")
                elif next2 == "C":
                    print("Right arrow")
                elif next2 == "D":
                    print("Left arrow")
        else:
            print(f"You pressed: {repr(key)}")

