import pigpio
import time

servo_pin = 18  # GPIO 18
pi = pigpio.pi()

# Set PWM pulse width (1000–2000 μs typical for 0°–180°)
pi.set_servo_pulsewidth(servo_pin, 1500)  # Middle position
time.sleep(1)

pi.set_servo_pulsewidth(servo_pin, 1000)  # 0 degrees
time.sleep(1)

pi.set_servo_pulsewidth(servo_pin, 2000)  # 180 degrees
time.sleep(1)

pi.set_servo_pulsewidth(servo_pin, 0)  # Turn off the servo
pi.stop()
