from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# GPIO pin assignments
in1 = DigitalOutputDevice(17)  # IN1
in2 = DigitalOutputDevice(27)  # IN2
ena = PWMOutputDevice(18)      # ENA (with PWM)

in1 = DigitalOutputDevice(22)  # IN1
in2 = DigitalOutputDevice(23)  # IN2
ena = PWMOutputDevice(13)      # ENA (with PWM)

#IN1, IN2, ENA = 17, 27, 18
#IN3, IN4, ENB = 22, 23, 13

def forward(speed=0.5):
    in1.on()
    in2.off()
    ena.value = speed  # speed between 0.0 and 1.0
    
    in3.on()
    in4.off()
    enb.value = speed  # speed between 0.0 and 1.0

def backward(speed=0.5):
    in1.off()
    in2.on()
    ena.value = speed
    
    in3.off()
    in4.on()
    enb.value = speed

def stop():
    ena.value = 0
    enb.value = 0

# Test sequence
print("Forward")
forward(1.0)
sleep(5)

print("Backward")
backward(0.7)
sleep(5)

print("Stopping")
stop()

