import pigpio
import time

# Initializes a Raspberry PI
pi = pigpio.pi()

# Debug info
print(pi.connected)

# The pin connected to the signal of the motor you want to control
pin = 18

# Sets the mode of the pin to output
pi.set_mode(pin, pigpio.OUTPUT)

# Helper stop function
# This slowly decreases the speed of the motor so it doesn't cut out
def stop(current):
    now = current
    while now > 1500:
        now -= 2
        pi.set_servo_pulsewidth(18, now)
        time.sleep(0.02)
    pi.set_servo_pulsewidth(pin, 1500)

# Sets the motor to go at a slow speed
pi.set_servo_pulsewidth(pin, 1700)
time.sleep(5)

# Stops the motor slowly
stop(1700)

# Failsafe immediate stop
pi.set_servo_pulsewidth(pin, 1500)
time.sleep(2)
