import pigpio
import time

pi = pigpio.pi()

print(pi.connected)

pin = 18

def stop(current):
    now = current
    while now > 1500:
        now -= 2
        pi.set_servo_pulsewidth(18, now)
        time.sleep(0.02)
    pi.set_servo_pulsewidth(18, 1500)

pi.set_mode(pin, pigpio.OUTPUT)

pi.set_servo_pulsewidth(18, 1700)
time.sleep(5)

stop(1700)

pi.set_servo_pulsewidth(18, 1500)
time.sleep(2)
