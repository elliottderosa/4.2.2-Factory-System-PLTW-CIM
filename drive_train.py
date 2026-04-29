import time
import can

from sparkmax_can import SparkMax, HeartbeatTask

class SparkMaxGroup:
    def __init__(self, leader, followers, inverted=False):
        self.leader = leader
        self.followers = followers
        self.inverted = inverted
        self.last = None

    def set_duty_cycle(self, value):
        value = max(-1.0, min(1.0, value))

        if self.inverted:
            value = -value

        if value == self.last:
            return
        self.last = value

        self.leader.set_duty_cycle(value)
        for f in self.followers:
            f.set_duty_cycle(value)

    def stop(self):
        self.set_duty_cycle(0.0)

bus = can.Bus(interface="socketcan", channel="can0")

"""
left = SparkMaxGroup(
    SparkMax(bus, 1),
    [SparkMax(bus, 2)]
)

right = SparkMaxGroup(
    SparkMax(bus, 3),
    [SparkMax(bus, 4)],
    inverted=True
)
"""
motor = SparkMax(bus, 1)
with HeartbeatTask(bus, enabled=True):
    while True:
        motor.set_duty_cycle(0.1)
        time.sleep(0.02)
