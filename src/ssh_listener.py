#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import math

FIFO = '/home/robot/cmdfifo'

# wait until the FIFO actually exists
while True:
    try:
        # MicroPython's uos.stat is supported
        import uos
        uos.stat(FIFO)
        break
    except OSError:
        wait(100)

# your r_Motor and Robot classes here (unchanged)…
class r_Motor:
    def __init__(self, port, invert):
        self.motor = Motor(port)
        self.invert = -1 if invert else 1
        self.motor.reset_angle(0)
    def run(self, speed):
        self.motor.run(self.invert * speed)
    def stop(self, stop_type=Stop.BRAKE):
        self.motor.stop(stop_type)
    def run_angle(self, angle):
        self.motor.run_angle(10, angle)
    def run_target(self, target):
        self.motor.run_target(10, target)

class Robot:
    def __init__(self, wheel_radius=3, length=25, width=22):
        self.r = wheel_radius
        self.steer_max_angle = 70
        self.leftMotor  = r_Motor(Port.A, invert=False)
        self.rightMotor = r_Motor(Port.D, invert=False)
        self.steerMotor = r_Motor(Port.B, invert=False)
    def move(self, v):
        # convert linear velocity v (cm/s) to deg/s
        speed = math.degrees(v / self.r)
        self.leftMotor.run(speed)
        self.rightMotor.run(speed)
    def steer(self, angle):
        angle = max(-self.steer_max_angle, min(self.steer_max_angle, angle))
        self.steerMotor.run_target(angle)
    def stop(self):
        self.leftMotor.stop()
        self.rightMotor.stop()
        self.steerMotor.stop()

robot = Robot()

# now open the pipe and listen forever
with open(FIFO, 'r') as fifo:
    while True:
        line = fifo.readline().strip()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]
        args = parts[1:]
        try:
            if   cmd == 'move':       
                robot.move(float(args[0]))
            elif cmd == 'steer':      
                robot.steer(float(args[0]))
            elif cmd == 'stop':       
                robot.stop()
            elif cmd == 'wait':       
                wait(int(args[0]))
        except Exception:
            pass
