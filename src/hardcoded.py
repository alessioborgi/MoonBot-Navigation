#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor  # Import the Motor class
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import math

DT = 0.1
DBG = False

class r_Motor:
    def __init__(self, port, invert):
        self.motor = Motor(port)
        # Some wheel orientations need inversion
        self.invert = -1 if invert else 1
        self.motor.reset_angle(0)  # Reset the motor angle to 0

    def run(self, speed):
        # speed in degrees/sec
        self.motor.run(self.invert * speed)

    def stop(self, stop_type=Stop.BRAKE):
        self.motor.stop(stop_type)

    def run_angle(self, angle):
        ''' This is the relative angle. '''
        # self.motor.run_angle(10, angle, wait=True)
        if DBG:
            self.motor.run_angle(10, angle, wait=True)
        else:
            self.motor.run_angle(10, angle)

    def run_target(self, target):
        ''' This is the absolute angle. '''
        if DBG:
            self.motor.run_target(10, target, wait=True)
        else:
            self.motor.run_target(10, target)


class Robot:
    def __init__(self, wheel_radius=3, length=25, width=22):
        # wheel_radius, length between front-back, width between left-right, all in cm
        self.r = wheel_radius
        self.l = length
        self.w = width
        self.steer_angle_curr = 0
        self.steer_max_angle = 70  # Maximum steering angle in degrees
        # Create motors: fl, fr, bl, br; invert flags based on orientation
        """self.motors = [
            r_Motor(Port.D, invert=True),  # Front Left
            r_Motor(Port.A, invert=True),   # Front Right
            #r_Motor(Port.C, invert=False),  # Back Left
            #r_Motor(Port.B, invert=False)    # Back Right
        ]"""
        self.leftMotor = r_Motor(Port.A, invert=False)
        self.rightMotor = r_Motor(Port.D, invert=False)
        self.steerMotor = r_Motor(Port.B, invert=False)
        self.angle = 0


    def move(self, v):
        for m in [self.leftMotor, self.rightMotor]:
            m.run(math.degrees(v / self.r))

    def steer(self, angle):
        clamped_angle = max(-self.steer_max_angle, min(self.steer_max_angle, angle))
        self.steerMotor.run_target(clamped_angle)

    def steer_left(self, angle):
        self.steerMotor.run_target(angle)      

    def steer_right(self, angle):
        self.steerMotor.run_target(-angle)

    def stop(self):
        for m in [self.leftMotor, self.rightMotor, self.steerMotor]:
            m.stop()


if __name__ == "__main__":

    vel = 15

    # Test Path
    robot = Robot()
    robot.move(vel)
    robot.steer_left(50)
    robot.steer(0)
    wait(1000)
    robot.move(vel)
    wait(3000)
    robot.move(vel)
    robot.steer_left(vel)
    wait(4000)



    # robot.steer_right(50)

    # robot.move(20)  # Move forward at 20 cm/s
    # wait(200)        # Wait for 0.2 seconds
    robot.stop()        # Stop the robot