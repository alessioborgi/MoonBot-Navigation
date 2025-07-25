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

    def run_time(self, speed, time, wait=False):
        self.motor.run_time(self.invert * speed, time, wait=wait)

    def stop(self, stop_type=Stop.BRAKE):
        self.motor.stop(stop_type)

    def run_angle(self, angle, wait=False):
        ''' This is the relative angle. '''
        self.motor.run_angle(10, angle, wait=wait)

    def run_target(self, target, speed=10, wait=False): 
        ''' This is the absolute angle. '''
        self.motor.run_target(speed, target, wait=wait)


class Robot:
    def __init__(self, wheel_radius=3, length=25, width=22):
        # wheel_radius, length between front-back, width between left-right, all in cm
        self.r = wheel_radius
        self.l = length
        self.w = width
        # self.steer_angle_curr = 0
        # self.steer_max_angle = 70  # Maximum steering angle in degrees
        # Create motors: fl, fr, bl, br; invert flags based on orientation
        self.leftMotor = r_Motor(Port.A, invert=False)
        self.rightMotor = r_Motor(Port.D, invert=False)
        self.steerMotor = r_Motor(Port.B, invert=False)
        self.gripperMotor = r_Motor(Port.C, invert=False)

    def move_time(self, v, time, wait=False):

        for m in [self.leftMotor, self.rightMotor]:
            m.run_time(math.degrees(v / self.r), time, wait=wait)

    def move(self, v):
        for m in [self.leftMotor, self.rightMotor]:
            m.run(math.degrees(v / self.r))

    def steer(self, angle, speed=20, wait=True):
        self.steerMotor.run_target(angle, speed=speed, wait=wait)

    def steer_left(self, angle, speed=20, wait=True):
        self.steerMotor.run_target(angle, speed=speed, wait=wait)      

    def steer_right(self, angle, speed=20):
        self.steerMotor.run_target(-angle, speed=speed, wait=True)

    def stop(self):
        for m in [self.leftMotor, self.rightMotor, self.steerMotor]:
            m.stop()

    def lower_gripper(self):
        self.gripperMotor.run_target(640, speed=200, wait=True)

    def raise_gripper(self):
        self.gripperMotor.run_target(0, speed=100, wait=True)


def grab_object(robot: Robot):

    robot.lower_gripper()
    robot.move(-10)
    wait(3000)
    robot.stop()
    robot.raise_gripper()

    robot.stop()        # Stop the robot

def main():

    VEL = -15

    # Test Path
    robot = Robot()
    robot.steer_left(40, speed=20, wait=False)
    robot.move(VEL)
    wait(6000)
    robot.steer(0)
    wait(1000)
    # sali il cratere
    robot.move(-20)
    wait(2500)
    robot.steer_left(40, speed=30, wait=False)
    robot.move(VEL)
    wait(3000)
    robot.steer(0, speed=30, wait=True)
    wait(1250)
    robot.stop()        # Stop the robot

    grab_object(robot)

    robot.stop()        # Stop the robot

def test():

    VEL = -15

    # Test Path
    robot = Robot()

    grab_object(robot)

    robot.stop()        # Stop the robot

if __name__ == "__main__":

    # main()
    test()