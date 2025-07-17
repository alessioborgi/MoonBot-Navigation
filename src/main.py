#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor  # Import the Motor class
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import math

class r_Motor:
    def __init__(self, port, invert):
        self.motor = Motor(port)
        # Some wheel orientations need inversion
        self.invert = -1 if invert else 1

    def run(self, speed):
        # speed in degrees/sec
        self.motor.run(self.invert * speed)

    def stop(self, stop_type=Stop.BRAKE):
        self.motor.stop(stop_type)


class Robot:
    def __init__(self, wheel_radius=3, length=25, width=22):
        # wheel_radius, length between front-back, width between left-right, all in cm
        self.r = wheel_radius
        self.l = length
        self.w = width
        # Create motors: fl, fr, bl, br; invert flags based on orientation
        """self.motors = [
            r_Motor(Port.D, invert=True),  # Front Left
            r_Motor(Port.A, invert=True),   # Front Right
            #r_Motor(Port.C, invert=False),  # Back Left
            #r_Motor(Port.B, invert=False)    # Back Right
        ]"""
        self.leftMotor = r_Motor(Port.D, invert=True)
        self.rightMotor = r_Motor(Port.A, invert=True)


    def move(self, v, w=0):
        """
        v: translational speed magnitude in cm/s (0..100)
        w: rotational speed in rad/s (positive = CCW)
        """
        # kinematic factor
        R = self.r
        # distance factor
        D = math.sqrt(self.l**2 + self.w**2)/2 # half diagonal of the robot
        
        # wheel angular velocities in rad/s
        omegas = [
        # fl, fr, bl, br
            (v-w*D)/R,  # Front Left
            (v+w*D)/R,  # Front Right
            (v-w*D)/R,  # Back Left
            (v+w*D)/R   # Back Right
        ]
        speed = [math.degrees(omega) for omega in omegas]  # convert to degrees/sec

        # convert to deg/s and run
        for m, s in zip(self.motors, speed):
            m.run(s)
    """
    def turn(self, speed, direction):
        if direction == "left":
            fl = -speed
            fr = speed
            
        if direction == "right":
            fl = speed
            fr = -speed
    def forward(self, speed):
            fl = v
            fr = v
    """
        
        
    
    def stop(self):
        for m in self.motors:
            m.stop()

if __name__ == "__main__":
    robot = Robot()

    v = 0
    w = math.radians(45)  # 45 degrees per second
    i = 0
    while True:
        robot.move(v, w)
        wait(100)
        # angle += math.radians(5)
        i += 1
        if i % 50 == 0:  # every 5 seconds
            robot.stop(Stop.COAST)  # stop the robot
            break

    # To stop: robot.stop()
