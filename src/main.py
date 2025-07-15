#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor  # Import the Motor class
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

class r_Motor:
    def __init__(self, port, is_fw, is_L):
        self.motor = Motor(port)
        self.is_fw = is_fw
        self.is_L = is_L

    def run(self, speed):
        if self.is_fw:
            self.motor.run(-speed)
        else:
            self.motor.run(speed)

    def stop(self, stop_type):
        self.motor.stop(stop_type)

class Robot:

    def __init__(self):
        motor_fw_L = r_Motor(Port.D, True, True)
        motor_fw_R = r_Motor(Port.A, True, False)  # Forward right motor 
        motor_bw_L = r_Motor(Port.C, False, True)
        motor_bw_R = r_Motor(Port.B, False, False)   
        self.motors = [motor_fw_L, motor_fw_R, motor_bw_L, motor_bw_R]

    def run_motors(self, speed):
        for motor in self.motors:
            motor.run(speed)

    def stop_motors(self):
        for motor in self.motors:
            motor.stop(Stop.BRAKE)


if __name__ == "__main__":
    robot = Robot()  # Create an instance of the Robot class
    robot.run_motors(360)  # Run all motors at 360 degrees per second
    wait(5000)  # Let the motors run for approximately 5 seconds
    robot.stop_motors()  # Stop all motors
    wait(2000)  # Wait for 2 seconds before ending the program
    robot.run_motors(-360)  # Run all motors in reverse at 360 degrees per second
    wait(5000)  # Let the motors run in reverse for approximately 5 seconds
    robot.stop_motors()  # Stop all motors again
    wait(2000)  # Wait for 2 seconds before ending the program
