#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor  # Import the Motor class
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

class Robot:

    def __init__(self):
        motor_fw_L = Motor(Port.A)  
        motor_fw_R = Motor(Port.B)   
        motor_bw_L = Motor(Port.C)  
        motor_bw_R = Motor(Port.D)   
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
