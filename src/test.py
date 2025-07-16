#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor  # Import the Motor class
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import math

# --- Spline interpolation utilities ---
def compute_tangents(points):
    """
    Given a list of (x, y) control points, compute a list of derivatives (tangents)
    using a finite-difference scheme for a natural cubic Hermite spline.
    """
    N = len(points)
    ms = []
    for k in range(N):
        if k == 0:
            # forward difference at start
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
            m = dy/dx
        elif k == N-1:
            # backward difference at end
            dx = points[-1][0] - points[-2][0]
            dy = points[-1][1] - points[-2][1]
            m = dy/dx
        else:
            # central difference in the middle
            dx = points[k+1][0] - points[k-1][0]
            dy = points[k+1][1] - points[k-1][1]
            m = dy/dx
        ms.append(m)
    return ms


def spline(i, points, ms):
    """
    Evaluate the cubic Hermite spline at 'i' given control points and tangents 'ms'.
    Returns the interpolated angle (rad).
    """
    # Find which segment i falls into
    for k in range(len(points) - 1):
        i0, w0 = points[k]
        i1, w1 = points[k+1]
        if i0 <= i <= i1:
            # normalized parameter
            t = (i - i0) / (i1 - i0)
            dt = i1 - i0
            # scale tangents by dt for Hermite basis
            m0 = ms[k] * dt
            m1 = ms[k+1] * dt
            # Hermite basis functions
            h00 =  2*t**3 - 3*t**2 + 1
            h10 =      t**3 - 2*t**2 + t
            h01 = -2*t**3 + 3*t**2
            h11 =      t**3 -     t**2
            # interpolate
            return h00*w0 + h10*m0 + h01*w1 + h11*m1
    # If outside defined control range, clamp to endpoints
    if i < points[0][0]:
        return points[0][1]
    return points[-1][1]


class r_Motor:
    def __init__(self, port, invert):
        self.motor = Motor(port)
        # Invert direction if needed (True => -1)
        self.invert = -1 if invert else 1

    def run(self, speed):
        # speed in deg/s
        self.motor.run(self.invert * speed)

    def stop(self, stop_type=Stop.BRAKE):
        self.motor.stop(stop_type)


class Robot:
    def __init__(self, wheel_radius=2, length=25, width=22):
        # Geometry in cm
        self.r = wheel_radius
        self.l = length     # front-back track
        self.w = width      # left-right track
        # Motors: FL, FR, BL, BR (invert flags tuned)
        self.motors = [
            r_Motor(Port.D, invert=True),   # Front Left
            r_Motor(Port.A, invert=True),   # Front Right
            r_Motor(Port.C, invert=False),  # Back Left
            r_Motor(Port.B, invert=False)   # Back Right
        ]

    def move(self, v, w=0):
        """
        v: translational speed magnitude (cm/s)
        w: rotational speed about center (rad/s)
        """
        # kinematic factors
        R = self.r
        D = math.sqrt(self.l**2 + self.w**2) / 2
        # compute wheel angular velocities in rad/s
        omegas = [
            ( v - w*D) / R,  # FL
            ( v + w*D) / R,  # FR
            ( v - w*D) / R,  # BL
            ( v + w*D) / R   # BR
        ]
        # convert to deg/s and run
        for m, omega in zip(self.motors, omegas):
            m.run(math.degrees(omega))

    def turn(self, angle):
        """
        Turn the robot by a specified angle in radians.
        """
        w = angle / 0.1  # angular velocity for 0.1s duration
        v = 0  # no translational speed during turn
        R = self.r
        D = math.sqrt(self.l**2 + self.w**2) / 2
        # compute wheel angular velocities in rad/s
        omegas = [
            0,  # FL
            0,  # FR
            ( v - w*D) / R,  # BL
            ( v + w*D) / R   # BR
        ]
        # convert to deg/s and run
        for m, omega in zip(self.motors, omegas):
            if omega==0:
                m.stop(Stop.COAST)
            else:
                m.run(math.degrees(omega))



    def stop(self):
        for m in self.motors:
            m.stop()


if __name__ == "__main__":
    robot = Robot()

    # --- Setup spline for dynamic rotation ---
    # control_points: (iteration_index, omega in rad/s)
    # angles_control_points = [
    #     (0, 0),
    #     (25, math.radians(-45)),
    #     (75, math.radians(45)),
    #     (150, math.radians(-45))
    # ]
    # angles_tangents = compute_tangents(angles_control_points)
    
    w_control_points = [
        (0, 0),
        (20, math.radians(30)),
        (40, 0),
        # (20, math.radians(-30)),
        # (30, math.radians(-30)),
        # (50, math.radians(30)), 
        # (75, math.radians(-45)) 
    ]
    
    vel_control_points = [
        (0, 5),
        (30, 10),
        (60, 10),
        # (50, 10),  # 10 cm/s at iteration 50
        # (100, 0)   # stop at iteration 100
    ]
    
    w_tangents = compute_tangents(w_control_points)
    vel_tangents = compute_tangents(vel_control_points)

    robot.turn(math.radians(-90))  # Initial turn to face forward

    i = 0
    max_i = max(w_control_points[-1][0], vel_control_points[-1][0])

    # Run loop, updating angle via spline_a(i)
    while True:
        # a_new = spline_a(i, angles_control_points, angles_tangents)
        # w = (a_new - a) / 0.1  # compute angular velocity
        # a = a_new

        w = spline(i, w_control_points, w_tangents)  # get angular velocity from spline

        v = spline(i, vel_control_points, vel_tangents)  # update speed

        robot.move(v, w)
        wait(100)  # 0.1s
        i += 1
        # Stop after last control point
        if i > max_i:
            robot.stop()
            break

    # To manually stop at any time: robot.stop()
