## @package navigation
#
# Module to implement localization of the robot and parking spot detection.
from math import sin, cos, atan2, sqrt, pi
import time

from pololu_3pi_2040_robot import robot
from parameters import ROBOT_AXIS_LENGTH, ROBOT_WHEEL_CIRC, COUNTS_PER_REV
from perception import Perception

RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0


## Struct representing the robot pose.
#
# The pose consists of a position in x-y-coordinates and an orientation angle phi.
class Pose:
    def __init__(self, x=0.0, y=0.0, phi=0.0):
        self.x = x
        self.y = y
        self.phi = phi


## Base class for pose estimators.
class PoseFilter:
    def __init__(self, pose: Pose):
        self.pose = pose
        self.last_time = time.ticks_ms()

    def update(self):
        raise NotImplementedError

    def reset(self):
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.phi = 0.0


## Pose estimator using the wheel encoders.
class EncoderPoseFilter(PoseFilter):
    def __init__(self, pose: Pose, encoders: robot.Encoders):
        super().__init__(pose)
        self.encoders = encoders
        self.last_counts_left, self.last_counts_right = encoders.get_counts()

    def update(self):
        counts_l, counts_r = self.encoders.get_counts()

        delta_l = counts_l - self.last_counts_left
        delta_r = counts_r - self.last_counts_right

        self.last_counts_left = counts_l
        self.last_counts_right = counts_r

        dxl = ROBOT_WHEEL_CIRC * delta_l / COUNTS_PER_REV
        dxr = ROBOT_WHEEL_CIRC * delta_r / COUNTS_PER_REV

        ds = (dxr + dxl) / 2.0
        dphi_rad = (dxr - dxl) / ROBOT_AXIS_LENGTH
        dphi = dphi_rad * RAD_TO_DEG

        phi_rad = self.pose.phi * DEG_TO_RAD

        # Trapezoidal update
        self.pose.x += (cos(phi_rad) + cos(phi_rad + dphi_rad)) * ds / 2.0
        self.pose.y += (sin(phi_rad) + sin(phi_rad + dphi_rad)) * ds / 2.0
        self.pose.phi += dphi

        return ds

## Struct representing a line in 2D-space.
class Line:
    def __init__(self, x_start, y_start, x_end, y_end):
        self.x_start = x_start
        self.y_start = y_start
        self.x_end = x_end
        self.y_end = y_end
        self.dx = x_end - x_start
        self.dy = y_end - y_start
        self.length = sqrt(self.dx**2 + self.dy**2)

## Class implementing all of the navigation functionality.
class Navigation:
    def __init__(self, per: Perception):
        self.per = per
        self.pose = Pose()
        self.pose_filter = EncoderPoseFilter(self.pose, self.per.encoders)
        self.parking_slots: list[list[tuple[int, int]]] = []
        self.parcours = self.get_map()

    def get_map(self):
        # todo is this accurate?
        # TODO: match final parcours or delete/stubify
        parcours = [
            Line(0, 0, 300, 0),
            Line(300, 0, 300, 300),
            Line(300, 300, 750, 300),
            Line(750, 300, 750, 600),
            Line(750, 600, 0, 600),
            Line(0, 600, 0, 0)
        ]
        return parcours

    ## Run all the necessary internals to update the navigation.
    # 
    # Should be periodically called in the main state machine.
    def update(self):
        self.pose_filter.update()

        # add further function calls to be executed

    ## Return the current pose.
    def get_pose(self):
        return self.pose

    ## Return the current position.
    def get_position(self):
        return (self.pose.x, self.pose.y)

    ## Reset the module to assert the robot is located in the starting pose.
    def reset(self):
        self.pose_filter.reset()

    ## Return all perceived parking spots.
    def get_parking_spots(self):
        return self.parking_slots

    ## Scan for available parking spots on the side.
    def scan_parking_spots(self):
        pass

