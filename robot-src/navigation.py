## @package navigation
#
# Module to implement localization of the robot and parking spot detection.
from math import sin, cos, atan2, sqrt, pi
import time
import math

from pololu_3pi_2040_robot import robot
from parameters import ROBOT_WHEEL_DISTANCE, ROBOT_WHEEL_RADIUS, COUNTS_PER_REV
from perception import Perception

RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0


## Struct representing the robot pose.
#
# The pose consists of a position in x-y-coordinates in mm and
# an orientation angle phi in degrees.
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
        self.COUNTS_TO_DISTANCE = 2 * math.pi * ROBOT_WHEEL_RADIUS / COUNTS_PER_REV

    ## Update the pose estimation using the wheel encoders.
    #
    # @returns the driven distance since the last update
    def update(self):
        counts_l, counts_r = self.encoders.get_counts()

        delta_l = counts_l - self.last_counts_left
        delta_r = counts_r - self.last_counts_right

        self.last_counts_left = counts_l
        self.last_counts_right = counts_r

        dxl = self.COUNTS_TO_DISTANCE * delta_l
        dxr = self.COUNTS_TO_DISTANCE * delta_r

        ds = (dxr + dxl) / 2.0
        dphi_rad = (dxr - dxl) / ROBOT_WHEEL_DISTANCE
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
        self.parking_spots: list[list[tuple[int, int]]] = []
        self.parcours = self.get_map()

    ## Return a map of the parcours.
    #
    # @returns the map of the parcours as list of Lines
    def get_map(self):
        # To be implemented!
        # parcours = [
        #     Line(...),
        #     Line(...),
        #     ...
        # ]
        # When finished, remove the following line.
        parcours = None
        return parcours

    ## Run all the necessary internals to update the navigation.
    # 
    # Should be periodically called in the main state machine.
    def update(self):
        self.pose_filter.update()

        # Add further function calls to be executed here.

    ## Return the current Pose.
    def get_pose(self):
        return self.pose

    ## Return the current position.
    #
    # @returns a tuple of x and y coordinates
    def get_position(self):
        return (self.pose.x, self.pose.y)

    ## Reset the module to assert the robot is located in the starting pose.
    def reset(self):
        self.pose_filter.reset()

    ## Return all perceived parking spots.
    #
    # @returns list[list[tuple(int, int)]] where the outer list represents each segment,
    # which in turn can have multiple parking spots in the inner list. Each spot is characterized
    # by a tuple of a local start and end coordinate expressed in millimeters.
    # Detected spots in the third and fourth segment of 20 cm length
    # could for example look like this:
    # [[],[],[(50, 250)],[(100, 300)],[],[]]
    def get_parking_spots(self):
        return self.parking_spots

    ## Scan for available parking spots on the side.
    def scan_parking_spots(self):
        pass

