## @package navigation
#
# REMOVED
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
# an orientation angle phi from x to y axis in degrees.
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
    def __init__(self, x_start: int, y_start: int, x_end: int, y_end: int):
        """create Line object

        Args:
            x_start (int): x1-coordinate in mm
            y_start (int): y1-coordinate in mm
            x_end (int): x2-coordinate in mm
            y_end (int): y2-coordinate in mm
        """
        self.x_start = x_start
        self.y_start = y_start
        self.x_end = x_end
        self.y_end = y_end
        self.dx = x_end - x_start
        self.dy = y_end - y_start
        self.length = sqrt(self.dx**2 + self.dy**2)


## Struct representing a parking spot.
#
# The coordinates should define corners of the rectangle representing the parking spot.
class ParkingSpot:
    def __init__(self, x1: int, y1: int, x2: int, y2: int, suitable_for_parking: bool):
        """create ParkingSpot object

        Args:
            x1 (int): x1-coordinate in mm
            y1 (int): y1-coordinate in mm
            x2 (int): x2-coordinate in mm
            y2 (int): y2-coordinate in mm
            suitable_for_parking (bool): suitability
        """
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.suitable_for_parking = suitable_for_parking


## Class implementing all of the navigation functionality.
class Navigation:
    def __init__(self, per: Perception):
        self.per = per
        self.pose = Pose()
        self.pose_filter = EncoderPoseFilter(self.pose, self.per.encoders)
        ## dictionary for saving the detected ParkingSpots using an int as key
        self.parking_spots: dict[int, ParkingSpot] = {}

        # todo students: define parcours using Line segments!
        # self.parcours = [
        #     Line(...),
        #     Line(...),
        #     ...
        # ]
        self.parcours: list[Line] = []

    ## Return a map of the parcours.
    #
    # @returns the map of the parcours as list of Lines
    def get_map(self) -> list[Line]:
        """get the parcours data

        Returns:
            list[Line]: list of lines
        """
        return self.parcours

    ## Run all the necessary internals to update the navigation.
    #
    # Should be periodically called in the main state machine.
    def update(self):
        self.pose_filter.update()

        # Add further function calls to be executed here.

    ## Return the current Pose.
    def get_pose(self) -> Pose:
        return self.pose

    ## Return the current position.
    #
    # @returns a tuple of x and y coordinates
    def get_position(self) -> tuple[float, float]:
        return (self.pose.x, self.pose.y)

    ## Adds a parking spot to the database.
    def add_parking_spot(self, id: int, parking_spot: ParkingSpot):
        self.parking_spots[id] = parking_spot

    ## Return the ParkingSpot of a matching id.
    def get_parking_spot(self, id: int) -> ParkingSpot:
        return self.parking_spots[id]

    ## Reset the module to assert the robot is located in the starting pose.
    def reset(self):
        self.pose_filter.reset()

    ## Return all perceived parking spots.
    #
    # @returns dict of ids and respective ParkingSpots
    def get_parking_spots(self) -> dict:
        return self.parking_spots

    ## Scan for available parking spots on the side.
    def scan_parking_spots(self):
        pass
