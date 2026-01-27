## @package navigation
#
#
#
# Gruppe HSAMR01
# REMOVED 
# Matrikel-Nr.: REMOVED
#
#
# Module to implement localization of the robot and parking spot detection.

# for communication
from machine import Pin, UART


from math import sin, cos, atan2, sqrt, pi
import time
import math

from pololu_3pi_2040_robot import robot
from parameters import ROBOT_WHEEL_DISTANCE, ROBOT_WHEEL_RADIUS, COUNTS_PER_REV, ROBOT_WHEEL_RADIUS_L, ROBOT_WHEEL_RADIUS_R
from perception import Perception

RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0


CORNER_DISTANCE_THRESHOLD = 10

FILTER_MIN_DIST = 40 # minimal distance between start and end pose of parkingspot
FILTER_MAX_ANGLE = 45 # maximal angle betwenn start and end pose of parkingspot

THRESHOLD_DISTANCE_SENSOR = 100 #Threshold for detecting Parkingspot

PARKING_SPOT_SIZE = 200 # Parkplatzgröße entscheidet über Eignung der Parklücke zum parken


## Struct representing the robot pose.
#
# The pose consists of a position in x-y-coordinates in mm and
# an orientation angle phi from x to y axis in degrees.
class Pose:
    def __init__(self, x=0.0, y=0.0, phi=0.0, dist=0.0):
        self.x = x
        self.y = y
        self.phi = phi
        self.dist = dist


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
        self.pose.dist = 0.0


## Pose estimator using the wheel encoders.
class EncoderPoseFilter(PoseFilter):
    def __init__(self, pose: Pose, encoders: robot.Encoders):
        super().__init__(pose)
        self.encoders = encoders
        self.last_counts_left, self.last_counts_right = encoders.get_counts()
        self.COUNTS_TO_DISTANCE_R = 2 * math.pi * ROBOT_WHEEL_RADIUS_R / COUNTS_PER_REV
        self.COUNTS_TO_DISTANCE_L = 2 * math.pi * ROBOT_WHEEL_RADIUS_L / COUNTS_PER_REV

    ## Update the pose estimation using the wheel encoders.
    #
    # @returns the driven distance since the last update
    def update(self):
        counts_l, counts_r = self.encoders.get_counts()

        delta_l = counts_l - self.last_counts_left
        delta_r = counts_r - self.last_counts_right

        self.last_counts_left = counts_l
        self.last_counts_right = counts_r

        dxl = self.COUNTS_TO_DISTANCE_L * delta_l
        dxr = self.COUNTS_TO_DISTANCE_R * delta_r

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
    def __init__(self, x1: int, y1: int, x2: int, y2: int, suitable_for_parking: bool, region: int ):
        """create ParkingSpot object

        Args:
            x1 (int): x1-coordinate in mm
            y1 (int): y1-coordinate in mm
            x2 (int): x2-coordinate in mm
            y2 (int): y2-coordinate in mm
            suitable_for_parking (bool): suitability
            region(int): region of parkingspot can be 1, 2 or 3
        """
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.suitable_for_parking = suitable_for_parking
        self.region = region


## Class implementing all of the navigation functionality.
class Navigation:
    def __init__(self, per: Perception):
        self.has_parkingspot = False #Variable zur Zustandsspeicherung (fährt an Parklücke vorbei oder nicht)
        self.per = per
        self.has_flag = False

        self.rc_old = 0
        self.lc_old = 0

        self.closest_point = Pose()
        
        self.pose = Pose()
        self.corner_correction_enabled = True   #Variable zur Aktivierung/Deaktivierung von Eckenerkennung (für Setup)
        self.axis_lock_enabled = True #Variable zur Aktivierung/Deaktivierung der festen Koordinatenachsen
        self.set_angle_at_corner = True #Variable zur Aktivierung/Deaktivierung der Winkelaktualisierung an den Ecken
        #Kommunikation (Test)
        self.uart: UART = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))

        self.pose_filter = EncoderPoseFilter(self.pose, self.per.encoders)
        ## dictionary for saving the detected ParkingSpots using an int as key
        self.parking_spots: dict[int, ParkingSpot] = {}

        #Array for spot_id which has to be delated from parking_spots
        self.to_delete = []

        # todo students: define parcours using Line segments!
        # self.parcours = [
        #     Line(...),
        #     Line(...),
        #     ...
        # ]
        self.parcours = [
            Line(0,0,300,0),
            Line(300,0,300,300),
            Line(300,300,800,300),
            Line(800,300,800,600),
            Line(800,600,0,600),
            Line(0,600,0,0),
        ]

        self.corners = [
            Pose(0,0,0), 
            Pose(300,0,90), 
            Pose(300,300,0),
            Pose(800,300,90),
            Pose(800,600,180),
            Pose(0,600,270),
            ]
         
        self.closest_line = self.parcours[0]
        self.idx = -1
        self.dist = float('inf')

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
        self.update_pose_distance()

        # including flag for corners
        if not self.corner_correction_enabled:
        # im Setup: weder Corner noch Parklücken-Scan
            return
        
        if self.per.get_corner() == True and self.has_flag == False:    # makes shure that code gets executed once 
            self.has_flag = True
            #self.uart.write("Ecke erkannt")
            # finding closest point to current position
            self.closest_point, self.idx, self.dist = self.find_closest_point()
            # find closest line from closest point
            self.closest_line = self.parcours[self.idx]
            # set x,y to closest corner
            self.uart.write(f"{self.dist}")
            if self.dist < CORNER_DISTANCE_THRESHOLD:
                self.uart.write("Ecke erkannt")
                self.set_pose(self.closest_point.x,self.closest_point.y, self.pose.phi)    # Villeicht muss man den Winkel auch gar nicht mit setzen
            #self.uart.write(f"{self.idx}")
        #   resets the has_flag variabled
        if self.per.get_corner() == False and self.has_flag == True:
            #self.uart.write("Ecke vorbei")
            if self.dist < CORNER_DISTANCE_THRESHOLD:
            # set phi to target-angle when corner is over
                if (not self.set_angle_at_corner) and (self.idx == 3 or self.idx ==5):
                    self.set_pose(self.pose.x, self.pose.y, self.pose.phi)
                    #self.uart.write(f"Winkel wird nicht gesetzt")
                else:
                    self.set_pose(self.pose.x, self.pose.y, self.closest_point.phi)
                    #self.uart.write(f"Winkel gesetzt")
            self.has_flag = False

        if self.axis_lock_enabled == True:

            #  when x coordninate does not change 
            if self.closest_line.x_end == self.closest_line.x_start:
                #lock x coordinate 
                if self.per.get_corner() == True:
                    self.set_pose_no_sync(self.closest_line.x_start, self.pose.y, self.pose.phi)
                else:
                    self.set_pose_no_sync(self.closest_line.x_start, self.pose.y, self.closest_point.phi)
            # when y coordninate does not change 
            if self.closest_line.y_end == self.closest_line.y_start:
                # lock y coordniate
                if self.per.get_corner() == True:
                    self.set_pose_no_sync(self.pose.x, self.closest_line.y_start, self.pose.phi)
                else:
                    self.set_pose_no_sync(self.pose.x, self.closest_line.y_start, self.closest_point.phi)
           # else:
               # self.set_pose_no_sync(self.pose.x, self.pose.y, self.pose.phi)
 
            
        self.scan_parking_spots()   

        # Add further function calls to be executed here.
   
    #function to find closest corner from current position. It returns the closest point and the list index 
    def find_closest_point(self):
        min_dist = float('inf')  
        closest_point = None
        closest_idx = 0
        # Iterate through the list and determine which point has the shortest distance to the current position.   
        for idx, element in enumerate(self.corners):
            dist = sqrt((element.x - self.pose.x)**2 + (element.y - self.pose.y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_point = element
                closest_idx = idx
        return closest_point, closest_idx, min_dist

    def update_pose_distance(self):
        d = self.per.get_distance()
        if d is None:
            return
        if 0 <= d <= 2000:
            self.pose.dist = d

    # set_pose to a fixed vatue (Eckenflag)
    def set_pose(self, x:float, y:float, phi:float):
        self.pose.x = x
        self.pose.y = y
        self.pose.phi = phi
        # Synchronize the stored encoder counts with the current hardware values 
        # to prevent a false position jump after manually setting the robot pose
        self.pose_filter.last_counts_left, self.pose_filter.last_counts_right = self.per.encoders.get_counts() 

    def set_pose_no_sync(self, x:float, y:float, phi:float):
        self.pose.x = x
        self.pose.y = y
        self.pose.phi = phi

    ## Return the current Pose.
    def get_pose(self) -> Pose:
        return self.pose
    
    def print_pose(self):
        return self.pose.x , self.pose.y , self.pose.phi

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
    
    def print_parking_spot(self, id: int):
        ps = self.get_parking_spot(id)
        return(ps.x1, ps.y1, ps.x2, ps.y2, ps.suitable_for_parking)
    
    def print_pose(self, pose: Pose):
        return(pose.x, pose.y, pose.phi, pose.dist)

    ## Reset the module to assert the robot is located in the starting pose.
    def reset(self):
        self.pose_filter.reset()

    def get_counts(self):
        right_count, left_count = self.per.encoders.get_counts()
        return right_count, left_count


    ## Return all perceived parking spots.

    # @returns dict of ids and respective ParkingSpots
    def get_parking_spots(self) -> dict:
        return self.parking_spots

    ## Scan for available parking spots on the side.
    def scan_parking_spots(self):
        self.d = self.per.get_distance()
        if self.d is None:
            return
        if self.d > THRESHOLD_DISTANCE_SENSOR and self.has_parkingspot == False: #Trigger for start_pose
            self.has_parkingspot = True
            sx, sy = self.shift_along_heading(self.pose, 16)
            self.pose_start = Pose(sx, sy, self.pose.phi) # safe start pose 
        
        if self.d <= THRESHOLD_DISTANCE_SENSOR and self.has_parkingspot == True: # Trigger for end_pose
            self.has_parkingspot = False 
            sx, sy = self.shift_along_heading(self.pose, 16)
            self.pose_end = Pose(sx, sy, self.pose.phi)   # safe end pose 
            a = math.sqrt((self.pose_end.x - self.pose_start.x)**2 + (self.pose_end.y - self.pose_start.y)**2)  # calculate distance between start and end (filtering Noise)
            phi = self.angle_diff_deg(self.pose_start.phi, self.pose_end.phi)   # calculating angle between two points to filter out the corners

            if a > FILTER_MIN_DIST and phi < FILTER_MAX_ANGLE: # checking if it is real parkingspot

                if abs(self.pose_start.x - self.pose_end.x) > abs(self.pose_start.y - self.pose_end.y): # checking if parking spot is on the x side (Bereich 1)

                    # Filter out false parking spots caused by sensor noise
                    if 280 < self.pose_start.x < 380:
                        return
                    
                    region = 1

                    if self.pose_start.x - self.pose_end.x < 0:
                        self.pose_start.y = 200 # sets the y-value to a fixed preset value (line on map)
                        self.pose_end.y = 200

                    if abs(self.pose_start.x - self.pose_end.x)> PARKING_SPOT_SIZE:    #checking if parking-spot is big enough 
                        self.has_size = True    
                    else: 
                        self.has_size = False

                if abs(self.pose_start.x - self.pose_end.x) < abs(self.pose_start.y - self.pose_end.y): #checking if parking spot is on the y side (right or left) (Bereich 2 oder 3)
                    if self.pose_start.y - self.pose_end.y < 0: # checking if the parking-spot is on the right side of map (Bereich 2)
                        region = 2
                        self.pose_start.x = 900     # set the x-value to a fixed preset value (line on map)
                        self.pose_end.x = 900
                    
                    if self.pose_start.y - self.pose_end.y > 0: #checking if the parking-spot is on the left side of map (Bereich 3)
                        region = 3
                        self.pose_start.x = -100    # set the x-value to a fixed preset value (line on map)
                        self.pose_end.x = -100    

                    if abs(self.pose_start.y - self.pose_end.y) > PARKING_SPOT_SIZE:   #checking if parking spot is big enough
                        self.has_size = True
                    else:
                        self.has_size = False
                     
                spot = ParkingSpot(self.pose_start.x, self.pose_start.y, self.pose_end.x, self.pose_end.y, self.has_size, region)    #creating new spot

                #new_orient, new_line = self.spot_orientation_and_line(spot) #orientation and fixed value of the parking spot

                self.to_delete.clear()  # clearing the array of old parking spots

                # Iterate through the list of parking spots to detect overlapping spots
                for spot_id, old_spot in list(self.parking_spots.items()):

                    if old_spot.region == spot.region:
                        # checking if there is overlapping
                        if spot.region == 1:
                            if self.intervals_overlap(spot.x1, spot.x2, old_spot.x1, old_spot.x2):
                                self.to_delete.append(spot_id)
                            continue  
                        # checking if there is overlapping in region 2 or 3
                        else:
                            if self.intervals_overlap(spot.y1, spot.y2, old_spot.y1, old_spot.y2):
                                self.to_delete.append(spot_id)
                            continue

                    else:
                        continue

                for spot_id in self.to_delete:
                    # delete overlapping parkingspots from parking_spots
                    self.parking_spots.pop(spot_id, None)
                # adding new parking spot to parking_spots
                if self.parking_spots:
                    new_id = max(self.parking_spots.keys()) + 1
                else:
                    new_id = 1

                self.parking_spots[new_id] = spot
        return 


    
    # calculating angle difference from two values
    def angle_diff_deg(self, a, b):
        d = (a - b + 180) % 360 - 180
        return abs(d)
    # shifting pose for parkingspot (sensor is not mounted in the middle of the robot)
    def shift_along_heading(self, pose: Pose, dx: float):
        phi = pose.phi * DEG_TO_RAD
        return pose.x + math.cos(phi)*dx, pose.y + math.sin(phi)*dx

    def intervals_overlap(self,u1, u2, v1, v2):
        # sorting: smaller number has to be first
        lo1, hi1 = sorted((u1, u2))
        lo2, hi2 = sorted((v1, v2))
        # returens true when intervals overlap
        return (lo1 <= hi2) and (lo2 <= hi1)