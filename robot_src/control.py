## @package control
#
# Module containing the general modal controller and specific control algorithms

from pololu_3pi_2040_robot.motors import Motors
import time

from perception import Perception
from navigation import Navigation

MEDIUM_WHEEL_BASE = 85 # Medium wheel base in mm

## Enum for modes of the ModeController, that is the different specific control algorithms.
class ControlMode:
    Line = "Line"
    Kinematic = "Kinematic"
    Path = "Path"
    Position = "Position"
    Inactive = "Inactive"




## Main Controller to wrap all control tasks.
#
# This class wraps all individual controller implementations to expose an easy interface to the guidance module.
# The specific control algorithm (ControlMode) can be set, which will then be executed when run() gets called.
class ModeController:
    def __init__(self, perception: Perception, navigation: Navigation):
        self._perception = perception
        self._navigation = navigation
        self._mode: ControlMode = ControlMode.Inactive
        self._motors = Motors()

        self.line_follower = LineFollower(self._motors, perception)
        self.kinematic_controller = KinematicController(self._motors, self._perception)
        self.path_follower = PathFollower()
        self.position_controller = PositionController()

    ## Select a specific control algorithm.
    #
    # @param mode The control mode to activate.
    def set_mode(self, mode: ControlMode):
        self._mode = mode

    ## Execute the selected control algorithm.
    def run(self):
        if self._mode == ControlMode.Inactive:
            self._motors.off()

        elif self._mode == ControlMode.Kinematic:
            self.kinematic_controller.run()

        elif self._mode == ControlMode.Line:
            self.line_follower.run()

        elif self._mode == ControlMode.Path:
            self.path_follower.run()

        elif self._mode == ControlMode.Position:
            self.position_controller.run()


## Controller for following the black parcours line.
class LineFollower:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

    def run(self):
        deviation = self._perception.get_line_deviation()

        THRESHOLD = 50000
        TURNSPEED = 600
        FORWARDSPEED = 600

        if deviation >= THRESHOLD:
            self._motors.set_speeds(FORWARDSPEED + TURNSPEED, FORWARDSPEED - TURNSPEED)
        elif deviation <= -THRESHOLD:
            self._motors.set_speeds(FORWARDSPEED - TURNSPEED, FORWARDSPEED + TURNSPEED)
        else:
            self._motors.set_speeds(FORWARDSPEED, FORWARDSPEED)


## Controller to attain a given movement of forward speed and turning rate.
class KinematicController:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

        self.forward_speed = 0
        self.turn_speed = 0
        self.manual_sensitivity_fwd = 100
        self.manual_sensitivity_rot = 50

        self.prevT = 0
        self.initialT = 0

        # PI parameters
        self.Kp = 1
        self.Ki = 0

        self.iLeft = 0
        self.iRight = 0

        self.maxWheelSpeed = 6000
        self.safetyFactor = 0.8

    ## Set movement setpoint.
    #
    # @param v forward speed
    # @param w turn rate
    def set_vw(self, v: float, w: float):
        pass

    def increase_v(self):
        self.forward_speed += self.manual_sensitivity_fwd
        return self.forward_speed

    def decrease_v(self):
        self.forward_speed -= self.manual_sensitivity_fwd
        return self.forward_speed

    def increase_w(self):
        self.turn_speed += self.manual_sensitivity_rot
        return self.turn_speed

    def decrease_w(self):
        self.turn_speed -= self.manual_sensitivity_rot
        return self.turn_speed

    def run(self):
        if (self.prevT == 0):
            self.initialT = time.time_ns()
            self.prevT = self.initialT
            return


        # Reference speeds
        refLeft = max(self.forward_speed - (self.turn_speed*MEDIUM_WHEEL_BASE/2), self.safetyFactor * self.maxWheelSpeed)
        refRight = max(self.forward_speed + (self.turn_speed*MEDIUM_WHEEL_BASE/2), self.safetyFactor * self.maxWheelSpeed)

        eLeft = self._perception.get_wheel_speed_left() - refLeft
        eRight = self._perception.get_wheel_speed_right() - refRight

        # Calculation of integral part
        dT = time.time_ns() - self.prevT
        T = time.time_ns() - self.initialT

        self.iLeft += eLeft*dT
        self.iRight += eRight*dT


        # Manipulated variables
        mLeft = self.Kp * eLeft + self.Ki * (self.iLeft/T)
        mRight = self.Kp * eRight + self.Ki * (self.iRight/T)

        self._motors.set_speeds(max(refLeft + mLeft, self.maxWheelSpeed),
                                max(refRight + mRight, self.maxWheelSpeed))


## Controller to follow a polynomial path between a start and target pose.
class PathFollower:
    def __init__(self):
        pass

    def run(self):
        pass

## Controller implementing a position control algorithm.
class PositionController:
    def __init__(self):
        self.target = (0, 0)

    ## Set target position with x and y coordinate.
    def set_position(self, x, y):
        self.target = (x, y)
        
    def run(self):
        pass
