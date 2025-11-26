## @package control
#
# Module containing the general modal controller and specific control algorithms

from pololu_3pi_2040_robot.motors import Motors
import time

from perception import Perception
from navigation import Navigation
import parameters
import math

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

        # PD gains (tune as needed)
        self.kp = 10000.0
        self.kd = 0.01
        self.dt = 0.01
        self.prev_e = 0

        # Base forward PWM
        self.duty_cycle = 0.50
        self.v0 = 600 * self.duty_cycle  # PWM

        # Maximum fraction of v0 to use for steering
        self.max_steer_fraction = 0.35  # never more than 50% of v0

    def run(self):
        # 1) Get line deviation in radians
        error = self._perception.get_line_deviation()

        # 2) Derivative
        derr = (error - self.prev_e) / self.dt

        # 3) PD output in radians
        ctrl = self.kp * error + self.kd * derr

        # 4) Convert to PWM steering
        STEER_GAIN = 1000  # start small, tune later
        w = ctrl * STEER_GAIN

        # 5) Cap steering to a fraction of forward speed
        max_w = self.v0 * self.max_steer_fraction
        w = max(min(w, max_w), -max_w)

        # 6) Compute motor PWM
        left = self.v0 - w
        right = self.v0 + w

        # Clip just in case
        left = max(min(left, 6000), -6000)
        right = max(min(right, 6000), -6000)

        # 7) Send to motors
        self._motors.set_speeds(left, right)

        # 8) Store previous error
        self.prev_e = error


## Controller for following the black parcours line.
class LineFollower:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

        # PD gains (tune as needed)
        # Actual 'functional' values
        self.kp = 0.006
        self.kd = 0.0000002

        self.dt = 0.05
        self.prev_e = 0

        # Base forward PWM
        self.duty_cycle = 0.12
        self.v0 = 6000 * self.duty_cycle  # PWM
        self.ctrl = 0

    def run(self):
        # Get line deviation in mm

        ## As the reference = 0, the error would be -(deviation).
        error = -self._perception.get_line_deviation()
        der = (error - self.prev_e) / self.dt  # mm/s

        # PD output in mm/s : w
        self.ctrl = self.kp * error + self.kd * der

        # Motor PWM
        left = self.v0 - self.ctrl
        right = self.v0 + self.ctrl

        # Clip just in case, use safety factor
        safe = 0.9
        left = max(min(left, 6000 * safe), -6000 * safe)
        right = max(min(right, 6000 * safe), -6000 * safe)

        # Send to motors
        self._motors.set_speeds(left, right)

        # Store previous error
        self.prev_e = error

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
