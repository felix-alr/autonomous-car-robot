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

        #PD Parameters: TUNE
        self.kp = 0.01 # Proportional part
        self.kd = 0.1 #Derivative variable

        self.dt = 0.05 #SAMPLETIME = 50000 us
        self.prev_e = 0 #Previous error

        self.duty_cycle = 0.50 # Duty cycle of PWM of 50%
        self._wheelr = (parameters.ROBOT_WHEEL_RADIUS)/1000 #m
        self._wheeld = (parameters.ROBOT_WHEEL_DISTANCE)/1000 #m



    def run(self):
        error = self.perception.get_line_deviation()

        #Implementation of the PD
        der1 = (error-self.prev_e)/self.dt

        #Control unit 
        ctrl = (der1*self.kd)+ (self.kp*error) #Kd*s + kp

        #limit to prevent saturation
        #Assumptions/ estimate for motors max values
        #Gear ratio 75:1 - 87 rpm for pololulu motors
        f = 87/60 #rpm/60s
        maxw_w=2*math.pi*f #2pi*f
        maxv_w = maxw_w*self._wheelr #w*r

        max_wr = (2*maxv_w)/self._wheeld # 2v/d
        w = 0.8*max(min(ctrl, max_wr), -max_wr) # signal to send to the actuators
        #0.8 used for safety factor

        #MAX SPEED = 0.146
        v0 = 6000*self.duty_cycle #Constant forward speed

        #Constants for the difference bt left and right wheels
        #Still to look whether its going to be an added or multiplied ctt.
        cl = 1
        cr = 1

        left = cl*v0-w
        right = cr*v0+w
        self._motors.set_speeds(left, right)

        #Update the terms for the error
        self.prev_e = error
    


## Controller to attain a given movement of forward speed and turning rate.
class KinematicController:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

        self.forward_speed = 0
        self.turn_speed = 0
        self.manual_sensitivity_fwd = 100
        self.manual_sensitivity_rot = 50

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
        self._motors.set_speeds(
            self.forward_speed + self.turn_speed, self.forward_speed - self.turn_speed
        )


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
