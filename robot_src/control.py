## @package control
#
# Module containing the general modal controller and specific control algorithms

from pololu_3pi_2040_robot.motors import Motors
import time
import parameters

from perception import Perception
from navigation import Navigation

from control_test import record_test_data


from machine import Pin, UART

## Enum for modes of the ModeController, that is the different specific control algorithms.
class ControlMode:
    Line = "Line"
    Kinematic = "Kinematic"
    Path = "Path"
    Position = "Position"
    Inactive = "Inactive"


uart_int = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))

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
        self.manual_sensitivity_rot = -5

        self.prevT = 0
        self.initialT = 0

        self.prevELeft = 0
        self.prevERight = 0

        # PID parameters
        self.KpR = 0.5
        self.KiR = 10
        self.KdR = 0.5

        self.KpL = 0.5
        self.KiL = 10
        self.KdL = 0.5

        self.iLeft = 0
        self.iRight = 0

        self.maxWheelSpeed = 6000
        self.safetyFactor = 0.8

        # Debugging variables
        # self.data = [] # [ref, devL, devR, speedL, speedR, deltaSpeed, deltaYMCalc]
        # self.apPrevT = 0
        # self.ap = False
        # self.stop = False
        # self.start = True
        # self.debug = False

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

        # PID control loop
        if (self.prevT == 0):
            self.initialT = time.time_ns()
            self.prevT = self.initialT
            return

        # Reference speeds
        refLeft = self.forward_speed - (self.turn_speed*parameters.ROBOT_WHEEL_DISTANCE/2)
        refRight = self.forward_speed + (self.turn_speed*parameters.ROBOT_WHEEL_DISTANCE/2)

        eLeft = refLeft - self.yMLeft(self._perception.get_wheel_speed_left())
        eRight = refRight - self.yMRight(self._perception.get_wheel_speed_right())

        #uart_int.write(f"Right: (ref: {refRight}, actual: {self._perception.get_wheel_speed_right()}, actual(calc): {self.yMRight(self._perception.get_wheel_speed_right())}, err: {eRight})\nLeft: (ref: {refLeft}, actual: {self._perception.get_wheel_speed_left()}, actual(calc): {self.yMLeft(self._perception.get_wheel_speed_left())}, err: {eLeft})\n\n")

        # Calculation of integral part
        dT = time.time_ns() - self.prevT
        T = time.time_ns() - self.initialT

        self.iLeft += eLeft*dT
        self.iRight += eRight*dT

        self.prevT = time.time_ns()

        # Manipulated variables
        mLeft = refLeft + self.KpL * eLeft + self.KiL * (self.iLeft/T) - self.KdL*(eLeft-self.prevELeft)
        mRight = refRight + self.KpR * eRight + self.KiR * (self.iRight/T) - self.KdR*(eRight-self.prevERight)

        self.prevELeft = eLeft
        self.prevERight = eRight

        # Set motor speeds
        self._motors.set_speeds(max(min(mLeft, self.safetyFactor * self.maxWheelSpeed), -self.safetyFactor * self.maxWheelSpeed),
                                max(min(mRight, self.safetyFactor * self.maxWheelSpeed), -self.safetyFactor * self.maxWheelSpeed))

    # yM (for the right motor) as a function of right wheel speed
    def yMRight(self, wheel_speed_right):
        if wheel_speed_right < 0:
            return -0.004496663911564471*wheel_speed_right*wheel_speed_right*wheel_speed_right -0.3495643763999523*wheel_speed_right*wheel_speed_right + 53.6919009995278*wheel_speed_right -165.18648593231
        elif wheel_speed_right > 0:
            return wheel_speed_right * 1700/(29.38887) - 300*(29.59929)/1700
        return 0

    # yM (for the left motor) as a function of left wheel speed
    def yMLeft(self, wheel_speed_left):
        if wheel_speed_left < -0:
            return -0.00199006068026202*wheel_speed_left*wheel_speed_left*wheel_speed_left -0.135572309354336*wheel_speed_left*wheel_speed_left + 56.2486786207565*wheel_speed_left -188.22243084817
        elif wheel_speed_left > 0:
            return wheel_speed_left * 1700/(29.59929-1.052107) - 300*(29.59929-1.052107)/1700
        return 0

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
