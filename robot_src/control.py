## @package control
#
# Module containing the general modal controller and specific control algorithms

from pololu_3pi_2040_robot.motors import Motors
import time
import parameters

from perception import Perception
from navigation import Navigation
import parameters
import math


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
    def _init_(self, motors: Motors, perception: Perception):
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

## Controller to attain a given movement of forward speed and turning rate.
class KinematicController:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

        self.forward_speed = 0
        self.turn_speed = 0
        self.manual_sensitivity_fwd = 100
        self.manual_sensitivity_rot = 5

        self.prevT = 0
        self.initialT = 0

        # PI parameters
        self.Kp = 0.5
        self.Ki = 0.3

        self.iLeft = 0
        self.iRight = 0

        self.maxWheelSpeed = 6000
        self.safetyFactor = 0.8

        # ref, devL, devR, speedL, speedR, deltaSpeed, deltaYMCalc
        self.data = []

        self.apPrevT = 0
        self.ap = False
        self.stop = False
        self.start = True
        self.debug = False

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
        if self.debug:
            if self.start:
                if self.apPrevT + 500000000 < time.time_ns():
                    if self.forward_speed >= -3000 and not self.stop:
                        self.forward_speed -= 500
                        self.apPrevT = time.time_ns()
                    else:
                        self.start = False
        if not self.start:
            if (self.forward_speed >= 3000):
                self.forward_speed = 0
                self.stop = True
            if (self.apPrevT + 400000000 < time.time_ns() and not self.stop):
                self.increase_v()
                self.ap = True
                self.apPrevT = time.time_ns()
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
        mLeft = self.Kp * eLeft + self.Ki * (self.iLeft/T)
        mRight = self.Kp * eRight + self.Ki * (self.iRight/T)
        # refL, refR, devL, devR, speedL, speedR, deltaSpeed, deltaYMCalc
        speedL = self._perception.get_wheel_speed_left()
        speedR = self._perception.get_wheel_speed_right()
        if self.ap:
            self.ap = False
            self.data.append([refLeft, eLeft, eRight, speedL, speedR, speedL-speedR, self.yMLeft(speedL), self.yMRight(speedR)])

        self._motors.set_speeds(max(min(refLeft + mLeft, self.maxWheelSpeed), -self.maxWheelSpeed),
                                max(min(refRight + mRight, self.maxWheelSpeed), -self.maxWheelSpeed))

    def yMRight(self, wheel_speed_right):
        if wheel_speed_right < -1:
            return -0.004496663911564471*wheel_speed_right*wheel_speed_right*wheel_speed_right -0.3495643763999523*wheel_speed_right*wheel_speed_right + 53.6919009995278*wheel_speed_right -165.18648593231
        elif wheel_speed_right > 1:
            return 0.00158962537790145*wheel_speed_right*wheel_speed_right*wheel_speed_right - 0.126700243805418*wheel_speed_right*wheel_speed_right + 63.0847083536231*wheel_speed_right + 116.819530102035
        return 0

    def yMLeft(self, wheel_speed_left):
        if wheel_speed_left < -1:
            return -0.00199006068026202*wheel_speed_left*wheel_speed_left*wheel_speed_left -0.135572309354336*wheel_speed_left*wheel_speed_left + 56.2486786207565*wheel_speed_left -188.22243084817
        elif wheel_speed_left > 1:
            return 0.000609007723240748*wheel_speed_left*wheel_speed_left*wheel_speed_left - 0.0026257455235966*wheel_speed_left*wheel_speed_left + 58.0932190955994*wheel_speed_left + 167.694825200004
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
