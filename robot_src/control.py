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
        self.path_follower = PathFollower(self.kinematic_controller)
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
            self.path_follower.run()

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
        # Actual 'functional' values

        self.kp = 0.005
        self.kd = 0.00025

        self.dt = 0.05
        self.prev_e = 0

        # Base forward PWM
        self.duty_cycle = 0.13
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
        self.manual_sensitivity_fwd = 0.5
        self.manual_sensitivity_rot = -0.1

        self.prevT = 0

        # PI parameters
        self.Kp = 0.5
        self.Ki = 2

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
        self.forward_speed = v
        self.turn_speed = w

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
            self.prevT = time.ticks_us()
            return

        # Reference speeds
        refLeft = self.forward_speed - (self.turn_speed * parameters.ROBOT_WHEEL_DISTANCE / 2)
        refRight = self.forward_speed + (self.turn_speed * parameters.ROBOT_WHEEL_DISTANCE / 2)

        eLeft = refLeft - self._perception.get_wheel_speed_left()
        eRight = refRight - self._perception.get_wheel_speed_right()

        # Calculation of integral part
        dT = time.ticks_diff(time.ticks_us(), self.prevT) / 1e6

        self.iLeft += eLeft * dT
        self.iRight += eRight * dT

        self.prevT = time.ticks_us()

        # Manipulated variables
        mLeft = refLeft + self.Kp * eLeft + self.Ki * (self.iLeft)
        mRight = refRight + self.Kp * eRight + self.Ki * (self.iRight)

        # Set motor speeds
        self._motors.set_speeds(
            max(min(self.yMLeft(mLeft), self.safetyFactor * self.maxWheelSpeed), -self.safetyFactor * self.maxWheelSpeed),
            max(min(self.yMLeft(mRight), self.safetyFactor * self.maxWheelSpeed), -self.safetyFactor * self.maxWheelSpeed))

    # yM (for the right motor) as a function of right wheel speed
    def yMRight(self, wheel_speed_right):
        if wheel_speed_right < 0:
            return -0.004496663911564471 * wheel_speed_right * wheel_speed_right * wheel_speed_right - 0.3495643763999523 * wheel_speed_right * wheel_speed_right + 53.6919009995278 * wheel_speed_right - 165.18648593231
        elif wheel_speed_right > 0:
            return wheel_speed_right * 1700 / (29.38887) - 300 * (29.59929) / 1700
        return 0

    # yM (for the left motor) as a function of left wheel speed
    def yMLeft(self, wheel_speed_left):
        if wheel_speed_left < -0:
            return -0.00199006068026202 * wheel_speed_left * wheel_speed_left * wheel_speed_left - 0.135572309354336 * wheel_speed_left * wheel_speed_left + 56.2486786207565 * wheel_speed_left - 188.22243084817
        elif wheel_speed_left > 0:
            return wheel_speed_left * 1700 / (29.59929 - 1.052107) - 300 * (29.59929 - 1.052107) / 1700
        return 0


## Controller to follow a polynomial path between a start and target pose.
class PathFollower:
    def __init__(self, kinematic_controller: KinematicController):
        self.s = 0
        self.ps = [200, 200, 0]
        self.pz = [0,0,0]
        self.kin_ctr = kinematic_controller

    def run(self):
        T = 10
        if self.s < 10:
            self.kin_ctr.set_vw(self.v(self.s, self.ps, self.pz, T), self.w(self.s, self.ps, self.pz, T))
            self.s += 0.1
            uart_int.write(f"s: {self.s}, v: {self.v(self.s, self.ps, self.pz, T)}, w: {self.w(self.s, self.ps, self.pz, T)}\n")
        else:
            self.kin_ctr.set_vw(0,0)
        self.kin_ctr.run()


    def x1(self, s, ps, pz):
        return s*(pz[0] - ps[0]) + ps[0]

    def dds_x1(self, s, ps, pz):
        return pz[0] - ps[0]

    def dds2_x1(self, s, ps, pz):
        return 0

    def x2(self, s, ps, pz):
        a = pz[2] + ps[2] + 2*ps[1] - 2* pz[1]
        b = 3*pz[1] - 3*ps[1] -2*ps[2] - pz[2]
        c = ps[2]
        d = ps[1]
        return a*s**3 + b*s**2 + c*s + d

    def dds_x2(self, s, ps, pz):
        a = pz[2] + ps[2] + 2*ps[1] - 2* pz[1]
        b = 3*pz[1] - 3*ps[1] -2*ps[2] - pz[2]
        c = ps[2]

        return 3*a*s**2 + 2*b*s + c

    def dds2_x2(self, s, ps, pz):
        a = pz[2] + ps[2] + 2*ps[1] - 2* pz[1]
        b = 3*pz[1] - 3*ps[1] -2*ps[2] - pz[2]

        return 3*a*s + 2*b

    def v(self, s, ps, pz, T):
        return math.sqrt(self.dds_x1(s, ps, pz)**2 + self.dds_x2(s, ps, pz)**2)/T

    def w(self, s, ps, pz, T):
        return 1/T*(self.dds_x1(s, ps, pz)*self.dds2_x2(s, ps, pz))/((self.dds_x1(s, ps, pz))**2 + (self.dds_x2(s, ps, pz))**2)

## Controller implementing a position control algorithm.
class PositionController:
    def __init__(self):
        self.target = (0, 0)

    ## Set target position with x and y coordinate.
    def set_position(self, x, y):
        self.target = (x, y)

    def run(self):
        pass
