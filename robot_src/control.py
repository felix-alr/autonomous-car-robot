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
            self.path_follower.run()   # CHANGE BACK TO KINEMATIC CONTROLLER --------------------------------------- !!!

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

        self.forward_speed = 0     # in mm/s  (forward speed of the robot itself)
        self.turn_speed = 0        # in rad/s (turn speed of the robot itself)

        self.manual_sensitivity_fwd = 10
        self.manual_sensitivity_rot = -math.pi/4

        # PI parameters
        self.kp = 0.15
        self.ki = 0.4

        self.i_left = 0
        self.i_right = 0

        self.prev_t = 0

        # Safety parameters
        self.max_motor_speed_value = 6000
        self.safety_factor = 0.8

    ## Set movement setpoint.
    #
    # @param v forward speed of the robot in mm/s
    # @param w turn rate of the robot in rad/s
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
        # Convert desired forward speed in mm/s to required wheel turn speed in rad/s
        v_w = self.forward_speed / parameters.ROBOT_WHEEL_RADIUS
        # Convert desired turn speed of the robot in rad/s to required wheel turn speed in rad/s
        omega_w = self.turn_speed * (parameters.ROBOT_WHEEL_DISTANCE/parameters.ROBOT_WHEEL_RADIUS)

        # PID control loop
        if (self.prev_t == 0):
            self.prev_t = time.ticks_us()
            return

        # Reference speeds
        refLeft = v_w - (omega_w/2)
        refRight = v_w + (omega_w/2)

        # Errors
        eLeft = refLeft - self._perception.get_wheel_speed_left()
        eRight = refRight - self._perception.get_wheel_speed_right()

        # Integral part
        dT = time.ticks_diff(time.ticks_us(), self.prev_t) / 1e6

        self.i_left += eLeft * dT
        self.i_right += eRight * dT

        self.prev_t = time.ticks_us()

        # Manipulated variables
        mLeft = refLeft + self.kp * eLeft + self.ki * (self.i_left)
        mRight = refRight + self.kp * eRight + self.ki * (self.i_right)

        # Set motor speeds
        self._motors.set_speeds(
            max(min(self.yMLeft(mLeft), self.safety_factor * self.max_motor_speed_value), -self.safety_factor * self.max_motor_speed_value),
            max(min(self.yMLeft(mRight), self.safety_factor * self.max_motor_speed_value), -self.safety_factor * self.max_motor_speed_value))

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
        self.ps = [0, 0, 0]
        self.pz = [200,200,0]
        self.kin_ctr = kinematic_controller

        # Time parameters for measurement
        self.prevT = 0

    def run(self):
        if self.s < 1:
            if self.prevT == 0:
                self.prevT = time.ticks_us()
            else:
                dT = time.ticks_diff(time.ticks_us(), self.prevT) / 1e6  # delta time in seconds
                dx = dT * self.v(self.s, self.ps, self.pz)
                ds = self.get_delta_s(dx, self.s, self.ps, self.pz)
                self.s += ds
                uart_int.write(f"s: {self.s}, dds_x2(s): {self.dds_x2(self.s, self.ps, self.pz)}, delta s: {ds}, delta x: {dx}\n")

            self.kin_ctr.set_vw(self.v(self.s, self.ps, self.pz), self.w(self.s, self.ps, self.pz))
            uart_int.write(f"s: {self.s}, v: {self.v(self.s, self.ps, self.pz)}, w: {self.w(self.s, self.ps, self.pz)}\n")
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

    def v(self, s, ps, pz):
        return 50 * math.copysign(1, (pz[1]-ps[1]))

    def w(self, s, ps, pz):
        return (self.dds_x1(s, ps, pz)*self.dds2_x2(s, ps, pz) - self.dds2_x1(s, ps, pz)*self.dds_x2(s, ps, pz))/((self.dds_x1(s, ps, pz))**2 + (self.dds_x2(s, ps, pz))**2)

    def get_delta_s(self, delta_x, s, ps, pz):
        return math.sqrt(delta_x**2 - (self.dds_x2(s, ps, pz))**2)

## Controller implementing a position control algorithm.
class PositionController:
    def __init__(self):
        self.target = (0, 0)

    ## Set target position with x and y coordinate.
    def set_position(self, x, y):
        self.target = (x, y)

    def run(self):
        pass
