## @package control
#
# Module containing the general modal controller and specific control algorithms
import random

from pololu_3pi_2040_robot.motors import Motors
import time
import parameters

from perception import Perception
from navigation import RAD_TO_DEG, Navigation
import parameters
import math

#from machine import Pin, UART

from navigation import Pose


## Enum for modes of the ModeController, that is the different specific control algorithms.
class ControlMode:
    Line = "Line"
    Kinematic = "Kinematic"
    Path = "Path"
    Position = "Position"
    Inactive = "Inactive"


#uart_int = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))


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
        self.path_follower = PathFollower(self.kinematic_controller, self._navigation)
        self.position_controller = PositionController(self._perception,self._navigation,self.kinematic_controller)


    ## Select a specific control algorithm.
    #
    # @param mode The control mode to activate.
    def set_mode(self, mode: ControlMode):
        self._mode = mode

    ## Execute the selected control algorithm.
    def run(self):
        ret = False
        if self._mode == ControlMode.Inactive:
            self._motors.off()

        elif self._mode == ControlMode.Kinematic:
            self.kinematic_controller.run()


        elif self._mode == ControlMode.Line:
            self.line_follower.run()


        elif self._mode == ControlMode.Path:
            ret = self.path_follower.run()

        elif self._mode == ControlMode.Position:
            self.position_controller.run()
        return ret


## Controller for following the black parcours line.
class LineFollower:
    def __init__(self, motors: Motors, perception: Perception):
        self._motors = motors
        self._perception = perception

        # PD gains (tune as needed)
        # Actual 'functional' values

        #as comment goes the actual best values
        #kp: if langsmam + si oscila -
        #kd: if jitters - if overshoot +

        self.kp = 0.005 - (3*0.0005)   -(3*0.0001)      #0.0032                                    #3
        self.kd = 0.00025+(1*0.00005)    -   (0*0.00001)       # 0.0003                             #5

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

        #used to try and smooth possible sensor delay that was causing one single shoot
        if abs(error) < 0.5:   # mm
            der = 0
        else:
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
        self.kp = 0.5
        self.ki = 2

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

    def reset(self):
        self.prev_t = 0
        self.i_right = 0
        self.i_left = 0


## Controller to follow a polynomial path between a start and target pose.
class PathFollower:
    def __init__(self, kinematic_controller: KinematicController, navigation: Navigation):
        self.s = 0.0

        self.v_min = 5.0           # mm/s
        self.v_target = 100.0        # mm/s
        self.v_current = self.v_min # mm/s
        self.direction = 1

        self.accel_limit = 50.0    # mm/s^2

        # Distance at which we start braking (v^2 / (2*a))
        self.brake_dist = ((self.v_target - self.v_min) ** 2) / (2 * self.accel_limit) # mm


        self.kin_ctr = kinematic_controller
        self.nav = navigation
        self.prev_t = 0
        self.end_reached = False

        # Bezier curve default initialization
        self.phi_end = 0

    def run(self):
        if self.s < 1.0 and not self.end_reached:

            # Computing Delta Time
            dt = self.compute_dt()

            if dt == -1:
                return self.end_reached

            # Calculate Distance Remaining
            # approximated by the distance from the robot to P3
            curr_x, curr_y = self.get_position(self.s)
            dist_remaining = math.sqrt((self.p3[0] - curr_x) ** 2 + (self.p3[1] - curr_y) ** 2)

            # Velocity Ease-In & Ease-Out
            # Target velocity based on stopping distance
            v_limit = math.sqrt(2 * self.accel_limit * dist_remaining)
            v_req = min(self.v_target, v_limit)
            v_req = max(v_req, self.v_min)  # Don't stall before the end

            # Smooth velocity adjustment
            if self.v_current < v_req:
                self.v_current = min(v_req, self.v_current + self.accel_limit * dt)
            elif self.v_current > v_req:
                self.v_current = max(v_req, self.v_current - self.accel_limit * dt)

            # Compute Step
            # yields omega and ds
            ds, omega = self.compute_step(self.s, dt, self.v_current)
            if ds > 0.05:
                ds = 0.05
            self.s += ds

            # Termination Check
            # stop if close to end of the path
            if self.s >= 0.999 or dist_remaining < 2.0:
                self.s = 1.0
                self.v_current = 0
                self.end_reached = True

            self.kin_ctr.set_vw(self.v_current * self.direction, omega)
        else:
            self.kin_ctr.set_vw(0, 0)
            self.end_reached = True

        if self.end_reached and not self.adjust_angle():
            self.kin_ctr.run()
            return False

        self.kin_ctr.run()
        return self.end_reached

    def set_points(self, p_start, p_end):
        phi_start = p_start[2]
        self.phi_end = p_end[2]
        horizontal = math.cos(phi_start) > 1/math.sqrt(2) or math.cos(phi_start) < -1/math.sqrt(2)
        delta = (p_end[0] - p_start[0]) if horizontal else (p_end[1] - p_start[1])
        if delta < 0:
            self.direction = -1
        else:
            self.direction = 1
        self.p0 = [p_start[0], p_start[1]]
        self.p1 = [p_start[0] + delta/3*math.cos(phi_start), p_start[1] + delta/3*math.sin(phi_start)]
        self.p2 = [p_end[0] - delta/3*math.cos(self.phi_end), p_end[1] - delta/3*math.sin(self.phi_end)]
        self.p3 = [p_end[0], p_end[1]]

    def reset(self):
        self.s = 0
        self.direction = 0
        self.prev_t = 0
        self.v_current = 0
        self.end_reached = False

        # Reset KinematicController
        self.kin_ctr.i_left = 0
        self.kin_ctr.i_right = 0
        self.kin_ctr.prev_t = 0

    def set_velocity(self, v):
        self.v_target = v

    def get_velocity(self):
        return self.v_current

    def get_position(self, s):
        om_s = 1.0 - s
        x = (om_s ** 3 * self.p0[0] +
             3 * om_s ** 2 * s * self.p1[0] +
             3 * om_s * s ** 2 * self.p2[0] +
             s ** 3 * self.p3[0])
        y = (om_s ** 3 * self.p0[1] +
             3 * om_s ** 2 * s * self.p1[1] +
             3 * om_s * s ** 2 * self.p2[1] +
             s ** 3 * self.p3[1])
        return x, y

    def get_derivatives(self, s):
        s = max(0.0, min(1.0, s))

        # Pre-calculate common terms for efficiency
        om_s = 1.0 - s

        # First Derivative
        # B'(s) = 3(1-s)^2(P1-P0) + 6(1-s)s(P2-P1) + 3s^2(P3-P2)
        dx = (3 * om_s ** 2 * (self.p1[0] - self.p0[0]) +
              6 * om_s * s * (self.p2[0] - self.p1[0]) +
              3 * s ** 2 * (self.p3[0] - self.p2[0]))

        dy = (3 * om_s ** 2 * (self.p1[1] - self.p0[1]) +
              6 * om_s * s * (self.p2[1] - self.p1[1]) +
              3 * s ** 2 * (self.p3[1] - self.p2[1]))

        # Second Derivative
        # B''(s) = 6(1-s)(P2 - 2P1 + P0) + 6s(P3 - 2P2 + P1)
        ddx = (6 * om_s * (self.p2[0] - 2 * self.p1[0] + self.p0[0]) +
               6 * s * (self.p3[0] - 2 * self.p2[0] + self.p1[0]))

        ddy = (6 * om_s * (self.p2[1] - 2 * self.p1[1] + self.p0[1]) +
               6 * s * (self.p3[1] - 2 * self.p2[1] + self.p1[1]))

        return dx, dy, ddx, ddy

    def compute_step(self, s, dt, v_now):
        dx, dy, ddx, ddy = self.get_derivatives(s)

        speed_s_sq = dx ** 2 + dy ** 2
        speed_s = math.sqrt(speed_s_sq)

        # If handles are collapsed (dx/dy both 0), we set a small floor
        # to prevent division by zero while maintaining s-progression.
        if speed_s < 1e-6:
            return 0.001, 0.0

        kappa = (dx * ddy - dy * ddx) / (speed_s_sq * speed_s)

        omega = v_now * kappa
        ds = (self.v_current * dt) / speed_s

        return ds, omega

    def adjust_angle(self):
        angle = self.nav.get_pose().phi*(math.pi/180)
        error = (self.phi_end-angle)
        error = math.atan2(math.sin(error), math.cos(error))
        gain = 10
        if abs(error) > 0.025:
            self.kin_ctr.set_vw(0, error*gain)
            return False
        return True

    def compute_angle_adjustment_step(self):
        error = self.nav.get_pose().phi - self.phi_end
        gain = 2.0 # Gain for calculating omega from error
        if abs(error) < 0.025:
            return False
        omega = max(min(error * gain, 1.5), -1.5)
        self.kin_ctr.set_vw(0, omega)
        return True

    def compute_dt(self):
        now = time.ticks_us()
        if self.prev_t == 0:
            self.prev_t = now
            return -1

        dt = time.ticks_diff(now, self.prev_t) / 1e6
        self.prev_t = now

        return dt


##Seguir tuneando kv y kt y buscar como justificar uso de ctrl hiperbolico
## Controller implementing a position control algorithm.
class PositionController:
    def __init__(self, perception, navigation, kin_controler):
        self.target = (200,0)#aqui hardcordear target para pruebas
        self.nav = navigation

        self.per = perception
        self.kin = kin_controler

        #self.kv = 1.25        #gain controller
        #self.kt = 0.9         #gain controller

        #Proportional
        self.kv = 1.5        #gain controller
        self.kt = 0.9         #gain controller

        self.thresh = 2.0       #en mm



    ## Set target position with x and y coordinate.
    def set_position(self, x, y):
        self.target = (x, y)

    def run(self):
        x, y = self.nav.get_position()
        phi = self.nav.get_pose().phi #in rad

        xd, yd = self.target

        ex = xd - x
        ey = yd - y
        phi_des = math.atan2(ey,ex)

        dist = math.sqrt((ex**2)+(ey**2))

        if dist < self.thresh:
            self.kin.set_vw(0.0, 0.0)
            self.kin.run()
            return

        ephi = phi_des - phi
        ephi = math.atan2(math.sin(ephi), math.cos(ephi))

    #Control proporcional
     #   v = dist*self.kv
     #   w = self.kt*ephi

        vmax = 250        #mm/s
        wmax = math.pi/2      #rad/s

    #Control hiperbolico, just test jeje
        v = vmax*math.tanh((self.kv*dist)/vmax)
        w = wmax*math.tanh((self.kt*ephi)/wmax)*-1

        #v (mm/s) w(rad/s)
        self.kin.set_vw(v,w)
        self.kin.run()



