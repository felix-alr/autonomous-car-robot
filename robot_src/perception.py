## @package perception
#REMOVED
#Wieso soll das alles nicht funktionieren
# Module to evaluate sensor inputs and provide necessary measurements for the other modules.
import time
import machine


from pololu_3pi_2040_robot import robot
from parameters import COUNTS_PER_REV


## Averaging filter of specified size.
#
# Averaging is a simple means to smoothen a signal.
class AvgFilter:
    def __init__(self, size: int):
        """create AveragingFilter object

        Args:
            size (int): length of the filter
        """
        self.size = size
        self.values = [0.0] * size
        self.weights = [1.0] * size

    def update(self, value):
        self.values.pop(0)
        self.values.append(value)

    def clear(self):
        self.values = [0.0] * self.size

    def get_value(self) -> float:
        weighted_sum = sum(w * v for w, v in zip(self.weights, self.values))
        return weighted_sum / self.size


## Filter for calculating the angular speeds of the wheels.
class WheelSpeedFilter:
    def __init__(self, encoders: robot.Encoders):
        self.encoders = encoders

    def update(self):
        counts_l, counts_r = self.encoders.get_counts()
        pass

    def get_wheel_speed_left(self):
        raise NotImplementedError

    def get_wheel_speed_right(self):
        raise NotImplementedError

## Class to model a line sensor to calculate the lateral deviation from
# the center of the line.
class PerceptionLineSensor:
    def __init__(self, line_sensors: robot.LineSensors):
        self.line_sensors = line_sensors

    def get_raw_data(self) -> list[int, int, int, int, int]:
        """read line sensor raw data

        Returns:
            list[int, int, int, int, int]: list of brightness values
        """
        return list(self.line_sensors.read())

    def calibrate(self):
        self.line_sensors.calibrate()

    ## Get the deviation from the center of the parcours line.
    def read_line(self):
        # todo this is to be implemented by students
        raise NotImplementedError

    def read_line_reduced(self) -> int:
        """calculate bad approximation for line deviation

        Returns:
            int: deviation
        """
        values = self.get_raw_data()
        return (-3000 * values[0] + 3000 * values[-1]) // 2


## Class to read and control the Sharp GP2Y0E03 triangulation sensor.
class DistanceSensor:
    ## Initialize the distance sensor in a disabled state. Use 'activate' to enable the sensor when needed.
    def __init__(self):
        self.i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=400_000)
        self._device_addr = 0x80 >> 1
        self._reg_upper_addr = 0x5E
        self._reg_lower_addr = 0x5F
        self._shift_addr = 0x35

        self.control_pin = machine.Pin(24, mode=machine.Pin.OUT, value=1)
        self.enabled = True
        time.sleep_ms(1)
        self.shift = self.i2c.readfrom_mem(self._device_addr, self._shift_addr, 1)[0]

        # deactivate sensor by default to save power
        self.deactivate()

    ## Enable the sensor if not already enabled.
    def activate(self):
        if not self.enabled:
            self.control_pin.value(1)
            time.sleep_ms(1)  # ensure sensor is ready for I2C
            self.enabled = True

    ## Disable the sensor.
    # This may save some battery power when the sensor is not in use.
    def deactivate(self):
        if self.enabled:
            self.control_pin.value(0)
            self.enabled = False

    ## Get the measured distance.
    # @returns The measured distance in mm or -1 if there was an error.
    def get_distance(self) -> int:
        """return measured distance in mm.
        return -1 if error during measuring occurs.

        Returns:
            int: distance in mm
        """
        # automatically activate if not active
        self.activate()
        
        # dist/mm = (upper * 16 + lower) / 16 / 2^shift * 10
        try:
            dist_bytes = self.i2c.readfrom_mem(
                self._device_addr, self._reg_upper_addr, 2
            )
            return (dist_bytes[0] * 16 + dist_bytes[1]) * 10 // 16 // 2**self.shift
        except OSError:
            return -1


## Main class of the perception module exposing the interface to other modules.
class Perception:
    def __init__(self):
        self.encoders = robot.Encoders()
        self.line_sensor = PerceptionLineSensor(robot.LineSensors())
        self.wheel_speed_filter = WheelSpeedFilter(self.encoders)
        self.distance_sensor = DistanceSensor()
        self.imu = robot.IMU()
        self.imu.enable_default()

    ## Run all update routines of the perception module.
    def update(self):
        self.wheel_speed_filter.update()

    def get_wheel_speed_left(self):
        return self.wheel_speed_filter.get_wheel_speed_left()

    def get_wheel_speed_right(self):
        return self.wheel_speed_filter.get_wheel_speed_right()

    def calibrate_linesensor(self):
        self.line_sensor.calibrate()

    ## Get lateral deviation from black line.
    def get_line_deviation(self):
        # todo students: improve this method
        return self.line_sensor.read_line_reduced()

    ## Get distance of obstacles to the right of the robot in mm.
    def get_distance(self):
        return self.distance_sensor.get_distance()

    ## Determine whether the robot is currently in a corner.
    #
    # @returns True if in corner
    def get_corner(self) -> bool:
        return False
