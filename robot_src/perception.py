## @package perception
#REMOVED
# Module to evaluate sensor inputs and provide necessary measurements for the other modules.
import time
import machine

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot import imu
from parameters import COUNTS_PER_REV, ROBOT_WHEEL_RADIUS
from pololu_3pi_2040_robot import yellow_led



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
        self.last_counts_l, self.last_counts_r = self.encoders.get_counts()
        self.last_time =time.ticks_ms()
        self.omega_l = 0.0
        self.omega_r =0.0
        self.displacement_l =0.0
        self.displacement_r =0.0

        #Filter zur Glättung
        self.filter_left = AvgFilter(5)
        self.filter_right = AvgFilter(5)
    def update(self):
        #Aktuelle daten aufnehmen
        counts_l, counts_r = self.encoders.get_counts()
        now = time.ticks_ms()

        #Zeitdifferenz in Sekunden
        self.dt= time.ticks_diff(now, self.last_time) /1000.0
        if self.dt <= 0:
            return#Abbruch, falls der Timer nicht stimmt
        #pass
        # Differenz der Encoder counts
        self.delta_l = counts_l - self.last_counts_l
        self.delta_r = counts_r - self.last_counts_r

        # Winkelgeschwindigkeit in rad/s berechnen
        self.omega_l = (self.delta_l / COUNTS_PER_REV) * 2 * 3.14159265 / self.dt
        self.omega_r = (self.delta_r / COUNTS_PER_REV) * 2 * 3.14159265 / self.dt

        self.displacement_l =(self.delta_l/COUNTS_PER_REV)*2 * 3.14159265
        self.displacement_r =(self.delta_r/COUNTS_PER_REV)*2 * 3.14159265

        # Gefilterte Werte speichern
        self.filter_left.update(self.omega_l)
        self.filter_right.update(self.omega_r)

        # Alte Werte aktualisieren
        self.last_counts_l = counts_l
        self.last_counts_r = counts_r
        self.last_time = now

    def get_wheel_speed_left(self):
        return self.filter_left.get_value()
        raise NotImplementedError

    def get_wheel_speed_right(self):
        return self.filter_right.get_value()
        raise NotImplementedError
    def get_wheel_distance_deviation(self) -> float:
        """
        Berechnet die Abweichung der zurückgelegten Strecke der beiden Räder.
        
        Returns:
            float: Abweichung in mm (positiv = linkes Rad weiter, negativ = rechtes Rad weiter)
        """
        counts_l, counts_r = self.encoders.get_counts()
        
        # Umrechnung Counts -> Strecke
        s_l = (counts_l / COUNTS_PER_REV) * 2 * 3.14159265 * ROBOT_WHEEL_RADIUS
        s_r = (counts_r / COUNTS_PER_REV) * 2 * 3.14159265 * ROBOT_WHEEL_RADIUS
        
        return s_l , s_r

## Class to model a line sensor to calculate the lateral deviation from
# the center of the line.
class PerceptionLineSensor:
    def __init__(self, line_sensors: robot.LineSensors):
        self.line_sensors = line_sensors
        self.weights =[-3000, -1000, 0,1000, 3000] # wichtung der Sensoren übergeben (Sensoren befinden sich in 1 cm und 3 cm Abstand)
        self.last_deviation =0

    def get_raw_data(self) -> list[int, int, int, int, int]:
        """read line sensor raw data

        Returns:
            list[int, int, int, int, int]: list of brightness values
        """
        return list(self.line_sensors.read())

    def calibrate(self):
        self.line_sensors.calibrate()

    ## Get the deviation from the center of the parcours line.
    def read_line(self) -> int:
        
        # 1) Normierte Werte direkt vom Sensor holen (0..1000)
        values = list(self.line_sensors.read_calibrated())

        total = sum(values)
        if total < 50:    # Linie verloren
            return self.last_deviation

        # 2) Gewichtete Summe
        weighted_sum = 0
        for v, w in zip(values, self.weights):
            weighted_sum += v * w

        raw_deviation = weighted_sum // total

        # 3) Low-pass Filter (Glättung)
        alpha = 0.6   # Glättung
        deviation = int(alpha * raw_deviation + (1 - alpha) * self.last_deviation)

        # Begrenzung
        #deviation = raw_deviation
        #max(-3000, min(3000, deviation))

        self.last_deviation = deviation
        return deviation *100

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
        #self.csv_logger = CSVLogger("corner_log.csv", ["timestamp","left_speed","right_speed","z_angle","corner_detected"])
        self.imu.enable_default()
        self.led_corner = yellow_led.YellowLED()
        self._last_time_gyro = time.ticks_ms()
        self._integrated_z_angle = 0.0 #°

    ## Run all update routines of the perception module.
    def update(self):
        self.wheel_speed_filter.update()
        self.get_corner()

    def get_wheel_speed_left(self):
        return self.wheel_speed_filter.get_wheel_speed_left()

    def get_wheel_speed_right(self):
        return self.wheel_speed_filter.get_wheel_speed_right()

    def calibrate_linesensor(self):
        self.line_sensor.calibrate()

    ## Get lateral deviation from black line.
    def get_line_deviation(self) -> int:
        # todo students: improve this method
        return self.line_sensor.read_line()

    ## Get distance of obstacles to the right of the robot in mm.
    def get_distance(self):
        return self.distance_sensor.get_distance()

    ## Determine whether the robot is currently in a corner.
    #
    # @returns True if in corner
    def get_corner(self) -> bool:
        left_speed = self.wheel_speed_filter.get_wheel_speed_left()
        right_speed= self.wheel_speed_filter.get_wheel_speed_right()
        self.imu.read()

        SPEED_DIFF_THRESHOLD = 2.0

        speed_diff = abs(left_speed - right_speed)
        wheel_turning = speed_diff > SPEED_DIFF_THRESHOLD

        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time_gyro)/1000
        self._last_time_gyro = now

        self.imu.gyro.read()
        gz = self.imu.gyro.last_reading_dps[2] # Z-Achse

        self._integrated_z_angle += gz * dt #°

        ROTATIONAL_THRESHOLD = 25 # 25° Änderung zwischen zwei messungen erwwartet

        corner_detected = wheel_turning and abs(self._integrated_z_angle) >= ROTATIONAL_THRESHOLD

        if corner_detected:
            #self.led_corner.on()#LED anschalten
            self._integrated_z_angle =0.0

            return True     
        else:
            #self.led_corner.off()#LED wieder ausschalten
            return False 
        
        

    def get_wheel_distance_deviation(self) -> float:
        """
        Berechnet die Abweichung der zurückgelegten Strecke der beiden Räder.
        
        Returns:
            float: Abweichung in mm (positiv = linkes Rad weiter, negativ = rechtes Rad weiter)
        """
        counts_l, counts_r = self.encoders.get_counts()
        
        # Umrechnung Counts -> Strecke
        s_l = (counts_l / COUNTS_PER_REV) * 2 * 3.14159265 * ROBOT_WHEEL_RADIUS
        s_r = (counts_r / COUNTS_PER_REV) * 2 * 3.14159265 * ROBOT_WHEEL_RADIUS
        
        return s_l- s_r
    
    def test_gyro_loop(self):
        import time

        print("Initialisiere IMU...")
        self.imu.enable_default()   # 🔥 WICHTIG!
        time.sleep(0.1)

        print("Starte Gyroskop-Test... STRG+C zum Stoppen\n")

        try:
            while True:
                self.imu.read()  # aktualisiert gyro, acc, mag

                gx, gy, gz = self.imu.gyro.last_reading_dps

                print("Gyro (dps) | X: {:7.2f}   Y: {:7.2f}   Z: {:7.2f}".format(
                    gx, gy, gz
                ))

                time.sleep_ms(50)

        except KeyboardInterrupt:
            print("Test beendet.")


    def test_mag_loop(self):
        import time
        import math

        print("Initialisiere IMU...")
        self.imu.enable_default()
        time.sleep(0.1)

        print("Starte Magnetometer-Test... STRG+C zum Stoppen\n")

        try:
            while True:
                # Magnetometer direkt auslesen
                mx, my, mz = self.imu.mag.read()   # 🔥 korrekt!

                # Heading berechnen
                heading = math.degrees(math.atan2(my, mx))
                if heading < 0:
                    heading += 360

                print(
                    "MAG raw | X: {:7d}   Y: {:7d}   Z: {:7d}   Heading: {:6.1f}°"
                    .format(mx, my, mz, heading)
                )

                time.sleep_ms(100)

        except KeyboardInterrupt:
            print("Test beendet.")




