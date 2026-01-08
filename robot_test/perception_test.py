# This test file serves the purpose of testing specific features.
from perception import Perception
import time

p = Perception()

print("Starte Kalibrierung…")
p.calibrate_linesensor()
while True:
    p.update()  # wichtig!
    "values:", p.line_sensor.line_sensors.read_calibrated()
    time.sleep(0.1)



