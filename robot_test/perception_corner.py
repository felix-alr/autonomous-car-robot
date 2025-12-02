import time
from perception import Perception

p = Perception()
p.imu.enable_default()

while True:
    # Test Gyro / Wheel-Differenz
    p.wheel_speed_filter.filter_left.update(5.0)
    p.wheel_speed_filter.filter_right.update(0.0)

    # Gyro simulieren
    p._integrated_z_angle += 30.0 * 0.1  # Simuliert 30°/s über 0.1 s
    corner = p.get_corner()
    print("Corner erkannt:", corner)
    time.sleep(0.1)
