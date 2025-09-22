import time
from perception import Perception
from pololu_3pi_2040_robot import robot

per = Perception()
display = robot.Display()
while True:
    # data = per.line_sensor.get_raw_data()
    data = str([i for i in range(200)])
    display.fill(0)
    msg = str(data)
    display.show()