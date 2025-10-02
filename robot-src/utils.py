## @package utils
# Various helpers for the project.
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.battery import Battery
from pololu_3pi_2040_robot.rgb_leds import RGBLEDs

from navigation import Navigation
from perception import Perception


class Display(robot.Display):
    def __init__(self):
        super().__init__()

    ## Print text to the screen that gets automatically wrapped on line bounds.
    # @param text The text to print.
    def long_text(self, text: str):
        msg = str(text)
        for i in range(8):
            msg_line = msg[i * 16 : (i + 1) * 16]
            self.text(msg_line, 0, 8 * i, 1)

    ## Print text to a specified line.
    # @param text The text to print.
    # @param line_no The zero-indexed line number where the text should be printed.
    def text_line(self, text, line_no):
        self.text(str(text), 0, 8 * line_no)

    ## Clear the screen, print pose components linewise at the top and the measured side distance at the bottom.
    def show_pose_and_dist(self, per: Perception, nav: Navigation):
        """clear the screen, print pose components linewise at the top and the measured side distance at the bottom"""
        pose = nav.get_pose()
        self.fill(0)
        self.text(f"{pose.x:.6g}", 0, 0)
        self.text(f"{pose.y:.6g}", 0, 8)
        self.text(f"{pose.phi:.6g}", 0, 16)
        self.text(f"{per.get_distance():6g}", 0, 48)
        self.show()

## Check whether the battery level is to be considered empty.
# @return True if battery voltage is low.
def check_battery_empty(bat: Battery) -> bool:
    return bat.get_level_millivolts() < 4400

## LED to signal nominal operation with a simple animation
class HeartbeatLED:
    def __init__(self,leds: RGBLEDs, idx):
        self.leds = leds
        self.idx = idx
        self.state = 0
        self.step = 10

    def update(self):
        self.leds.set(self.idx, (0, self.state, 0))
        self.leds.show()

        self.state += self.step
        if self.state > 255:
            self.state = 255
            self.step = -self.step
        elif self.state < 0:
            self.state = 0
            self.step = -self.step

## Write data to a local csv file
class CSVLogger:
    ## Initialize the Logger
    # @param file the file path of the resulting csv
    # @param columns list of all column names
    def __init__(self, file, columns: list):
        self.file = file
        header = ",".join(columns) + "\n"
        with open(file, "w") as f:
            f.write(header)

    ## Write a datum to the file
    # @param row list of row elements
    def write(self, row: list):
        row_str = [str(i) for i in row]
        entry = ",".join(row_str) + "\n"
        with open(self.file, "+a") as f:
            f.write(entry)
