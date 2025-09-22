## @package communication
#  Module to facilitate communication with an external bluetooth device
#
# The communication module implements the necessary information exchange with an external bluetooth device
# to operate the robot and send data. This is done by writing to the onboard HC-05 module connected via
# UART0 on pins 28 and 29.
#
# All communication is text-based, so a simple terminal can be used to send and receive legible data.
# Standard protocols for pose and parking spot exchange are already implemented.
# Users can further bind callback functions to keystrokes, so they will be executed when the corresponding
# key is received.

from machine import Pin, UART
from navigation import Navigation


class Mode:
    LISTENER = 1
    READER = 2

## Class to implement the serial bluetooth communication.
class Communicator:
    def __init__(self, nav: Navigation):
        self.uart: UART = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))
        self.nav = nav
        self.mode = Mode.LISTENER
        self.buf = ""
        self.bindings = {}
        self.delim = "\r\n"

        self.println("HSAMR2-Robot TUD-EuI 2025")

    ## Register multiple key - callback bindings from a dictionary.
    #
    # @param keymap The dictionary containing key - callback pairs.
    def bind_map(self, keymap: dict[str, callable]):
        self.bindings.update(keymap)

    ## Bind a callback function to a key.
    #
    # The bound function will be executed by @ref run when the key is encountered.
    def bind(self, cb: callable, key: str):
        if len(key) != 1:
            raise ValueError(
                "Only binding single keystrokes is supported at this time."
            )
        self.bindings[key] = cb

    ## Delete a binding.
    #
    # @param key The key to delete as single character string.
    def unbind(self, key: str):
        self.bindings.pop(key)

    def set_mode(self, mode: Mode):
        self.buf = ""
        self.mode = mode

    ## Process incoming data and possibly invoke a registered callback.
    def run(self):
        if self.mode == Mode.LISTENER:
            if self.uart.any():
                # this will error with invalid input!
                char = self.uart.read(1).decode("utf-8", "strict")[0]
                if char in self.bindings:
                    self.bindings[char]()

        elif self.mode == Mode.READER:
            # todo: is this functionality still necessary?
            pass

    ## Print a line of text to the connected device.
    #
    # @param buf The string to print.
    def println(self, buf):
        self.uart.write(f"{buf}{self.delim}")

    ## Send the current pose to connected bluetooth device.
    def send_pose(self):
        precision = 2
        pose = self.nav.get_pose()
        buf = f"p{self.delim}{round(pose.x, precision)}{self.delim}{round(pose.y, precision)}{self.delim}{round(pose.phi, precision)}{self.delim}"
        self.uart.write(buf)

    ## Send registered parking spots to connected bluetooth device.
    def send_spots(self):
        """send registered parking spots to connected bluetooth device"""
        spots_list = self.nav.get_parking_spots()
        self.uart.write(f"s{self.delim}")
        for (sec_idx, sec) in enumerate(spots_list): 
            for (spot_idx, spot) in enumerate(sec):
                self.uart.write(f"{sec_idx}{self.delim}{spot_idx}{self.delim}{spot[0]}{self.delim}{spot[1]}{self.delim}")
        self.uart.write(f"end{self.delim}")
        

    ## Receive the user-selected target parking spot.
    #
    # Get the target spot, which the user selected from the connected bluetooth device as a tuple of section number and index of the spot in that section.
    # @return target spot as (section, index)
    def receive_target_spot(self) -> tuple[int, int]:
        buf = self.uart.read().decode("utf-8", "strict")
        split = buf.split('\r')
        section = int(split[0])
        idx = int(split[1])
        return (section, idx)
