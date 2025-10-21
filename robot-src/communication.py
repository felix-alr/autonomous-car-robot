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
from navigation import Navigation, ParkingSpot


## Class to implement the serial bluetooth communication.
class Communicator:
    def __init__(self, nav: Navigation):
        self.uart: UART = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))
        self.nav = nav
        self.buf = ""
        self.bindings = {}
        self.delim = "\r\n"
        self.target_pos: tuple[int, int] | None = None

        self.println("HSAR-Robot TUD-EuI 2025")

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


    ## Process incoming data and possibly invoke a registered callback.
    def run(self):
        if self.uart.any():
            # this will error with invalid input!
            char = self.uart.read(1).decode("utf-8", "strict")[0]
            if char in self.bindings:
                self.bindings[char]()


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
    #
    # Write all parking spots to the serial connection in the format
    # 
    # s
    # <id: int>
    # <x1: int>
    # <y1: int>
    # <x2: int>
    # <y2: int>
    # <suitable_for_parking: bool (0 or 1)>
    # [repeat for remaining spots ...]
    # end
    #
    # where each element is on a new line.
    def send_spots(self):
        """send registered parking spots to connected bluetooth device"""
        spots_map = self.nav.get_parking_spots()
        self.uart.write(f"s{self.delim}") #  start symbol
        for (id, parking_spot) in spots_map.items():
            self.uart.write(
                f"{id}{self.delim}" + 
                f"{int(parking_spot.x1)}{self.delim}" + 
                f"{int(parking_spot.y1)}{self.delim}" + 
                f"{int(parking_spot.x2)}{self.delim}" + 
                f"{int(parking_spot.y2)}{self.delim}" + 
                f"{int(parking_spot.suitable_for_parking)}{self.delim}"
            )
        self.uart.write(f"end{self.delim}") # end of transmission

    ## Read two ints divided by newlines and return as tuple
    def receive_2_tuple_int(self) -> tuple[int, int]:
        buf = self.uart.read().decode("utf-8", "strict")
        split = buf.split("\r")
        return (int(split[0]), int(split[1]))

    ## Receive the user-selected target parking spot.
    #
    # Get the id of the user selected target spot from the tablet.
    # @return id of the selected spot
    def receive_target_spot(self) -> int:
        buf = self.uart.read().decode("utf-8", "strict")
        split = buf.split("\r")
        return int(split[0])

    ## Receive and save a target position
    def receive_target_position(self):
        self.target_pos = self.receive_2_tuple_int()

    ## Get the target position.
    #
    # @returns (x, y) of target position in mm or None if no target has been received.
    def get_target_pos(self) -> tuple[int, int]:
        return self.target_pos
