## @package guidance
#
# Module containing the state machines to orchestrate the other modules, so the robot fulfills its tasks.
from perception import Perception
from control import ModeController, ControlMode
from navigation import Navigation
from communication import Communicator
from utils import Display

from pololu_3pi_2040_robot import motors

SETUP_SPEED = 600


## Enum for states of the main state machine.
class GuidanceState:
    IDLE = "idle"
    SETUP = "setup"
    SCOUT = "scout"
    PARKING = "parking"
    EXTERNAL = "external"


## Enum for the states of the setup submachine.
class GuidanceSetupState:
    RIGHT1 = 1
    LEFT = 2
    RIGHT2 = 3
    DONE = 4


# define more state enums here

## Main state machine for operating the robot.
class GuidanceStateMachine:
    def __init__(self, per: Perception, nav: Navigation, con: ModeController, com: Communicator):
        self.current_state = GuidanceState.IDLE
        self.last_state = GuidanceState.EXTERNAL
        self.requested_state = None
        self.current_setup_state = None
        self.last_setup_state = None

        self.display = Display()

        self.perception = per
        self.navigation = nav
        self.control = con
        self.com = com
        # example of making bindings on runtime, for the general keymap see main.py
        self.com.bind(lambda: self.request_state(GuidanceState.SETUP), "c")
        self.com.bind(lambda: self.navigation.reset(), "l")

    ## Initiate the parking maneuver.
    #
    # Start the parking maneuver by receiving the user-selected target spot and enabling the appropriate state.
    # This should be used as a callback by the communication module bound to "3".
    def start_parking(self):
        """start the parking maneuver by receiving the user-selected spot and enabling the respective Guidance state"""
        id = self.com.receive_target_spot()
        # todo students: request suitable GuidanceState here, process the id ...


    ## Request the next state.
    def request_state(self, state: GuidanceState):
        self.requested_state = state

    ## Helper to print the current state on the display.
    def show_current_state(self):
        self.display.text_line(self.current_state, 0)
        self.display.show()

    ## Execute the main state machine.
    def run(self):
        # update other modules
        self.perception.update()
        self.navigation.update()
        # self.display.show_pose_and_dist(self.perception, self.navigation)
        self.show_current_state()

        if self.current_state == GuidanceState.IDLE:
            if self.current_state != self.last_state:
                # entry action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

            # nominal action

            if self.requested_state:
                # exit action
                pass

        elif self.current_state == GuidanceState.SETUP:
            if self.current_state != self.last_state:
                # entry action
                self.current_setup_state = GuidanceSetupState.RIGHT1
                self.last_setup_state = None
                self._motors = motors.Motors()
                self.navigation.reset()
                self.display.fill(0)
                self.display.text_line("setup", 0)
                self.display.show()

            # nominal action: run submachine
            self.perception.calibrate_linesensor()
            if self.current_setup_state == GuidanceSetupState.RIGHT1:
                if self.current_setup_state != self.last_setup_state:
                    # entry action
                    self._motors.set_speeds(SETUP_SPEED,-SETUP_SPEED)
                    self.display.text_line("r1", 1)
                    self.display.show()
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi <= -90.0:
                    # exit action
                    self.current_setup_state = GuidanceSetupState.LEFT
                    self._motors.set_speeds(0, 0)

            elif self.current_setup_state == GuidanceSetupState.LEFT:
                if self.current_setup_state != self.last_setup_state:
                    self._motors.set_speeds(-SETUP_SPEED, SETUP_SPEED)
                    self.display.text_line("l", 1)
                    self.display.show()
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi >= 90.0:
                    self.current_setup_state = GuidanceSetupState.RIGHT2
                    self._motors.set_speeds(0, 0)

            elif self.current_setup_state == GuidanceSetupState.RIGHT2:
                if self.current_setup_state != self.last_setup_state:
                    self._motors.set_speeds(SETUP_SPEED, -SETUP_SPEED)
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi <= 0.0:
                    self._motors.set_speeds(0, 0)
                    self.current_setup_state = GuidanceSetupState.DONE

            if self.current_setup_state == GuidanceSetupState.DONE:
                self.current_state = GuidanceState.IDLE

            if self.requested_state:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        elif self.current_state == GuidanceState.SCOUT:
            if self.current_state != self.last_state:
                # entry action
                self.control.set_mode(ControlMode.Line)

            # nominal action
            self.control.run()

            if self.requested_state:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        elif self.current_state == GuidanceState.EXTERNAL:
            if self.current_state != self.last_state:
                # entry action
                self.control.set_mode(ControlMode.Kinematic)
            # nominal action
            self.control.run()

            if self.requested_state:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        # todo students: implement more states here



        # run the communicator
        self.com.run()
        # finally save current state and apply possible next state
        self.last_state = self.current_state
        if self.requested_state:
            self.current_state = self.requested_state
            self.requested_state = None
