## @package guidance
#
# Module containing the state machines to orchestrate the other modules, so the robot fulfills its tasks.
from perception import Perception
from control import ModeController, ControlMode
from navigation import Navigation
from communication import Communicator
from utils import Display

from pololu_3pi_2040_robot import motors
from navigation import Pose
from math import pi

# For debugging -----------------
from machine import Pin, UART
uart = UART(0, baudrate=115200, tx=Pin(28), rx=Pin(29))



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

## Enum for the states of the parking submachine.
class GuidanceParkingState:
    APPROACH = "approach"
    ALIGN = "align"
    PARK = "park"
    HOLD = "hold"
    LEAVE = "leave"

## Main state machine for operating the robot.
class GuidanceStateMachine:
    def __init__(self, per: Perception, nav: Navigation, con: ModeController, com: Communicator):
        self.current_state = GuidanceState.IDLE
        self.last_state = None
        self.requested_state = None
        self.current_setup_state = None
        self.last_setup_state = None
        self.current_parking_state = None
        self.last_parking_state = None
        
        self.parking_spots = None
        self.current_parking_spot = None
        self.start_pose = None
        self.end_pose = None

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
        self.request_state(GuidanceState.PARKING)
        spots = self.navigation.get_parking_spots()
        self.current_parking_spot = spots[id]

    ## Generate start and end poses for the parking maneuver.
    def generate_parking_path(self):
        spot = self.current_parking_spot

        x1 = spot.x1
        y1 = spot.y1
        x2 = spot.x2
        y2 = spot.y2


        if x1 == x2:
            if y1 > y2:
                self.start_pose = Pose(x1+ 100, y1, 270)
                self.end_pose = Pose(x1 - 75, (y1 + y2)/2, 270)
            else:
                self.start_pose = Pose(x1 - 100, y1, 90)
                self.end_pose = Pose(x1 + 75, (y1 + y2) / 2, 90)
        else:
            self.start_pose = Pose(x1, y1 + 100, 0)
            self.end_pose = Pose((x1 + x2) / 2, y1 - 75, 0)

    ## Support function for tolerance in position checking.
    def near(self, a, b, tolerance):
        return abs(a - b) < tolerance

    ## Request the state of next execution.
    def request_state(self, state: GuidanceState):
        self.requested_state = state
        if self.current_state == GuidanceState.PARKING and self.requested_state == GuidanceState.SCOUT: # Um Ausparken über Scout Befehl zu realisieren
            self.requested_state = GuidanceState.PARKING
            self.current_parking_state = GuidanceParkingState.LEAVE
            

    ## Helper to print the state of current execution on the display.
    def show_current_state(self):
        self.display.text_line(self.current_state, 0)
        self.display.show()

    ## Execute the main state machine.
    def run(self):
        # update other modules
        self.perception.update()
        self.navigation.update()
        self.show_current_state()
        # run the communicator
        self.com.run()

        if self.current_state == GuidanceState.IDLE:
            if self.current_state != self.last_state:
                # entry action
                self.last_parking_state = None # Damit die Eingangsaktion nach Stoppen und Rückkehr in den Parkmodus erneut ausgeführt wird
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

            # nominal action

            if self.requested_state and self.requested_state != GuidanceState.IDLE:
                # exit action
                pass

        elif self.current_state == GuidanceState.SETUP:
            if self.current_state != self.last_state:
                self.navigation.corner_correction_enabled = False   #Eckenerkennung muss deaktiviert bleiben
                # entry action
                self.current_setup_state = GuidanceSetupState.RIGHT1
                self.last_setup_state = None
                self._motors = motors.Motors()
                self.navigation.reset()
                self.display.text_line("setup", 0)

            # nominal action: run submachine
            self.perception.calibrate_linesensor()
            if self.current_setup_state == GuidanceSetupState.RIGHT1:
                if self.current_setup_state != self.last_setup_state:
                    # entry action
                    self._motors.set_speeds(SETUP_SPEED,-SETUP_SPEED)
                    self.display.text_line("r1", 1)
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi <= -90.0:
                    # exit action
                    self.current_setup_state = GuidanceSetupState.LEFT
                    self._motors.set_speeds(0, 0)

            elif self.current_setup_state == GuidanceSetupState.LEFT:
                if self.current_setup_state != self.last_setup_state:
                    self._motors.set_speeds(-SETUP_SPEED, SETUP_SPEED)
                    self.display.text_line("l1", 1)
                    self.display.show()
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi >= 90.0:
                    self.current_setup_state = GuidanceSetupState.RIGHT2
                    self._motors.set_speeds(0, 0)

            elif self.current_setup_state == GuidanceSetupState.RIGHT2:
                if self.current_setup_state != self.last_setup_state:
                    self.display.text_line("r2", 1)
                    self._motors.set_speeds(SETUP_SPEED, -SETUP_SPEED)
                    self.last_setup_state = self.current_setup_state

                if self.navigation.get_pose().phi <= 0.0:
                    self._motors.set_speeds(0, 0)
                    self.current_setup_state = GuidanceSetupState.DONE

            if self.current_setup_state == GuidanceSetupState.DONE:
                self.request_state(GuidanceState.IDLE)

            if self.requested_state and self.requested_state != GuidanceState.SETUP:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        elif self.current_state == GuidanceState.SCOUT:
            if self.current_state != self.last_state:
                self.navigation.axis_lock_enabled = True # axis lock nach Parken wieder einschalten
                self.navigation.corner_correction_enabled = True    #Eckenerkennung darf aktiviert werden
                self.navigation.set_angle_at_corner = True #"Winkel an Ecken setzen" nach Parken wieder aktivieren
                # entry action
                self.control.set_mode(ControlMode.Line)
                self.current_parking_state = GuidanceParkingState.APPROACH #Damit der Roboter nach der Parklückensuche beim Übergang in den Zustand parking immer erstmal die Parklücke anfährt
            # nominal action
            self.control.run()

            if self.requested_state and self.requested_state != GuidanceState.SCOUT:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        elif self.current_state == GuidanceState.PARKING:
            if self.current_state != self.last_state:
                # entry action
                self.generate_parking_path() #Pfadrandposen berechnen
            # nominal action: run submachine
            # anfahren
            if self.current_parking_state == GuidanceParkingState.APPROACH:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    self.navigation.set_angle_at_corner = False # beim Anfahren den Winkel nicht zurücksetzen
                    self.control.set_mode(ControlMode.Line)
                # nominal action
                self.control.run()
                self.display.text_line("anfahren", 1)

                #exit action (nur beim Übergang in anderen Unterzustand)
                if self.near(self.navigation.get_pose().x, self.start_pose.x, 10) and self.near(self.navigation.get_pose().y, self.start_pose.y, 10):   #Prüfen, ob sich Roboter auf Startposition befindet mit Toleranz von 10 mm   
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = GuidanceParkingState.ALIGN
            # ausrichten
            if self.current_parking_state == GuidanceParkingState.ALIGN:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    self.navigation.axis_lock_enabled = False
                    self.control.set_mode(ControlMode.Kinematic)
                    self.control.kinematic_controller.set_vw(0, 0.3) # Wert verändern?
                # nominal action
                self.display.text_line("ausrichten", 1)
                self.control.run()

                #exit action
                if self.near(self.navigation.get_pose().phi, self.start_pose.phi, 0.5):   #Prüfen, ob sich Roboter in Ausrichtungswinkel befindet mit Toleranz von 0,5 Grad
                    self.start_pose = self.navigation.get_pose() #Aktualisierung der Startpose mit aktueller Pose nach Ausrichten augrund der Toleranz der Startposition, für erhöhte Genauigkeit bei der Pfadfolge
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = GuidanceParkingState.PARK
            # einparken
            if self.current_parking_state == GuidanceParkingState.PARK:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    #Übergabe der Start- und Endpose an den PathFollower
                    s = self.start_pose
                    e = self.end_pose
                    s_pose = [s.x, s.y, s.phi * pi / 180.0]
                    e_pose = [e.x, e.y, e.phi * pi / 180.0]
                    self.display.text_line("einparken", 1)
                    self.display.text_line(f"x={s.x} y={s.y}", 2)
                    self.display.text_line(f"x={e.x} y={e.y}", 3)
                    # self.display.long_text(f"s_pose x={s.x} y={s.y} phi={s.phi} e_pose x={e.x} y={e.y} phi={e.phi}")

                    self.control.path_follower.set_points(s_pose, e_pose)
                    self.control.set_mode(ControlMode.Path)
                # nominal action
                self.display.text_line("einparken", 1)

                #exit action
                if self.control.run():
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = GuidanceParkingState.HOLD
            # halten
            if self.current_parking_state == GuidanceParkingState.HOLD:
                if self.current_parking_state != self.last_parking_state:
                    #entry action
                    self.display.text_line("halten", 1)
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = GuidanceParkingState.HOLD
            # verlassen
            if self.current_parking_state == GuidanceParkingState.LEAVE:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    #Übergabe der Start- und Endpose an den PathFollower
                    self.display.text_line("ausparken", 1)
                    s = self.start_pose
                    e = self.end_pose
                    s_pose = [s.x, s.y, s.phi * pi / 180.0]
                    e_pose = [e.x, e.y, e.phi * pi / 180.0]
                    self.control.path_follower.set_points(e_pose, s_pose)
                    self.control.set_mode(ControlMode.Path)
                # nominal action

                #exit action
                if self.control.run():   #Prüfen, ob sich Roboter auf Startposition befindet
                    self.request_state(GuidanceState.SCOUT)
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = self.current_parking_state

            if self.requested_state and self.requested_state != GuidanceState.PARKING:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        # finally save state of current execution and apply possible state of next execution
        self.last_state = self.current_state
        if self.requested_state:
            self.current_state = self.requested_state
            self.requested_state = None
