## @package guidance
# Author: REMOVED, Matrikelnummer: REMOVED
# Module containing the state machines to orchestrate the other modules, so the robot fulfills its tasks.
from perception import Perception
from control import ModeController, ControlMode
from navigation import Navigation
from communication import Communicator
from utils import Display

from pololu_3pi_2040_robot import motors
from navigation import Pose
from math import pi

# speed for setup maneuvers
SETUP_SPEED = 600


## Enum for states of the main state machine.
class GuidanceState:
    """
    Enumeration class for the main guidance state machine states.
    
    Attributes:
        IDLE (str): Robot is idle and waiting for input.
        SETUP (str): Robot is in setup state.
        SCOUT (str): Robot is scouting for parking spots.
        PARKING (str): Robot is executing parking maneuver.
    """
    IDLE = "idle"
    SETUP = "setup"
    SCOUT = "scout"
    PARKING = "parking"
## Enum for the states of the setup submachine.
class GuidanceSetupState:
    """
    Enumeration class for the setup submachine states.
    
    Attributes:
        RIGHT1 (int): First right turn during setup calibration.
        LEFT (int): Left turn during setup calibration.
        RIGHT2 (int): Second right turn during setup calibration.
        DONE (int): Setup calibration is complete.
    """
    RIGHT1 = 1
    LEFT = 2
    RIGHT2 = 3
    DONE = 4

## Enum for the states of the parking submachine.
class GuidanceParkingState:
    """
    Enumeration class for the parking submachine states.
    
    Attributes:
        APPROACH (str): Robot is approaching the parking spot.
        PARK (str): Robot is executing the parking maneuver.
        HOLD (str): Robot is parked and holding position.
        LEAVE (str): Robot is leaving the parking spot.
    """
    APPROACH = "approach"
    PARK = "park"
    HOLD = "hold"
    LEAVE = "leave"

## Main state machine for operating the robot.
class GuidanceStateMachine:
    """
    Main state machine for orchestrating the robot's behavior.
    
    This class manages the overall state transitions and behavior of the robot, including
    setup, scouting for parking spots, and parking maneuvers.
    It coordinates with perception, navigation, control, and communication modules.
    
    Attributes:
        current_state (str): Current state of the main state machine.
        last_state (str): Previous state of the main state machine.
        requested_state (str): Next requested state to transition to.
        current_setup_state (int): Current state within the setup submachine.
        last_setup_state (int): Previous state within the setup submachine.
        current_parking_state (str): Current state within the parking submachine.
        last_parking_state (str): Previous state within the parking submachine.
        parking_spots (list): List of detected parking spots.
        current_parking_spot (ParkingSpot): Target parking spot for current maneuver.
        start_pose (Pose): Start pose for parking maneuver.
        end_pose (Pose): End pose for parking maneuver.
        display (Display): Display interface for showing status information.
        perception (Perception): Perception module instance.
        navigation (Navigation): Navigation module instance.
        control (ModeController): Control module instance.
        com (Communicator): Communication module instance.
    """
    def __init__(self, per: Perception, nav: Navigation, con: ModeController, com: Communicator):
        """
        Initialize the Guidance State Machine.
        
        Args:
            per (Perception): Perception module 
            nav (Navigation): Navigation module 
            con (ModeController): Control module 
            com (Communicator): Communication module 
        """
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

    def start_parking(self):
        """
        Initiate the parking maneuver.
        
        Receives the user-selected target parking spot from the communication module
        and transitions the state machine to the PARKING state.
        
        This method should be used as a callback bound to the communication module.
        """
        id = self.com.receive_target_spot()        
        if self.current_parking_state == None or self.current_parking_state in (GuidanceParkingState.APPROACH, GuidanceParkingState.PARK): #only when not already parking
            self.request_state(GuidanceState.PARKING)
            spots = self.navigation.get_parking_spots()
            self.current_parking_spot = spots[id]

    ## Generate start and end poses for the parking maneuver.
    def generate_parking_path(self):
        """
        Generate start and end poses for the parking path.
        
        Calculates the appropriate start and end poses based on the orientation of the
        current parking spot.
        """
        spot = self.current_parking_spot

        x1 = spot.x1
        y1 = spot.y1
        x2 = spot.x2
        y2 = spot.y2

        if x1 == x2:
            if y1 > y2:
                self.start_pose = Pose(x1+ 100, y1 - 60, 270)
                self.end_pose = Pose(x1 - 80, y1 - 110, 270)
            else:
                self.start_pose = Pose(x1 - 100, y1 + 60, 90)
                self.end_pose = Pose(x1 + 80, y1 + 110, 90)
        else:
            self.start_pose = Pose(x1 + 60, y1 + 100, 0)
            self.end_pose = Pose(x1 + 110, y1 - 80, 0)

    ## Support function for tolerance in position checking.
    def near(self, a, b, tolerance):
        """
        Check if two values are within a specified tolerance.
        
        Args:
            a (float): First value to compare.
            b (float): Second value to compare.
            tolerance (float): Maximum allowed absolute difference.
            
        Returns:
            bool: True if the absolute difference is less than tolerance, False otherwise.
        """
        return abs(a - b) < tolerance

    ## Request the state of next execution.
    def request_state(self, state: GuidanceState):
        """
        Request a state transition for the next execution cycle.
        
        Args:
            state (str): The GuidanceState to transition to.
        """
        self.requested_state = state
            

    ## Helper to print the state of current execution on the display.
    def show_current_state(self):
        """
        Display the current state on the robot's display.
        """
        self.display.text_line(self.current_state, 0)
        self.display.show()

    ## Execute the main state machine.
    def run(self):
        """
        Execute one cycle of the main state machine.
        
        Updates all modules (perception, navigation), processes communications,
        and executes the current state's logic.
        """
        # update other modules
        self.perception.update()
        self.navigation.update()
        self.show_current_state()
        # run the communicator
        self.com.run()
        # leave parking using scout button
        if self.current_state == GuidanceState.SCOUT and self.current_parking_state in (GuidanceParkingState.PARK, GuidanceParkingState.HOLD, GuidanceParkingState.LEAVE):
            self.current_state = GuidanceState.PARKING
            self.last_parking_state = self.current_parking_state
            self.current_parking_state = GuidanceParkingState.LEAVE
        #idle state
        if self.current_state == GuidanceState.IDLE:
            if self.current_state != self.last_state:
                # entry action
                self.control.path_follower.initiate_pause() #for resuming parking maneuver
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

            # nominal action

            if self.requested_state and self.requested_state != GuidanceState.IDLE:
                # exit action
                pass

        elif self.current_state == GuidanceState.SETUP:
            if self.current_state != self.last_state:
                # entry action
                self.navigation.corner_correction_enabled = False   #Corner detection must remain disabled
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
                self.display.clear()

        elif self.current_state == GuidanceState.SCOUT:
            if self.current_state != self.last_state:
                # entry action
                self.navigation.axis_lock_enabled = True # re-enable axis lock after parking
                self.navigation.corner_correction_enabled = True    #Corner detection can be enabled
                self.navigation.set_angle_at_corner = True #Re-enable "set angle at corners" after parking
                self.control.set_mode(ControlMode.Line)
                self.current_parking_state = GuidanceParkingState.APPROACH #Ensures robot approaches parking spot first after scouting
            # nominal action
            self.control.run()

            if self.requested_state and self.requested_state != GuidanceState.SCOUT:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        elif self.current_state == GuidanceState.PARKING:
            if self.current_state != self.last_state:
                # entry action
                self.generate_parking_path() #Calculate path boundary poses
            # nominal action: run submachine
            # anfahren
            if self.current_parking_state == GuidanceParkingState.APPROACH:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    self.navigation.set_angle_at_corner = False # Do not reset angle when approaching
                    self.control.set_mode(ControlMode.Line)

                # nominal action
                self.control.run()

                #exit action 
                if self.near(self.navigation.get_pose().x, self.start_pose.x, 5) and self.near(self.navigation.get_pose().y, self.start_pose.y, 5):   #Check if robot is at start position with tolerance of 5 mm   
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = GuidanceParkingState.PARK
            # parking
            if self.current_parking_state == GuidanceParkingState.PARK:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    # deactivating features that could interfere with path following
                    self.navigation.axis_lock_enabled = False
                    self.navigation.corner_correction_enabled = False
                    self.navigation.set_angle_at_corner = False
                    #Pass start and end poses to PathFollower
                    s = self.start_pose
                    e = self.end_pose
                    s_pose = [s.x, s.y, s.phi * pi / 180.0] #converting degrees to radians
                    e_pose = [e.x, e.y, e.phi * pi / 180.0]

                    self.control.path_follower.set_points(s_pose, e_pose)
                    self.control.set_mode(ControlMode.Path)
                
                if self.control.run(): # nominal action, returns true if path is completed
                    #exit action
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.control.path_follower.reset()
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = GuidanceParkingState.HOLD
            # hold
            if self.current_parking_state == GuidanceParkingState.HOLD:
                if self.current_parking_state != self.last_parking_state:
                    #entry action
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.last_parking_state = GuidanceParkingState.HOLD
            # leaving
            if self.current_parking_state == GuidanceParkingState.LEAVE:
                if self.current_parking_state != self.last_parking_state:
                    # entry action
                    # Pass start and end poses to PathFollower
                    s = self.start_pose
                    e = self.end_pose
                    s_pose = [s.x, s.y, s.phi * pi / 180.0] #converting degrees to radians
                    e_pose = [e.x, e.y, e.phi * pi / 180.0]
                    self.control.path_follower.set_points(e_pose, s_pose)
                    self.last_parking_state = self.current_parking_state
                    
                # nominal action
                self.control.set_mode(ControlMode.Path)
                if self.control.run():   #returns true if path is completed
                    #exit action
                    self.request_state(GuidanceState.SCOUT)
                    self.control.set_mode(ControlMode.Inactive)
                    self.control.run()
                    self.control.path_follower.reset() #Reset controller for next parking maneuver
                    self.last_parking_state = self.current_parking_state
                    self.current_parking_state = None

            if self.requested_state and self.requested_state != GuidanceState.PARKING:
                # exit action
                self.control.set_mode(ControlMode.Inactive)
                self.control.run()

        # finally save state of current execution and apply possible state of next execution
        self.last_state = self.current_state
        if self.requested_state:
            self.current_state = self.requested_state
            self.requested_state = None
