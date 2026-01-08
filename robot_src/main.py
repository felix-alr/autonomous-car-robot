## @package main
#
# Entry point and init script

try:
    import time
    import sys
    from pololu_3pi_2040_robot.battery import Battery
    from pololu_3pi_2040_robot.buttons import ButtonA, ButtonB, ButtonC

    from pololu_3pi_2040_robot.extras import menu
    from pololu_3pi_2040_robot.rgb_leds import RGBLEDs
    from utils import Display, check_battery_empty, HeartbeatLED

    import guidance
    import perception
    import navigation
    import control
    import communication

    # DEBUG
    import control_test

    ## time for one execution cycle
    SAMPLETIME = 50000  # in microseconds (us)

    _gui_run_fp = None # eventually hold the original gui.run func

    display = Display()
    button_a = ButtonA()
    button_b = ButtonB()
    button_c = ButtonC()
    battery = Battery()
    leds = RGBLEDs()
    heartbeat = HeartbeatLED(leds, 4) # front center led

    ## items to show in the onboard display menu
    class MenuItems:
        IDLE = "idle"
        SETUP = "setup"
        SCOUT = "scout"
        POSCTRL = "position control"
        EXIT = "exit to repl"

        def listify(self):
            return [self.IDLE, self.SETUP, self.SCOUT, self.POSCTRL, self.EXIT]

    menu_items = MenuItems().listify()
    menu = menu.Menu(menu_items)
    menu.display = display
    menu.next_button = button_c
    menu.previous_button = button_a
    menu.select_button = button_b

    # general config for interfaces, handled by the respective modules
    # uart0 = machine.UART(0, baudrate=115200, tx=machine.Pin(28), rx=machine.Pin(29))
    # i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=400_000)

    per = perception.Perception()
    nav = navigation.Navigation(per)
    con = control.ModeController(per, nav)
    com = communication.Communicator(nav)
    gui = guidance.GuidanceStateMachine(per, nav, con, com)
    voltage = battery.get_level_millivolts()
    if voltage < 4400:
        com.println(f"Battery low! {voltage} mV")
    else:
        com.println(f"Battery OK {voltage} mV")

    ## add default keybindings
    # Mind that these are exposing the communication interface and should only be changed if necessary
    # and in accordance with the HMI project.
    keymap = {
        "1": com.send_pose,
        "2": com.send_spots,
        "3": gui.start_parking,
        "4": com.receive_target_position,
        "v": lambda: com.println(f"{battery.get_level_millivolts()}"),
        "y": lambda: gui.request_state(guidance.GuidanceState.SCOUT),
        "z": lambda: gui.request_state(guidance.GuidanceState.IDLE),
        "q": lambda: gui.request_state(guidance.GuidanceState.EXTERNAL),
        "r": lambda: gui.request_state(guidance.GuidanceState.SETUP),
        "w": lambda: com.println(f"inc v to {con.kinematic_controller.increase_v()}"),
        "s": lambda: com.println(f"dec v to {con.kinematic_controller.decrease_v()}"),
        "d": lambda: com.println(f"inc w to {con.kinematic_controller.increase_w()}"),
        "a": lambda: com.println(f"dec w to {con.kinematic_controller.decrease_w()}"),

        # DEBUG
        "p": lambda: control_test.alter_parameters(com.uart, con, "kp"),
        "k": lambda: control_test.alter_parameters(com.uart, con, "kd"),
    }
    com.bind_map(keymap)

    bat_led_state = False
    com.println("Init complete.")

    def populate_test_spots():
        """save fictitious spots for testing purposes"""
        nav.add_parking_spot(1, navigation.ParkingSpot(100, -100, 300, -300, True))

    ## Run function to hijack the state machine with position control.
    #
    # This run function will be used instead of the one defined in the guidance module
    # to activate position control. (to keep the guidance FSM clean from this extra task)
    def run_position_control(self: guidance.GuidanceStateMachine):
        self.current_state = "Position_Control"

        # update other modules
        self.perception.update()
        self.navigation.update()
        self.com.run()

        if self.current_state != self.last_state:
            # entry action
            self.display.clear()
            self.display.text_line(self.current_state, 1)
            self.control.set_mode(control.ControlMode.Position)

        # nominal action
        target = com.get_target_pos()
        if target:
            self.control.position_controller.set_position(*target)
        self.display.text_line(f"T: {con.position_controller.target}", 3)
        pos = nav.get_position()
        self.display.text_line(f"C: ({int(pos[0])}, {int(pos[1])})", 5)
        self.control.run()

        if self.requested_state and self.requested_state != "Position_Control":
            # exit action
            self.control.set_mode(control.ControlMode.Inactive)

        # finally save current state and apply possible next state
        self.last_state = self.current_state
        if self.requested_state:
            self.current_state = self.requested_state
            self.requested_state = None
            self.last_state = None
            # restore original run when exiting from position control
            guidance.GuidanceStateMachine.run = _gui_run_fp

    ## main execution loop
    #
    # Wrapping this in a function makes it possible to cancel and restart normal operation via the REPL.
    def main_loop():
        global _gui_run_fp
        while True:
            ts = time.ticks_us()  # get start time
            # handle manual operation via onboard buttons
            if button_a.check() or button_b.check() or button_c.check():
                # set the robot inactive
                gui.request_state(guidance.GuidanceState.IDLE)
                gui.run()
                index = menu.run()

                if menu_items[index] == MenuItems.IDLE:
                    gui.request_state(guidance.GuidanceState.IDLE)
                elif menu_items[index] == MenuItems.SETUP:
                    gui.request_state(guidance.GuidanceState.SETUP)
                elif menu_items[index] == MenuItems.SCOUT:
                    gui.request_state(guidance.GuidanceState.SCOUT)
                elif menu_items[index] == MenuItems.EXIT:
                    display.clear()
                    display.text_line("Exiting to repl.", 3)
                    display.show()
                    sys.exit(0)
                elif menu_items[index] == MenuItems.POSCTRL:
                    # save original func and inject state machine for position control
                    _gui_run_fp = guidance.GuidanceStateMachine.run
                    guidance.GuidanceStateMachine.run = run_position_control
                else:
                    raise ValueError(f"Option {menu_items[index]} not handled.")

                # clear the display
                display.clear()

            # execute the main state machine
            gui.run()
            heartbeat.update()

            # check battery and set status led
            global bat_led_state
            if check_battery_empty(battery):
                leds.set(1, (255, 0, 0))  # set bottom center led red
                leds.show()
                bat_led_state = True
            elif bat_led_state:
                leds.set(1, (0, 0, 0))  # reset bottom center led
                leds.show()
                bat_led_state = False

            # pad execution time to SAMPLETIME
            dt = time.ticks_diff(time.ticks_us(), ts)
            if dt < SAMPLETIME:
                time.sleep_us(SAMPLETIME - dt)

    main_loop()

except Exception as e:
    # save exception for later examination in the repl and print to display
    exc = e
    Display.show_exception(e)
    with open("error.txt", "wt") as f:
        from sys import _exc_traceback
        tb = _exc_traceback(e)
        f.write(repr(e))
        f.write(repr(tb))
    raise
