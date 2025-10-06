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

    ## time for one execution cycle
    SAMPLETIME = 50000  # in microseconds (us)

    display = Display()
    button_a = ButtonA()
    button_b = ButtonB()
    button_c = ButtonC()
    battery = Battery()
    leds = RGBLEDs()
    heartbeat = HeartbeatLED(leds, 4)
    ## items to show in the onboard display menu
    options = ["idle", "calibrate", "scout", "exit to repl"]
    menu = menu.Menu(options)
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
        "w": lambda: com.println(f"inc v to {con.kinematic_controller.increase_v()}"),
        "s": lambda: com.println(f"dec v to {con.kinematic_controller.decrease_v()}"),
        "d": lambda: com.println(f"inc w to {con.kinematic_controller.increase_w()}"),
        "a": lambda: com.println(f"dec w to {con.kinematic_controller.decrease_w()}"),
    }
    com.bind_map(keymap)

    bat_led_state = False
    com.println("Init complete.")

    def populate_test_spots():
        """save fictitious spots for testing purposes"""
        nav.add_parking_spot(1, navigation.ParkingSpot(100, -100, 300, -300, True))

    ## main execution loop
    #
    # Wrapping this in a function makes it possible to cancel and restart normal operation via the REPL.
    def main_loop():
        while True:
            ts = time.ticks_us()  # get start time
            # handle manual operation via onboard buttons
            if button_a.check() or button_b.check() or button_c.check():
                # set the robot inactive
                gui.request_state(guidance.GuidanceState.IDLE)
                gui.run()
                index = menu.run()

                if options[index] == "idle":
                    gui.request_state(guidance.GuidanceState.IDLE)
                elif options[index] == "calibrate":
                    gui.request_state(guidance.GuidanceState.SETUP)
                elif options[index] == "scout":
                    gui.request_state(guidance.GuidanceState.SCOUT)
                elif options[index] == "exit to repl":
                    display.fill(0)
                    display.text_line("Exiting to repl.", 3)
                    display.show()
                    sys.exit(0)
                else:
                    raise ValueError(f"Option {options[index]} not handled.")

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
    raise
