# This test file serves the purpose of testing specific features.

from robot_src import guidance
import time
from math import pi
from machine import Pin
from perception import Perception
from control import ModeController, ControlMode
from navigation import Navigation, Pose
from communication import Communicator

# Hardware-Setup für den Test-Button (Button A am 3pi+)
self.com.bind(lambda: self.run(), "c")

def run(per, nav, con):
    # 1. Definieren der festen Test-Koordinaten (in mm und Grad)
    # Beispiel: Start bei (100, 100) mit 0°, Ziel bei (400, 100) mit 0°
    start = {"x": 100, "y": 100, "phi": 0}
    end = {"x": 400, "y": 100, "phi": 0}
    
    # Umrechnung in Radiant, wie in deinem Code
    s_pose = [start["x"], start["y"], start["phi"] * pi / 180.0]
    e_pose = [end["x"], end["y"], end["phi"] * pi / 180.0]
    
    print(f"Test gestartet: Von {start} nach {end}")
    
    # 2. Pfadregler konfigurieren
    con.path_follower.set_points(s_pose, e_pose)
    con.set_mode(ControlMode.Path)
    
    # 3. Ausführungsschleife (ähnlich deiner GuidanceStateMachine.run)
    active = True
    while active:
        per.update()
        nav.update()
        
        # con.run() gibt True zurück, wenn das Ziel erreicht ist
        finished = con.run()
        
        if finished:
            print("Ziel erreicht!")
            con.set_mode(ControlMode.Inactive)
            con.run()
            active = False
        
        time.sleep(0.01) # Kurze Pause zur Stabilisierung

# --- Main Setup ---
per = Perception()
nav = Navigation()
con = ModeController()
com = Communicator() # Falls für Initialisierung nötig

print("Warte auf Tastendruck (Button A)...")

while True:
    if button_a.value() == 0: # Button gedrückt
        run_path_test(per, nav, con)
        print("Bereit für nächsten Test.")
        time.sleep(1) # Entprellen