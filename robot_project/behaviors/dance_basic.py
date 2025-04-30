#!/usr/bin/python3

# ----------------------------------------------
# Title: dance.py
# Description: Example using threads and sleep (getting started)
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------



import threading
import time
from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class

from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class


class DanceBasicBehavior:
    def __init__(self):
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()

    def run(self):
        # Run ear movement from 0-2s
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        # Turn on LED from 1-8s
        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        # Move eyes from 6-8s
        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.eye_wink, args=(2,1, "right"))
        eyes_thread.start()

        # Wait for the first set of actions to finish
        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()  # Ensure eyes movement completes before moving on
        time.sleep(5)

        # No actions from 13-15s, just wait
        print("Waiting from 13-15 seconds...")
        time.sleep(2)

        # Run ear movement again from 15-19s
        ear_thread = threading.Thread(target=self.cosmetics_controller.move_one_ear, args=(4, 0.7, "left"))
        ear_thread.start()

        # Turn on LED from 17-19s
        time.sleep(2)
        led_thread_2 = threading.Thread(target=self.led_controller.toggle_led, args=(2, (160, 32, 240), 250))
        
        led_thread_2.start()

        # Wait for final actions to finish
        ear_thread.join()
        led_thread_2.join()    

