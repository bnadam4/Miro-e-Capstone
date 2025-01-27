#!/usr/bin/python3

# ----------------------------------------------
# Title: audiobooks.py
# Description: Play ch1 of alice in wonderland with body movement
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

from actuators.play_audio import play_audio  # function

class AudiobooksBehavior:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()

    def run(self):
        # Start playing the audiobook in the background
        audiobook_file = 'mp3_files/alices_adventures_ch1_v2.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audiobook_file,))
        play_thread.start()

        # Trigger ear movement at 0s
        
        threading.Timer(10, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 33)).start()).start()
        threading.Timer(10, lambda: threading.Thread(target=self.cosmetics_movement.eyes_squint, args=(4,)).start()).start()
        
        threading.Timer(18, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(4, -25)).start()).start()
        threading.Timer(20, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0.6)).start()).start()
        
        threading.Timer(23, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(2, 0)).start()).start()
        threading.Timer(23, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0)).start()).start()
        
        threading.Timer(25, lambda: threading.Thread(target=self.joints_movement.shake, args=(1,2)).start()).start()

        
        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        
        
        

