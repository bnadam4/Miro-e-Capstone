#!/usr/bin/python3

# ----------------------------------------------
# Title: audiobooks.py
# Description: Play ch1 of alice in wonderland with body movement
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Feb 5, 2024
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
        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= Rumpelstiltskin \n2= The Emperor's New Clothes \n"))
        
        if user_input == 0:
            print("Exiting program...")
        elif user_input == 1:
            print("1) Rumpelstiltskin")
            self.book1()
        elif user_input == 2:
            print("2) The Emperor's New Clothes")
            self.book2()
        else:
            print("Invalid choice or unrecognized number.")


    def book1(self): # Rumpelstiltskin
        # Start playing the audiobook in the background
        audiobook_file = 'mp3_files/rumpelstiltskin.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audiobook_file,))
        play_thread.start()
	
        threading.Timer(4, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 34)).start()).start()
        threading.Timer(4, lambda: threading.Thread(target=self.cosmetics_controller.move_eyes, args=(4, 0.2)).start()).start()
        
        threading.Timer(8, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0.6)).start()).start()
        
        threading.Timer(13, lambda: threading.Thread(target=self.led_controller.fade_in_led, args=(5, (255,255,0), 250)).start()).start()
        
        threading.Timer(20, lambda: threading.Thread(target=self.joints_movement.nod, args=(4,3)).start()).start()
        
        threading.Timer(24, lambda: threading.Thread(target=self.led_controller.turn_off_led).start()).start()
        threading.Timer(24, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0)).start()).start()
        
        threading.Timer(26, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(3, -25)).start()).start()
        threading.Timer(26, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(3, 20)).start()).start()
        
        threading.Timer(30, lambda: threading.Thread(target=self.joints_controller.move_pitch, args=(2, -15)).start()).start()
        
        threading.Timer(32, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(4, 25)).start()).start()
        
        threading.Timer(33, lambda: threading.Thread(target=self.led_controller.turn_on_led, args=((255, 165, 0), 250)).start()).start()
        
        threading.Timer(38, lambda: threading.Thread(target=self.joints_movement.shake, args=(3,3)).start()).start()
        
        threading.Timer(42, lambda: threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,)).start()).start()
        
        threading.Timer(43, lambda: threading.Thread(target=self.led_controller.turn_off_led).start()).start()
        
        threading.Timer(48, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 34)).start()).start()
        threading.Timer(48, lambda: threading.Thread(target=self.cosmetics_movement.open_eyes, args=(3,)) .start()).start()
        
        threading.Timer(52, lambda: threading.Thread(target=self.joints_controller.move_pitch, args=(2, 7)).start()).start()

        threading.Timer(54, lambda: threading.Thread(target=self.joints_movement.shake, args=(4,3)).start()).start()
        
        threading.Timer(55, lambda: threading.Thread(target=self.led_controller.fade_in_led, args=(2, (0, 0, 255), 250)).start()).start()
        threading.Timer(57, lambda: threading.Thread(target=self.led_controller.fade_out_led, args=(2,)).start()).start()
        threading.Timer(59, lambda: threading.Thread(target=self.led_controller.fade_in_led, args=(2, (0, 0, 255), 250)).start()).start()
        threading.Timer(61, lambda: threading.Thread(target=self.led_controller.fade_out_led, args=(2,)).start()).start()
        
        threading.Timer(63, lambda: threading.Thread(target=self.cosmetics_movement.blink_eyes, args=(2,3)).start()).start()
        
        threading.Timer(65, lambda: threading.Thread(target=self.joints_controller.move_pitch, args=(4, -15)).start()).start()
        
        threading.Timer(69, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 34)).start()).start()
        
        threading.Timer(75, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0.6)).start()).start()
        
        threading.Timer(80, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 28)).start()).start()
        
        threading.Timer(84, lambda: threading.Thread(target=self.led_controller.fade_in_led, args=(2, (255,65,0), 250)).start()).start()
        
        threading.Timer(88, lambda: threading.Thread(target=self.joints_movement.nod, args=(1.5,3)).start()).start()
        
        threading.Timer(93, lambda: threading.Thread(target=self.joints_movement.nod, args=(1.5,3)).start()).start()
        
        
        

        
        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        
        

    def book2(self): #The Emperor's New Clothes
        # Start playing the audiobook in the background
        audiobook_file = 'mp3_files/emperorsNewClothes.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audiobook_file,))
        play_thread.start()

        # Trigger ear movement at 0s
        
        threading.Timer(10, lambda: threading.Thread(target=self.joints_controller.move_neck, args=(4, 34)).start()).start()
        threading.Timer(10, lambda: threading.Thread(target=self.cosmetics_movement.eyes_squint, args=(4,)).start()).start()
        
        threading.Timer(18, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(4, -25)).start()).start()
        threading.Timer(20, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0.6)).start()).start()
        
        threading.Timer(23, lambda: threading.Thread(target=self.joints_controller.move_yaw, args=(2, 0)).start()).start()
        threading.Timer(23, lambda: threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0)).start()).start()
        
        threading.Timer(25, lambda: threading.Thread(target=self.joints_movement.shake, args=(1,2)).start()).start()

        
        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
