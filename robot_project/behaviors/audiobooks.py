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

from actuators.play_audio import AudioPlayer  # class

from IS_modules.node_detect_aruco import *


class AudiobooksBehavior:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()
        self.aruco_detect = NodeDetectAruco()
        self.audio_player = AudioPlayer()

        self.stop_flag = False

        self.parent_thread = None  # Store parent thread reference
        self.timers = []  # List to track timers

    def run(self):
        print("[AUDIOBOOK] Running audiobooks behavior")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        
        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= Rumpelstiltskin \n2= The Emperor's New Clothes \n"))
        
        if user_input == 0:
            print("[AUDIOBOOK] Exiting program...")
        elif user_input == 1:
            print("1) Rumpelstiltskin")
            self.book1()
        elif user_input == 2:
            print("2) The Emperor's New Clothes")
            self.book2()
        else:
            print("[AUDIOBOOK] Invalid choice or unrecognized number.")
        

    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            if self.aruco_detect.exit_behaviour:
                self.stop_flag = True
                self.audio_player.stop()
                print("[AUDIOBOOK] Exit behaviour detected, stopping audiobook.")
            time.sleep(0.1)

    def safe_execute(self, func, args):
        """Execute function only if stop flag is not set and parent thread is still alive."""
        if not self.stop_flag and (self.parent_thread and self.parent_thread.is_alive()):
            func(*args)
        else:
            print(f"[AUDIOBOOK] Skipping {func.__name__} because parent thread has exited.")

    # Function to safely start actions
    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: self.safe_execute(func, args))
        self.timers.append(timer)
        timer.start()

    def book1(self): # Rumpelstiltskin
        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing Rumpelstiltskin")
        audiobook_file = 'mp3_files/rumpelstiltskin.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        
        # Schedule all movements
        self.add_timer(4, self.joints_controller.move_neck, (4, 34))
        self.add_timer(4, self.cosmetics_controller.move_eyes, (4, 0.2))
        
        self.add_timer(8, self.cosmetics_controller.move_ears, (2, 0.6))
        
        self.add_timer(13, self.led_controller.fade_in_led, (5, (255,255,0), 250))
        
        self.add_timer(20, self.joints_movement.nod, (4,3))
        
        self.add_timer(24, self.led_controller.turn_off_led)
        self.add_timer(24, self.cosmetics_controller.move_ears, (2, 0))
        
        self.add_timer(26, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(26, self.joints_controller.move_neck, (3, 20))
        
        self.add_timer(30, self.joints_controller.move_pitch, (2, -15))
        
        self.add_timer(32, self.joints_controller.move_yaw, (4, 25))
        
        self.add_timer(33, self.led_controller.turn_on_led, ((255, 165, 0), 250))
        
        self.add_timer(38, self.joints_movement.shake, (3,3))
        
        self.add_timer(42, self.cosmetics_movement.close_eyes, (1,))
        
        self.add_timer(43, self.led_controller.turn_off_led)
        
        self.add_timer(48, self.joints_controller.move_neck, (4, 34))
        self.add_timer(48, self.cosmetics_movement.open_eyes, (3,))
        
        self.add_timer(52, self.joints_controller.move_pitch, (2, 7))

        self.add_timer(54, self.joints_movement.shake, (4,3))
        
        self.add_timer(55, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(57, self.led_controller.fade_out_led, (2,))
        self.add_timer(59, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(61, self.led_controller.fade_out_led, (2,))
        
        self.add_timer(63, self.cosmetics_movement.blink_eyes, (2,3))
        
        self.add_timer(65, self.joints_controller.move_pitch, (4, -15))
        
        self.add_timer(69, self.joints_controller.move_neck, (4, 34))
        
        self.add_timer(75, self.cosmetics_controller.move_ears, (2, 0.6))
        
        self.add_timer(80, self.joints_controller.move_neck, (4, 28))
        
        self.add_timer(84, self.led_controller.fade_in_led, (2, (255,65,0), 250))
        
        self.add_timer(88, self.joints_movement.nod, (1.5,3))
        
        self.add_timer(93, self.joints_movement.nod, (1.5,3))

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.5)

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files/i_will_stop.mp3',))
        exit_behaviour_thread.start()
        exit_behaviour_thread.join()
        
        

    def book2(self): #The Emperor's New Clothes
        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Emperor's New Clothes")
        audiobook_file = 'mp3_files/emperorsNewClothes.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(10, self.joints_controller.move_neck, (4, 34))
        self.add_timer(10, self.cosmetics_movement.eyes_squint, (4,))
        
        self.add_timer(18, self.joints_controller.move_yaw, (4, -25))
        self.add_timer(20, self.cosmetics_controller.move_ears, (2, 0.6))
        
        self.add_timer(23, self.joints_controller.move_yaw, (2, 0))
        self.add_timer(23, self.cosmetics_controller.move_ears, (2, 0))
        
        self.add_timer(25, self.joints_movement.shake, (1, 2))

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book2...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.5)

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files/i_will_stop.mp3',))
        exit_behaviour_thread.start()
        exit_behaviour_thread.join()
