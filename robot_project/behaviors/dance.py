#!/usr/bin/python3

# ----------------------------------------------
# Title: dance.py
# Description: Do a fun dance
# Author: Bryce Adam
# Date created: Apr 15, 2025
# Date modified: Apr 15, 2025
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

from actuators.wheels_controller import WheelsController #class

from IS_modules.node_detect_aruco import *


class DanceBehavior:
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
        print("\n\n")
        print("****************************\n\n")
        print("DANCE STARTED\n\n")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
                
        audiobook_confirm = 'mp3_files/audiobook_confirmation.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_confirm,))
        play_thread.start()
        play_thread.join()
        audiobook_select = 'mp3_files/audiobook_selection.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_select,))
        play_thread.start()
        play_thread.join()

       
    def book1(self): # The invisible alligators
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Invisible Alligators")
        audiobook_file = 'mp3_files/audiobooks/invisible_alligatorsApr5.m4a'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(0, self.joints_controller.move_neck, (3, 20))  # A young monkey named Ivy
        self.add_timer(2.5, self.cosmetics_controller.move_eyes, (2, 0.3))  # woke up one morning
        self.add_timer(4.5, self.led_controller.fade_in_led, (2, (255, 165, 0), 200))  # knew there was trouble
        play_thread.join()
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

        # Wait for all threads to finish (including the audio playback)
        self.add_timer(0, self.joints_movement.nod, (2,1))
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)
        play_end_thread.join()
        
        

    def book2(self): #The day the crayons quit
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Invisible Alligators")
        audiobook_file = 'mp3_files/audiobooks/the_day_the_crayons_quitApr5.m4a'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        play_thread.join()
        

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        # Wait for all threads to finish (including the audio playback)
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)
        

    ##Exit###


    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            if self.aruco_detect.exit_behaviour:
                self.stop_flag = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files/i_will_stop.mp3',))
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
                print("[AUDIOBOOK] Exit behaviour detected, stopping relaxation exercise.")
                
                # Turn LEDs orange to indicate a transition period
                self.current_color = (255, 165, 0)  # Orange
                self.led_controller.turn_on_led(self.current_color, 250)

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
