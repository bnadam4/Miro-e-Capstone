#!/usr/bin/python3

# ----------------------------------------------
# Title: audiobooks.py
# Description: Play audiobooks with body movement
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Feb 24, 2024
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
        print("\n\n")
        print("****************************\n\n")
        print("AUDIOBOOK BEHAVIOUR STARTED\n\n")
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

        while True:
            self.aruco_detect.tick_camera()

            if self.aruco_detect.emperor:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax_prompt.")
                self.aruco_detect.emperor = False
                self.book2()
                break
            if self.aruco_detect.rupelstiltskin:
                print("[AUDIOBOOK] Head touch detected, proceeding to relax_prompt.")
                self.aruco_detect.rupelstiltskin = False
                self.book1()
                break
            if self.aruco_detect.frog:
                print("[AUDIOBOOK] Head touch detected, proceeding to relax_prompt.")
                self.aruco_detect.frog = False
                self.book3()
                break
            time.sleep(0.1)
        

    def run_command(self):
        print("\n\n")
        print("****************************\n\n")
        print("AUDIOBOOK BEHAVIOUR STARTED\n\n")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        
        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= Rumpelstiltskin \n2= The Emperor's New Clothes \n3= The Frog Prince \n"))
        
        if user_input == 0:
            print("[AUDIOBOOK] Exiting program...")
        elif user_input == 1:
            print("1) Rumpelstiltskin")
            self.book1()
        elif user_input == 2:
            print("2) The Emperor's New Clothes")
            self.book2()
        elif user_input == 3:
            print("3) The Frog Prince")
            self.book3()
        else:
            print("[AUDIOBOOK] Invalid choice or unrecognized number.")

    def book1(self): # The invisible alligators
        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Invisible Alligators")
        audiobook_file = 'mp3_files/audiobooks/invisible_alligators.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(0, self.joints_controller.move_neck, (3, 20))  # A young monkey named Ivy
        self.add_timer(2.5, self.cosmetics_controller.move_eyes, (2, 0.3))  # woke up one morning
        self.add_timer(4.5, self.led_controller.fade_in_led, (2, (255, 165, 0), 200))  # knew there was trouble
        self.add_timer(6.5, self.joints_movement.nod, (1, 2))  # hopped out of bed
        self.add_timer(9.5, self.cosmetics_controller.move_ears, (2, 0.4))  # found the bridge broken
        self.add_timer(11, self.led_controller.toggle_led, (3, (255, 0, 0), 150))  # It took her forever to fix it
        self.add_timer(14.5, self.joints_controller.move_yaw, (2, -20))  # stairs were covered in toys
        self.add_timer(17, self.cosmetics_movement.eyes_squint, (1,))  # tripped and had to pick them up
        self.add_timer(19, self.led_controller.fade_out_led, (2,))  # Then her llamas were loose
        self.add_timer(22, self.joints_controller.move_pitch, (2, -10))  # took six tries to herd them
        self.add_timer(26, self.cosmetics_controller.move_eyes, (2, 0.6))  # stop eating her homework
        self.add_timer(28, self.led_controller.turn_on_led, ((0, 255, 0), 200))  # so late that she missed math class
        self.add_timer(33, self.joints_movement.shake, (2, 2))  # homework covered in bites
        self.add_timer(35.5, self.cosmetics_controller.move_ears, (2, 0))  # Tonight, she'd put a stop to this
        self.add_timer(38.3, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # That night, Ivy stayed awake
        self.add_timer(41, self.joints_controller.move_neck, (3, 30))  # heard rustling under her bed
        self.add_timer(44, self.cosmetics_movement.eye_wink, ("right", 1, 1))  # flipped it over and found
        self.add_timer(46, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # invisible alligators!
        self.add_timer(48, self.joints_controller.move_yaw, (3, 20))  # One of the alligators said
        self.add_timer(50.2, self.cosmetics_controller.move_eyes, (2, 0.2))  # "We cause trouble like this"
        self.add_timer(53.5, self.led_controller.turn_off_led)  # "We're just trying to help"
        self.add_timer(55.5, self.joints_movement.nod, (2, 1))  # "Let me show you"
        self.add_timer(58, self.cosmetics_controller.move_ears, (2, 0.6))  # Curious, Ivy followed him
        self.add_timer(59.5, self.led_controller.fade_in_led, (3, (255, 0, 255), 200))  # deep into their catacombs
        self.add_timer(63.5, self.joints_controller.move_pitch, (2, -15))  # hiding the sheep remotes
        self.add_timer(65.8, self.cosmetics_movement.eyes_squint, (2,))  # swapping cake mix for broccoli
        self.add_timer(68.6, self.led_controller.toggle_led, (4, (0, 255, 255), 200))  # singing hippos to sleep
        self.add_timer(71.4, self.joints_controller.move_neck, (3, 20))  # "But why?" Ivy asked
        self.add_timer(73.7, self.cosmetics_controller.move_eyes, (2, 0.5))  # "Why can't everything just go right?"
        self.add_timer(75.6, self.led_controller.fade_out_led, (3,))  # The alligator sighed
        self.add_timer(79.6, self.joints_movement.shake, (2, 1))  # He took her to their headquarters
        self.add_timer(83.2, self.cosmetics_controller.move_ears, (2, 0))  # her book
        self.add_timer(85.5, self.led_controller.turn_on_led, ((255, 165, 0), 200))  # all the troubles she'd faced
        self.add_timer(87.5, self.joints_controller.move_yaw, (3, -20))  # all the things she'd learned
        self.add_timer(90, self.cosmetics_movement.close_eyes, (1,))  # "I don't get it"
        self.add_timer(92, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # She left, disappointed
        self.add_timer(97, self.joints_controller.move_pitch, (2, -10))  # found rocks on the stairs
        self.add_timer(99, self.cosmetics_controller.move_eyes, (2, 0.3))  # a broken bridge
        self.add_timer(105, self.led_controller.toggle_led, (5, (0, 255, 0), 200))  # very dangerous blue goats
        self.add_timer(104.5, self.joints_movement.nod, (1, 3))  # she cleared the rocks
        self.add_timer(106, self.cosmetics_controller.move_ears, (2, 0.4))  # fixed the bridge
        self.add_timer(107.5, self.led_controller.turn_off_led)  # herded the goats
        self.add_timer(111, self.joints_controller.move_neck, (3, 25))  # climbed into bed
        self.add_timer(114, self.cosmetics_movement.eye_wink, ("right", 1, 2))  # dreaming of Math
        self.add_timer(115.8, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # How did she know?
        self.add_timer(118, self.joints_controller.move_yaw, (3, 20))  # Maybe the invisible alligators
        self.add_timer(120.5, self.cosmetics_controller.move_eyes, (2, 0.2))  # did help after all
        self.add_timer(124, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # they might visit you tonight
        self.add_timer(126.3, self.joints_movement.shake, (2, 1))  # The End
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
        
        


        
        

    def book2(self): #The day the crayons quit
        pass

    def book3(self): # TBD
        pass

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

