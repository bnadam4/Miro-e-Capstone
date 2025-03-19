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
        
        # Taking input from the user
        """user_input = int(input("Enter a number: \n0= exit \n1= Rumpelstiltskin \n2= The Emperor's New Clothes \n"))
        
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
        """
        
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
            time.sleep(0.1)
        

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

    def book1(self): # Rumpelstiltskin
        
        print("[AUDIOBOOK] Playing opening line Rumpelstiltskin")
        audiobook_open_file = 'mp3_files/rumpelstiltskin_open.mp3'
        play_open_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_open_file,))
        play_open_thread.start()
        self.add_timer(6, self.joints_movement.nod, (2,1))
        play_open_thread.join()
               

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
        
        self.add_timer(99, self.joints_controller.move_pitch, (4, -10))
        self.add_timer(99, self.joints_controller.move_yaw, (4, 25))
        
        self.add_timer(103, self.led_controller.turn_on_led, ((255, 255, 0), 250))
        
        self.add_timer(105, self.joints_controller.move_yaw, (4, 0))
        

        self.add_timer(110, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(113, self.cosmetics_movement.ears_inwards, (2,))
        
        self.add_timer(116, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(116, self.joints_controller.move_neck, (3, 20))
        
        self.add_timer(117, self.led_controller.fade_in_led, (2, (255, 165, 0), 250))
	
        self.add_timer(121, self.cosmetics_movement.close_eyes, (1,))
	
        self.add_timer(123, self.joints_controller.move_neck, (4, 34))
        self.add_timer(123, self.cosmetics_movement.open_eyes, (3,))
        
        self.add_timer(130, self.joints_controller.move_yaw, (3, 0))
        
        self.add_timer(132, self.cosmetics_controller.move_ears, (2, 0.6))
        self.add_timer(135, self.joints_controller.move_neck, (4, 28))
	
        self.add_timer(137, self.joints_movement.nod, (3, 3))
	
        self.add_timer(141, self.led_controller.fade_in_led, (2, (255, 255, 0), 250))
	
        self.add_timer(145, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(149, self.cosmetics_movement.ears_inwards, (2,))
        self.add_timer(151, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(151, self.joints_controller.move_neck, (3, 20))
        
        self.add_timer(159, self.cosmetics_movement.blink_eyes, (1, 2))
        self.add_timer(162, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(164, self.joints_movement.nod, (3, 2))
        self.add_timer(169, self.cosmetics_movement.ears_inwards, (2,))
        
        self.add_timer(172, self.joints_controller.move_pitch, (4, 0))
        self.add_timer(178, self.joints_movement.shake, (1, 2))
	
        self.add_timer(181, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(187, self.joints_controller.move_pitch, (3, 7))
        self.add_timer(190, self.joints_controller.move_yaw, (3, 0))
	
        self.add_timer(194, self.joints_movement.nod, (2, 2))
        self.add_timer(196, self.led_controller.fade_in_led, (4, (255, 255, 0), 250))
        self.add_timer(200, self.led_controller.fade_out_led, (2,))
	
        self.add_timer(203, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(208, self.joints_controller.move_pitch, (3, 7))
        self.add_timer(208, self.joints_controller.move_yaw, (3, -25))
	
        self.add_timer(212, self.joints_movement.shake, (3, 2))
        self.add_timer(215, self.cosmetics_controller.move_ears, (2, 0))
        self.add_timer(221, self.joints_controller.move_yaw, (2, 20))
        self.add_timer(224, self.joints_controller.move_yaw, (3, 0))
        self.add_timer(225, self.cosmetics_movement.blink_eyes, (1, 2))
        

        self.add_timer(229, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(229, self.joints_controller.move_neck, (3, 30))
        self.add_timer(231, self.led_controller.fade_in_led, (2, (255, 255, 0), 250))
        self.add_timer(234, self.joints_movement.shake, (3, 2))
        
        self.add_timer(242, self.joints_controller.move_neck, (3, 30))
        self.add_timer(242, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(242, self.led_controller.fade_out_led, (2,))
        
        self.add_timer(246, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(252, self.joints_movement.nod, (3, 2))
        
        self.add_timer(255, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(255, self.joints_controller.move_neck, (3, 20))
        
        self.add_timer(259, self.joints_controller.move_pitch, (2, -15))
        
        self.add_timer(261, self.joints_controller.move_yaw, (4, 25))
        
        self.add_timer(266, self.joints_movement.nod, (3, 3))
        self.add_timer(275, self.joints_movement.shake, (2, 2))
        
        self.add_timer(280, self.cosmetics_controller.move_ears, (2, 0.6))
        
        self.add_timer(285, self.joints_movement.nod, (3, 3))
        self.add_timer(288, self.joints_movement.shake, (2, 2))
        
        self.add_timer(295, self.cosmetics_controller.move_ears, (2, 0))
        
        self.add_timer(297, self.joints_controller.move_yaw, (3, 25))
        
        self.add_timer(302, self.cosmetics_movement.eyes_squint, (1,))
        self.add_timer(305, self.led_controller.fade_in_led, (2, (255, 65, 0), 250))
        self.add_timer(307, self.led_controller.fade_in_led, (2, (255, 165, 0), 250))
        
        self.add_timer(309, self.cosmetics_movement.open_eyes, (3,))
        self.add_timer(315, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(318, self.joints_controller.move_yaw, (4, 0))
        self.add_timer(324, self.joints_controller.move_pitch, (3, -15))
	  
        self.add_timer(329, self.cosmetics_controller.move_ears, (3, 0))
        self.add_timer(329, self.joints_controller.move_pitch, (3, 0))
        self.add_timer(337, self.joints_movement.shake, (2, 2))
        self.add_timer(342, self.joints_movement.shake, (2, 2))
	
        self.add_timer(345, self.joints_controller.move_pitch, (3, -15))
        self.add_timer(348, self.joints_movement.shake, (1, 1))
        self.add_timer(350, self.joints_movement.shake, (1, 1))
        self.add_timer(353, self.led_controller.fade_in_led, (2, (255, 0, 0), 250))
        self.add_timer(359, self.cosmetics_movement.blink_eyes, (1, 1))
        self.add_timer(362, self.cosmetics_movement.blink_eyes, (1, 1))
        self.add_timer(365, self.led_controller.fade_out_led, (2,))
        self.add_timer(367, self.cosmetics_controller.move_ears, (3, 1))
        self.add_timer(370, self.led_controller.toggle_led_sections, (15, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
	
        self.add_timer(374, self.led_controller.fade_out_led, (2,))
        self.add_timer(374, self.cosmetics_controller.move_ears, (2, 0))
        

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        
        
        
        print("[AUDIOBOOK] Playing ending line Rumpelstiltskin")
        audiobook_end_file = 'mp3_files/rumpelstiltskin_end.mp3'
        play_end_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_end_file,))
        play_end_thread.start()
        self.add_timer(0, self.joints_movement.nod, (2,1))
        play_end_thread.join()
        

        
        

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
            time.sleep(0.1)

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

