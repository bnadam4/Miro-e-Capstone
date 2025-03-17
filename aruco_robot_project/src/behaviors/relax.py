#!/usr/bin/python3

# ----------------------------------------------
# Title: relax.py
# Description: Muscle Relaxation Exercise
# Author: James 
# Date created: Jan 4, 2024
# Date modified: Feb 20, 2024
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
from IS_modules.detect_touch import *

class RelaxBehavior:
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

        # Initialize touch detection
        self.touch_detect = see_touch()

    def run(self):
        print("[RELAXATION] Running relaxation behavior")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= Relax Back \n2= Relax Arms \n3= Relax Tummy \n4= Relax Legs \n5= Full Relaxation\n"))

        if user_input == 0:
            print("[RELAXATION] Exiting program...")
        elif user_input == 1:
            print("1) Relax Back")
            self.relax_back()
        elif user_input == 2:
            print("2) Relax Arms")
            self.relax_arms()
        elif user_input == 3:
            print("3) Relax Tummy")
            self.relax_tummy()
        elif user_input == 4:
            print("4) Relax Legs")
            self.relax_legs()
        elif user_input == 5:
            print("5) Full Relaxation")
            self.full_relaxation()
        else:
            print("[RELAXATION] Invalid choice or unrecognized number.")
    def intro(self):
        print("\n\n")
        print("****************************\n\n")
        print("RELAX BEHAVIOUR STARTED")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
                
        relax_prompt = 'mp3_files/relaxV2/relax_promptV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(relax_prompt,))
        play_thread.start()
        
        # Schedule all movements
        self.add_timer(1, self.joints_movement.nod, (2, 2))
        self.add_timer(4, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(9, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(10, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(12, self.joints_controller.move_neck, (2, 35))
        self.add_timer(14, self.joints_controller.move_neck, (2, 0))
        self.add_timer(15, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(16, self.cosmetics_movement.open_eyes, (1,))

        # Wait for the first set of actions to finish
        play_thread.join()
        
        start_time = time.time()  # Record the start time
        while True:
            self.aruco_detect.tick_camera()

            # Check if any Aruco code is detected and proceed accordingly
            if self.aruco_detect.relax_arms:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax arms.")
                self.aruco_detect.relax_arms = False
                self.relax_back()
                break
            if self.aruco_detect.relax_back:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax back.")
                self.aruco_detect.relax_back = False
                self.relax_back()
                break
            if self.aruco_detect.relax_legs:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax legs.")
                self.aruco_detect.relax_legs = False
                self.relax_legs()
                break
            if self.aruco_detect.relax_tummy:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax tummy.")
                self.aruco_detect.relax_tummy = False
                self.relax_tummy()
                break
            if self.aruco_detect.relax_all:
                print("[AUDIOBOOK] Aruco code detected, proceeding to relax all.")
                self.aruco_detect.relax_all = False
                self.relax_all()
                break

            # Check if 10 seconds have passed
            if time.time() - start_time > 10:
                print("No Aruco code detected")
                break

            # Sleep for a short time to avoid excessive CPU usage
            time.sleep(0.1)

    def relax_arms_solo(self): #arms can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.relax_arms()
        self.relax_complete()

    def relax_back_solo(self): #back can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.relax_back()
        self.relax_complete()

    def relax_tummy_solo(self): #tummy can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.relax_tummy()
        self.relax_complete()

    def relax_legs_solo(self): #legs can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.relax_legs()
        self.relax_complete()

    def wait_for_head_touch(self):
        print("Playing relaxation prompt audio...")
        self.audio_player.play_audio('mp3_files/relaxV2/relax_beginV2.mp3')

        # Wait for touch or timeout
        start_time = time.time()
        while (time.time() - start_time) < 10:
            self.touch_detect.check_touch()
            if self.touch_detect.head_touched:
                print("Head touched! Starting relaxation.")
                return True
            time.sleep(0.1)
        print("Timeout. No head touch detected. Exiting relaxation.")
        return False

    def relax_back(self):
        print("[RELAXATION] Starting back relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relaxV2/relax_backV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

    def relax_arms(self):
        print("[RELAXATION] Starting arms relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relaxV2/relax_armsV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

    def relax_tummy(self):
        print("[RELAXATION] Starting tummy relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relaxV2/relax_tummyV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

    def relax_legs(self):
        print("[RELAXATION] Starting legs relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relaxV2/relax_legsV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

    def full_relaxation(self):
        print("[RELAXATION] Starting full relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        self.relax_arms()
        self.relax_back()
        self.relax_tummy()
        self.relax_legs()
        self.relax_complete()

    def relax_complete(self):
        audio_file = 'mp3_files/relaxV2/relax_completeV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()


        # Schedule all movements
        self.add_timer(1, self.led_controller.fade_in_led, (2, (0, 255, 0), 150))
        self.add_timer(5, self.led_controller.fade_out_led, (2,))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "left"))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "right"))
        self.add_timer(6, self.cosmetics_movement.wagging_tail, (2, 3))

        # Wait for the first set of actions to finish
        play_thread.join()


    ###Exit###
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
                print("[RELAXATION] Exit behaviour detected, stopping relaxation exercise.")

            time.sleep(0.1)

    def safe_execute(self, func, args):
        """Execute function only if stop flag is not set and parent thread is still alive."""
        if not self.stop_flag and (self.parent_thread and self.parent_thread.is_alive()):
            func(*args)
        else:
            print(f"[RELAXATION] Skipping {func.__name__} because parent thread has exited.")

    # Function to safely start actions
    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: self.safe_execute(func, args))
        self.timers.append(timer)
        timer.start()
