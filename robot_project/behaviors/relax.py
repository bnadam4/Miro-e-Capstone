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
        print("\n\n")
        print("****************************\n\n")
        print("RELAX BEHAVIOUR STARTED")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        
                
        relax_prompt = 'mp3_files/relax/relax_prompt.mp3'
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
        

    def wait_for_head_touch(self):
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("Playing relaxation prompt audio...")

        audio_file = 'mp3_files/relax/relax_begin.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()
        # Wait for touch or timeout
        start_time = time.time()
        while ((time.time() - start_time) < 20) and (self.stop_flag == False):
            
            self.touch_detect.check_touch()
            if self.touch_detect.head_touched:
                print("Head touched! Starting relaxation.")
                return True
            time.sleep(0.1)
        print("Timeout. No head touch detected. Exiting relaxation.")
        play_thread.join()
        return False

    def relax_back(self):
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting back relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relax/relax_back.mp3'
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
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting arms relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relax/relax_arms.mp3'
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
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting tummy relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relax/relax_tummy.mp3'
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
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting legs relaxation")
        if not self.wait_for_head_touch():
            return  # Exit if no head touch detected within 10 seconds

        audio_file = 'mp3_files/relax/relax_legs.mp3'
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
        if self.stop_flag==False:
            self.relax_arms()
        if self.stop_flag==False:
            self.relax_back()
        if self.stop_flag==False:
            self.relax_tummy()
        if self.stop_flag ==False:
            self.relax_legs()
        self.relax_complete()

    def relax_complete(self):
        audio_file = 'mp3_files/relax/relax_complete.mp3'
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
