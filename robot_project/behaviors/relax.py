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

from actuators.play_audio import play_audio  # function
#from robot_interface import RobotInterface  # class

from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *

from actuators.speech_to_text import SpeechToText

class RelaxBehavior:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()
        #self.robot_interface = RobotInterface()
        self.state = "relax_prep"
        self.timers = []  # List to track timers
        self.flag = None  # Initialize flag variable
        
        speech_to_text_thread = threading.Thread(target=self.speech_to_text.loop)
        speech_to_text_thread.daemon = True
        speech_to_text_thread.start()

    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            if self.aruco_detect.exit_behaviour:
                self.stop_flag = True
                self.audio_player.stop()
                print("[RELAXATION] Exit behaviour detected, stopping audiobook.")
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

    def run(self):
        self.relax_begin()
        self.wait_for_prompt()
        self.relax_prompt()
        if self.flag == "relax_back":
            self.relax_back()
        elif self.flag == "relax_arms":
            self.relax_arms()
        elif self.flag == "relax_tummy":
            self.relax_tummy()
        elif self.flag == "relax_legs":
            self.relax_legs()
        elif self.flag == "relax_full":
            self.relax_back()
            self.relax_arms()
            self.relax_tummy()
            self.relax_legs()
        else:
            self.exit()

    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: func(*args))
        self.timers.append(timer)
        timer.start()

    def relax_begin(self):
        audio_file = 'mp3_files/relax_begin.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
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
        for timer in self.timers:
            timer.join()
        time.sleep(5)

    def wait_for_prompt(self):
        print("[RELAXATION] Waiting for head touch or aruco code...")
        while True:
            self.aruco_detect.tick_camera()
            self.touch_detect.check_touch()

            if self.aruco_detect.relax_prompt:
                print("[RELAXATION] Aruco code detected, proceeding to relax_prompt.")
                self.aruco_detect.relax_prompt = False
                break
            if self.touch_detect.head_touched:
                print("[RELAXATION] Head touch detected, proceeding to relax_prompt.")
                break
            time.sleep(0.1)
    
    def relax_prompt(self):
        audio_file = 'mp3_files/relax_prompt.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
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
        for timer in self.timers:
            timer.join()
        time.sleep(5)

        full_check = ['full', 'everything', 'all']
        back_check = ['back']
        arms_check = ['arms']
        tummy_check = ['tummy']
        legs_check = ['legs']

        if any(word in self.speech_to_text.last_text.lower() for word in full_check):
            self.flag = "relax_full"
        elif any(word in self.speech_to_text.last_text.lower() for word in back_check):
            self.flag = "relax_back"
        elif any(word in self.speech_to_text.last_text.lower() for word in arms_check):
            self.flag = "relax_arms"
        elif any(word in self.speech_to_text.last_text.lower() for word in tummy_check):
            self.flag = "relax_tummy"
        elif any(word in self.speech_to_text.last_text.lower() for word in legs_check):
            self.flag = "relax_legs"
        else:
            self.flag = None

    def relax_back(self):
        audio_file = 'mp3_files/relax_back.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (7, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for the first set of actions to finish
        play_thread.join()
        for timer in self.timers:
            timer.join()
        time.sleep(5)

        #self.state = "relax_arms"

    def relax_arms(self):
        audio_file = 'mp3_files/relax_arms.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (7, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for the first set of actions to finish
        play_thread.join()
        for timer in self.timers:
            timer.join()
        time.sleep(5)

        #self.state = "relax_tummy"

    def relax_tummy(self):
        audio_file = 'mp3_files/relax_tummy.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (7, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for the first set of actions to finish
        play_thread.join()
        for timer in self.timers:
            timer.join()
        time.sleep(5)

        #self.state = "relax_legs"

    def relax_legs(self):
        audio_file = 'mp3_files/relax_legs.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(16, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))
        self.add_timer(17, self.led_controller.toggle_led_sections, (7, (255, 0, 0), (0, 0, 0), (0, 0, 0), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 255, 0), (0, 0, 0), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (7, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (7, (100, 0, 0), (0, 100, 0), (0, 0, 100), 250))

        # Wait for the first set of actions to finish
        play_thread.join()
        for timer in self.timers:
            timer.join()
        time.sleep(5)

        #self.state = "relax_complete"

    def relax_complete(self):
        audio_file = 'mp3_files/relax_complete.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(1, self.led_controller.fade_in_led, (2, (0, 255, 0), 150))
        self.add_timer(5, self.led_controller.fade_out_led, (2,))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "left"))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "right"))
        self.add_timer(6, self.cosmetics_movement.wagging_tail, (2, 3))

        # Wait for the first set of actions to finish
        play_thread.join()
        for timer in self.timers:
            timer.join()

        self.state = "complete"
    
    def exit(self):
        #play_thread.join()

        exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files/i_will_stop.mp3',))
        exit_behaviour_thread.start()
        exit_behaviour_thread.join()





