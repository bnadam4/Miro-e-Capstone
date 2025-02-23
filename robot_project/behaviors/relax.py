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
        
        speech_to_text_thread = threading.Thread(target=self.speech_to_text.loop)
        speech_to_text_thread.daemon = True
        speech_to_text_thread.start()

    def run(self):
        while self.state != "complete":
            if self.state == "relax_prep":
                self.relax_prep()
            elif self.state == "relax_back":
                self.relax_back()
            elif self.state == "relax_arms":
                self.relax_arms()
            elif self.state == "relax_tummy":
                self.relax_tummy()
            elif self.state == "relax_legs":
                self.relax_legs()
            elif self.state == "relax_complete":
                self.relax_complete()

    def relax_prep(self):
        audio_file = 'relax_prep.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Run ear movement from 0-2s
        time.sleep(1)
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        # Turn on LED from 1-8s
        time.sleep(3)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        # Blink eyes at 10s
        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(7)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        time.sleep(9)
        head_tilt_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, 35))
        head_tilt_thread.start()

        time.sleep(11)
        head_tilt_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, 0))
        head_tilt_thread.start()

        time.sleep(12)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(13)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        # Wait for the first set of actions to finish
        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()
        eyes_open_thread.join()
        play_thread.join()
        time.sleep(5)

        self.state = "relax_back"

    def relax_back(self):
        audio_file = 'relax_back.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(1)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        time.sleep(5)
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        time.sleep(6)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 0, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 255, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 0, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (100, 0, 0),(0, 100, 0),(0, 0, 100), 250))
        led_thread_1.start()

        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()
        eyes_open_thread.join()
        play_thread.join()
        time.sleep(5)

        self.state = "relax_arms"

    def relax_arms(self):
        audio_file = 'relax_arms.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(1)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        time.sleep(5)
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        time.sleep(6)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 0, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 255, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 0, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (100, 0, 0),(0, 100, 0),(0, 0, 100), 250))
        led_thread_1.start()

        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()
        eyes_open_thread.join()
        play_thread.join()
        time.sleep(5)

        self.state = "relax_tummy"

    def relax_tummy(self):
        audio_file = 'relax_tummy.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(1)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        time.sleep(5)
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        time.sleep(6)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 0, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 255, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 0, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (100, 0, 0),(0, 100, 0),(0, 0, 100), 250))
        led_thread_1.start()

        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()
        eyes_open_thread.join()
        play_thread.join()
        time.sleep(5)

        self.state = "relax_legs"

    def relax_legs(self):
        audio_file = 'relax_legs.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        time.sleep(5)
        eyes_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(1,))
        eyes_thread.start()

        time.sleep(1)
        eyes_open_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(1,))
        eyes_open_thread.start()

        time.sleep(5)
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2,2))
        head_thread.start()

        time.sleep(6)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 255, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (255, 0, 0),(0, 0, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 255, 0),(0, 0, 0), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (0, 0, 0),(0, 0, 0),(0, 0, 255), 250))
        led_thread_1.start()

        time.sleep(1)
        led_thread_1 = threading.Thread(target=self.led_controller.toggle_led_sections, args=(7, (100, 0, 0),(0, 100, 0),(0, 0, 100), 250))
        led_thread_1.start()

        head_thread.join()
        led_thread_1.join()
        eyes_thread.join()
        eyes_open_thread.join()
        play_thread.join()
        time.sleep(5)

        self.state = "relax_complete"

    def relax_complete(self):
        audio_file = 'relax_complete.mp3'
        play_thread = threading.Thread(target=play_audio, args=(audio_file,))
        play_thread.start()

        # Mimic cute animal movements (different from the previous set) - 3 movements only, affirming relaxation
        threading.Timer(1, lambda: threading.Thread(target=self.led_controller.fade_in_led, args=(2, (0, 255, 0), 150)).start()).start()
        threading.Timer(5, lambda: threading.Thread(target=self.led_controller.fade_out_led, args=(2,)).start()).start()

        threading.Timer(4, lambda: threading.Thread(target=self.cosmetics_movement.eye_wink, args=(1, 1, "left")).start()).start()
        threading.Timer(4, lambda: threading.Thread(target=self.cosmetics_movement.eye_wink, args=(1, 1, "right")).start()).start()

        threading.Timer(6, lambda: threading.Thread(target=self.cosmetics_movement.wagging_tail, args=(2, 3)).start()).start()

        play_thread.join()
        self.state = "complete"





