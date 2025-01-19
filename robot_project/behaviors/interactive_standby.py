#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 8, 2024
# Last modified: December 16, 2024
#
# Main default program for MiRo when it is not undergoing some pre-programmed activity

import threading
import time
import math
import cv2
import rospy
import random

from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class

from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class

from actuators.play_audio import play_audio  # function

from actuators.stereovision import Stereovision

from actuators.speech_to_text import SpeechToText

# Sensor nodes to import
from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *
from IS_modules.attend_face import *

ACT_ENGAGE = 1 # attempt to engage with person
ACT_IDLE = 2 # do some idle animations


BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

NECK_MAX = miro.constants.LIFT_RAD_MAX
NECK_MIN = miro.constants.LIFT_RAD_MIN
neck_upper = 15.0 # deg
neck_lower = 40.0 # deg

PITCH_MAX = miro.constants.PITCH_RAD_MAX
PITCH_MIN = miro.constants.PITCH_RAD_MIN
pitch_upper = -5.0 # deg
pitch_lower = -20.0 # deg

led_lower = 20
led_upper = 250

BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

class interactive_standby:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()


        self.stereovision = Stereovision()
        self.speech_to_text = SpeechToText()

        # Make detector nodes
        self.aruco_detect = NodeDetectAruco()
        self.touch_detect = see_touch()
        self.track_face = True
        # Make behaviour tracking variable
        self.behaviour = INTERACTIVE_STANDBY

        self.delay = 5 # random delay, in seconds. keep it between 3-10 seconds
        self.current_color = (255,255,255)

        self.wait = False

    def random_delay(self, low, high):
        return random.randint(low, high)

    def move_randomly(self):
        self.joints_controller.position_neck(random.randint(15,40))
        self.joints_controller.position_pitch(random.randint(-22,8))
        self.joints_controller.position_yaw(random.randint(-55,55))

        # move_neck_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, random.randint(15,40)))
        # move_pitch_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, random.randint(-22,8)))
        # move_yaw_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, random.randint(-55,55)))

        # move_neck_thread.start()
        # move_pitch_thread.start()
        # move_yaw_thread.start()

    def move_idle(self):
        rand = random.random()
        if rand < 0.5:
            self.joints_movement.nod(2, 2)
        else:
            self.joints_movement.shake(2, 2)        

    def get_activity_level(self, distance):
        if distance < 1.5:
            return ACT_ENGAGE
        else:
            return ACT_IDLE

    def run(self):
        # Start the stereovision in a separate thread
        stereovision_thread = threading.Thread(target=self.stereovision.run)
        stereovision_thread.start()

        # speech_to_text_thread = threading.Thread(target=self.speech_to_text.run)
        # speech_to_text_thread.daemon = True
        # speech_to_text_thread.start()

        # Make behaviour tracking variable
        self.behaviour = INTERACTIVE_STANDBY


    def loop(self):
        """
        Main control loop
        """

        while not rospy.core.is_shutdown():
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            # Detect touch
            self.touch_detect.check_touch()

            if self.aruco_detect.breath_ex_ON:
                print("Activated the breathing exercise through aruco codes")
                self.behaviour = BREATHING_EXERCISE
                self.aruco_detect.breath_ex_ON = False
                break
            
            if self.touch_detect.breath_ex_ON:
                print("Activated the breathing exercise through touch")
                self.behaviour = BREATHING_EXERCISE
                self.touch_detect.breath_ex_ON = False
                break

            # words_to_check = ['breathe', 'breathing', 'exercise']
            # if any(word in self.speech_to_text.get_last_text().lower() for word in words_to_check):                
            #     print("Activated the breathing exercise through speech")
            #     self.behaviour = BREATHING_EXERCISE
            #     break

            ##### Interactive Standby #####

            current_time = time.time()
            if current_time - last_time >= self.delay: # every # seconds
                #print number of threads running
                print(f"Thread count: {threading.active_count()}")
                audio_file = None
                if self.stereovision.face_detected:
                    #print(f"Face detected at distance: {self.stereovision.face_distance}")
                    activity_level = self.get_activity_level(self.stereovision.face_distance)
                else:
                    self.wait = False
                    activity_level = ACT_IDLE
                    # Move randomly
                    self.move_randomly()

                if activity_level == ACT_IDLE:
                    self.delay = self.random_delay(7,10)
                    self.current_color = (0, 0, 255)  # Blue
                    audio_file = None

                elif activity_level == ACT_ENGAGE and not self.wait:
                    self.delay = self.random_delay(2,4)
                    self.current_color = (255, 0, 0)  # Red
                    audio_file = 'hi_there.mp3'
                    play_thread = threading.Thread(target=play_audio, args=(audio_file,))
                    play_thread.start()
                    self.wait = True

                self.led_controller.turn_on_led(self.current_color, 250)

                last_time = current_time

            #### End of 5 second loop ####

            # Yield
            #self.stereovision.draw_frames()

        print("Exited the loop")
