#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 8, 2024
# Last modified: February 5, 2025
#
# Main default program for MiRo when it is not undergoing some pre-programmed activity

import threading
import time
import math
import cv2
import rospy
import random

import miro2 as miro

from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class

from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class

from actuators.wheels_controller import WheelsController #class

from actuators.play_audio import AudioPlayer  # class

from actuators.stereovision import Stereovision

from actuators.speech_to_text import SpeechToText

# Sensor nodes to import
from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *

ACT_ENGAGE = 1 # attempt to engage with person
ACT_IDLE = 2 # do some idle animations
ACT_LISTEN = 3 # listen to person

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

MUSCLE_RELAXATION = 1
AUDIOBOOK = 2
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

        self.wheels_controller = WheelsController()

        self.stereovision = Stereovision()
        self.speech_to_text = SpeechToText()
        self.audio_player = AudioPlayer()

        # Make detector nodes
        self.aruco_detect = NodeDetectAruco()
        self.touch_detect = see_touch()
        self.track_face = True
        # Make behaviour tracking variable
        self.behaviour = INTERACTIVE_STANDBY

        self.delay = 5 # random delay, in seconds. keep it between 3-10 seconds
        self.current_color = (255,255,255)

        self.wait = False
        self.petted = False
        self.crinkled = False
        self.speaking = False
        self.activity_level = ACT_IDLE

    def random_delay(self, low, high):
        return random.randint(low, high)

    def move_randomly(self):
        rand_neck = random.randint(15,40)
        rand_pitch = random.randint(-22,8)
        rand_yaw = random.randint(-55,55)

        move_thread = threading.Thread(target=self.joints_controller.move_all, args=(2,rand_yaw,rand_pitch,rand_neck))
        move_thread.daemon = True
        move_thread.start()

    def rotate_randomly(self):
        rand_speed = random.choice([-3, -2, -1, 1, 2, 3])
        rand_duration = 1

        rotate_thread = threading.Thread(target=self.wheels_controller.rotate, args=(rand_speed, rand_duration))
        rotate_thread.daemon = True
        rotate_thread.start()

    def get_activity_level(self, distance):
        if distance < 1.5:
            print("Engage")
            return ACT_ENGAGE
        else:
            print("Idle")
            return ACT_IDLE
        
    def engage(self, delay):
        return  ACT_ENGAGE, delay + 5
        
    def blink(self):
        blink_thread = threading.Thread(target=self.cosmetics_movement.close_eyes, args=(0.2, ))
        blink_thread.start()
        threading.Timer(0.2, lambda: threading.Thread(target=self.cosmetics_movement.open_eyes, args=(0.2, )).start()).start()

    def loop(self):
        """
        Main control loop
        """
        # Start the stereovision in a separate thread
        stereovision_thread = threading.Thread(target=self.stereovision.loop)
        stereovision_thread.daemon = True
        stereovision_thread.start()

        speech_to_text_thread = threading.Thread(target=self.speech_to_text.loop)
        speech_to_text_thread.daemon = True
        speech_to_text_thread.start()

        # Make behaviour tracking variable
        self.behaviour = INTERACTIVE_STANDBY

        last_time = time.time()
        last_pet = time.time()
        last_blink = time.time()
        last_crinkle = time.time()
        last_spoke = time.time()
        trigger_time = time.time()
        blink_delay = self.random_delay(5, 20)
        speak_delay = 5.0
        triggered = False

        last_react = 9

        print("\n\n")
        print("****************************\n\n")
        print("INTERACTIVE STANDBY STARTED\n\n")
        print("****************************\n\n")

        while not rospy.core.is_shutdown():
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            # Detect touch
            self.touch_detect.check_touch()

            trigger_words_to_check = ['miro', 'mirror', 'nero', 'amira', 'amiru', 'mural', 'neural', 'emira', 'amero', 'mira']
            breath_words_to_check = ['breathe', 'breathing', 'breath']
            audiobooks_words_to_check = ['audiobook', 'audio', 'book', 'story']
            muscle_words_to_check = ['muscle', 'relaxation', 'stretch', 'relax']

            if 'what' in self.speech_to_text.last_text.lower() and 'do' in self.speech_to_text.last_text.lower() and 'can' in self.speech_to_text.last_text.lower() and not self.speaking:
                self.speaking = True
                print("Activated capability response")
                audio_file = 'mp3_files/what_can_i_do.mp3'
                play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                play_thread.start()
                play_thread.join()
                self.speech_to_text.last_text= ''
                self.speaking = False

            if any(word in self.speech_to_text.last_text.lower() for word in trigger_words_to_check) or triggered:
                print("\nTrigger word heard\n")
                self.activity_level = ACT_LISTEN

                if not triggered:
                    trigger_time = time.time()
                    ear_thread = threading.Thread(target=self.cosmetics_movement.ear_outwards, args=(1, ))
                    ear_thread.start()
                
                triggered = True
                print(f"Time remaining: {10 - (time.time() - trigger_time)}")

                self.current_color = (255, 255, 0)  # Yellow
                self.led_controller.turn_on_led(self.current_color, 250)

            if time.time() - trigger_time >= 10 and triggered:
                print("\nResetting trigger\n")
                self.current_color = (0, 0, 255)  # Blue
                self.led_controller.turn_on_led(self.current_color, 250)
                triggered = False
                self.activity_level = ACT_IDLE
                ear_thread = threading.Thread(target=self.cosmetics_movement.ears_inwards, args=(1, ))
                ear_thread.start()

                

            if self.aruco_detect.breath_ex_ON or any(word in self.speech_to_text.last_text.lower() for word in breath_words_to_check) and not self.speaking and triggered:
                # audio_file = 'mp3_files/want_to_do_breath_ex.mp3'
                # play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                # play_thread.start()
                # play_thread.join()
                # self.speech_to_text.last_text= ''
                # self.speaking = False

                print("Activated the breathing exercise")
                self.behaviour = BREATHING_EXERCISE
                self.aruco_detect.breath_ex_ON = False
                self.stereovision.stop = True
                self.speech_to_text.stop = True
                break

            elif self.aruco_detect.muscle_relax_ON or any(word in self.speech_to_text.last_text.lower() for word in muscle_words_to_check) and not self.speaking and triggered:
                # audio_file = 'mp3_files/want_to_do_muscle_relax.mp3'
                # play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                # play_thread.start()
                # play_thread.join()
                # self.speech_to_text.last_text= ''
                # self.speaking = False

                print("Activated the muscle relaxation exercise")
                self.behaviour = MUSCLE_RELAXATION
                self.aruco_detect.muscle_relax_ON = False
                self.stereovision.stop = True
                self.speech_to_text.stop = True
                break

            elif self.aruco_detect.audiobook_ON or any(word in self.speech_to_text.last_text.lower() for word in audiobooks_words_to_check) and not self.speaking and triggered:
                # audio_file = 'mp3_files/want_to_do_audiobook.mp3'
                # play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                # play_thread.start()
                # play_thread.join()
                # self.speech_to_text.last_text= ''
                # self.speaking = False

                print("Activated the audio book")
                self.behaviour = AUDIOBOOK
                self.aruco_detect.audiobook_ON = False
                self.stereovision.stop = True
                self.speech_to_text.stop = True
                break

            ##### Interactive Standby #####

            current_time = time.time()

            if current_time - last_time >= self.delay: # every # seconds
                #print number of threads running
                print(f"Thread count: {threading.active_count()}")
                audio_file = None
                if self.stereovision.face_detected:
                    print(f"Face detected at distance: {self.stereovision.face_distance}")
                    self.activity_level = self.get_activity_level(self.stereovision.face_distance)
                    self.delay = 30 # Set high delay so that the robot continues to engage
                elif self.activity_level != ACT_LISTEN:
                    self.wait = False
                    self.activity_level = ACT_IDLE
                    # Move randomly
                    self.move_randomly()
                    #self.rotate_randomly()
                    self.speaking = True
                    speak_delay = 0.3
                    last_spoke = time.time()

                if self.activity_level == ACT_IDLE:
                    self.delay = self.random_delay(3,5)
                    self.current_color = (0, 0, 255)  # Blue
                    audio_file = None
                elif self.activity_level == ACT_ENGAGE and not self.wait:
                    self.delay = self.random_delay(2,3)
                    self.current_color = (0, 255, 0)  # Green
    
                    audio_file = 'mp3_files/hi_there.mp3'
                    play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                    play_thread.start()

                    self.wait = True
                    self.speaking = True
                    last_spoke = time.time()
                    speak_delay = 5.0

                self.led_controller.turn_on_led(self.current_color, 250)

                last_time = current_time

            # Add time to ACT_ENGAGE state if face detected
            if self.stereovision.face_detected and self.activity_level == ACT_ENGAGE:
                if current_time - last_time + 5 >= self.delay:
                    self.delay += 10 # Add time to ACT_ENGAGE state
                    self.current_color = (0, 255, 0)  # Green
                    print(f"\nTime added to engaged state\n")
                    print(f"New delay: {self.delay}")


            rand_pet = random.randint(0, 3)
            # Check if being pet and act accordingly
            if not self.petted and not self.speaking:
                self.touch_detect.check_touch()
                if self.touch_detect.head_touched:
                    self.petted = True
                    self.wait = True
                    #self.activity_level, self.delay = self.engage(self.delay)
                    last_spoke = time.time()
                    speak_delay = 3.0
                    self.speaking = True

                    if rand_pet == 0 and last_react != 0:
                        last_react = 0
                        print("Petting 1: Ears")
                        audio_file = 'mp3_files/Petting_1.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        ear_thread = threading.Thread(target=self.cosmetics_movement.ear_outwards, args=(1, ))
                        ear_thread.start()
                        threading.Timer(1.0, lambda: threading.Thread(target=self.cosmetics_movement.ears_inwards, args=(1, )).start()).start()
                    elif rand_pet == 1 and last_react != 1:
                        last_react = 1
                        print("Petting 2: Eyes")
                        audio_file = 'mp3_files/Petting_2.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        # Narrow MiRo's eyes in contentment
                        eye_thread = threading.Thread(target=self.cosmetics_movement.eyes_squint, args=(2, ))
                        eye_thread.start()
                    elif rand_pet == 2 and last_react != 2:
                        last_react = 2
                        print("Petting 3: Head nod")
                        audio_file = 'mp3_files/Petting_3.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        # Nod MiRo's head
                        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2, 2, ))
                        head_thread.start()
                    elif rand_pet == 3:
                        last_react = 3
                        # Occasionally do no respond to petting
                        # Wag MiRo's tail
                        print("Petting 4: Tail wag")
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()

                    last_pet = time.time()

                elif self.touch_detect.body_touched and not self.speaking:
                    self.petted = True
                    self.wait = True

                    last_spoke = time.time()
                    speak_delay = 3.0
                    self.speaking = True

                    if rand_pet == 0 and last_react != 0:
                        last_react = 0
                        print("Petting 5: Thanks")
                        audio_file = 'mp3_files/Pet_back1.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()
                        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2, 2, ))
                        head_thread.start()
                    elif rand_pet == 1 and last_react != 1:
                        last_react = 1
                        print("Petting 6: Ticklish")
                        audio_file = 'mp3_files/Pet_back2.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        head_thread = threading.Thread(target=self.joints_movement.shake, args=(1, 2, ))
                        head_thread.start()
                    elif rand_pet == 2 and last_react != 2:
                        last_react = 2
                        print("Petting 7: Hehe")
                        audio_file = 'mp3_files/Pet_back3.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()
                        self.move_randomly()
                    elif rand_pet == 3:
                        last_react = 3
                        # Occasionally do no respond to petting
                        # Wag MiRo's tail
                        print("Petting 8: Tail wag")
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()

                    last_pet = time.time()

            # Check if being crinkled and act accordingly
            if not self.crinkled and not self.speaking and not self.wait and not self.petted:
                if self.speech_to_text.right_crinkle or self.speech_to_text.left_crinkle:
                    print("Crinkling Right Ear")
                    self.crinkled = True
                    last_crinkle = time.time()
                    if rand_pet == 0 and last_react != 0:
                        last_react = 0
                        audio_file = 'mp3_files/Ear_touch_1.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        eye_thread = threading.Thread(target=self.cosmetics_movement.eyes_squint, args=(1, ))
                        eye_thread.start()
                    elif rand_pet == 1 and last_react != 1:
                        last_react = 1
                        audio_file = 'mp3_files/Petting_1.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()
                    elif rand_pet == 2 and last_react != 2:
                        last_react = 2
                        audio_file = 'mp3_files/Ear_touch_2.mp3'
                        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                        play_thread.start()
                        self.move_randomly()
                    elif rand_pet == 3:
                        last_react = 3
                        # Occasionally do not respond to petting with a voice line
                        # Wag MiRo's tail
                        tail_thread = threading.Thread(target=self.cosmetics_controller.wagging, args=(3.0, 15.0, ))
                        tail_thread.start()
                elif self.speech_to_text.tail_crinkle:
                    self.crinkled = True
                    print("Crinkling tail")
                    audio_file = 'mp3_files/Tail_touch_1.mp3'
                    play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
                    play_thread.start()
                    head_thread = threading.Thread(target=self.joints_movement.shake, args=(1, 2, ))
                    head_thread.start()
                    last_crinkle = time.time()

            # Reset petted variable if not petted for a certain time
            if current_time - last_pet >= 4 and self.petted:
                self.petted = False
                eye_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(2, ))
                eye_thread.start()
                self.wait = False

            # Reset crinkle variable if not petted for a certain time
            if current_time - last_crinkle >= 5 and self.crinkled:
                self.crinkled = False
                eye_thread = threading.Thread(target=self.cosmetics_movement.open_eyes, args=(2, ))
                eye_thread.start()
                self.wait = False

            # Reset speak variable if not spoken for a certain time
            if current_time - last_spoke >= speak_delay and self.speaking:
                # print("Set speaking to False")
                self.speaking = False

            #### End of 5 second loop ####

            # Blink every 10-30 seconds
            if current_time - last_blink >= blink_delay:
                print("Blinking")
                self.speaking = True
                speak_delay = 0.5
                if not self.petted and not self.crinkled:
                    self.blink()
                last_blink = current_time
                blink_delay = self.random_delay(10, 30)

            # Yield
            rospy.sleep(0.02)
            # self.stereovision.draw_frames()

        print("Exited the loop")
