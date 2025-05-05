#!/usr/bin/python3

# ----------------------------------------------
# Title: dance.py
# Description: Do a fun dance
# Author: Bryce Adam
# Date created: Apr 15, 2025
# Date modified: May 2, 2025
# ----------------------------------------------

import threading
import time
import random
import rospy
from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class

from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class

from actuators.play_audio import AudioPlayer  # class

from actuators.wheels_controller import WheelsController #class

from IS_modules.node_detect_aruco import *

from actuators.wheels_controller import WheelsController #class
from actuators.remote import connect_remote, receive_data, send_data, close_connection

from actuators.speech_to_text import SpeechToText

LOW_RAND = 0
HIGH_RAND = 1

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
        self.move_duration = 0.5  # Default duration for movements
        self.move_time = 0.0 # Variable to help with the timing of the dance moves
        self.wheels_controller = WheelsController()
        self.remote_data=[0,0,0,0,0]

        self.speech_to_text = SpeechToText()


    def run(self):
        print("\n\n")
        print("****************************\n\n")
        print("DANCE STARTED\n\n")
        print("****************************\n\n")
        # Start speech to text
        self.speech_to_text.stop = False
        speech_to_text_thread = threading.Thread(target=self.speech_to_text.loop)
        speech_to_text_thread.daemon = True
        speech_to_text_thread.start()

        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
                
        dance_confirm = 'mp3_files_slushy/dance/dance_intro.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(dance_confirm,))
        play_thread.start()
        play_thread.join()

        move_time = 0.0

        # Start the dance party
        print("[DANCE] Start the dance party!")
        dance_num = random.randint(0, 3)
        if dance_num == 0:
            music_file = 'mp3_files_slushy/dance/music_0.mp3'
        elif dance_num == 1:
            music_file = 'mp3_files_slushy/dance/music_1.mp3'
        elif dance_num == 2:
            music_file = 'mp3_files_slushy/dance/music_2.mp3'
        elif dance_num == 3:
            music_file = 'mp3_files_slushy/dance/music_3.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(music_file,))
        play_thread.start()

        # Schedule all movements
        while move_time < 30.0:
            dance_num = random.randint(0, 4)
            if dance_num == 0:
                self.move_1(move_time)
            elif dance_num == 1:
                self.move_2(move_time)
            elif dance_num == 2:
                self.move_3(move_time)
            elif dance_num == 3:
                self.move_4(move_time)
            elif dance_num == 4:
                self.move_5(move_time)
            move_time += self.move_duration

        # Schedule led color changes
        led_time = 0.0
        while led_time < 30.0:
            led_time += 0.5
            color = self.rand_color()
            self.add_timer(led_time, self.led_controller.turn_on_led, (color, 250))

        print("[DANCE] All moves scheduled, waiting for the dance to finish...")
        
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[DANCE] Stopping dance...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

        print("[DANCE] Ending line")
        end_file = 'mp3_files_slushy/dance/dance_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(end_file,))
        play_thread.start()
        play_thread.join()
        rospy.sleep(1.5)

        # Wait for all threads to finish (including the audio playback)
        self.add_timer(0, self.joints_movement.nod, (2,1))
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)
        
    def move_1(self, move_time):
        print("[DANCE] Move 1: Turn and nod")
        self.move_duration = 2.5  # Duration of move_1
        self.add_timer(move_time, self.wheels_controller.rotate, (1.0, 0.5))
        self.add_timer(move_time, self.joints_movement.nod, (2, 2))
        self.add_timer(move_time + 0.5, self.wheels_controller.rotate, (-1.0, 1.0))
        self.add_timer(move_time + 1.5, self.wheels_controller.rotate, (1.0, 0.5))

    def move_2(self, move_time):
        print("[DANCE] Move 2: Turn and wiggle ears")
        self.move_duration = 2.0
        # self.add_timer(move_time, wheels_controller.rotate, args=(-2.0, 2.0)).start()).start()
        self.add_timer(move_time, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(move_time + 1.0, self.cosmetics_movement.ears_inwards, (1, ))
        self.add_timer(move_time, self.wheels_controller.rotate, (1.0, 0.5))
        self.add_timer(move_time + 0.5, self.wheels_controller.rotate, args=(-1.0, 1.0))
        self.add_timer(move_time + 1.5, self.wheels_controller.rotate, args=(1.0, 0.5))

    def move_3(self, move_time):
        print("[DANCE] Move 3: Shake head")
        self.move_duration = 2.0
        self.add_timer(move_time, self.joints_movement.shake, (1, 2))
        self.add_timer(move_time, self.wheels_controller.rotate, (1.0, 0.5))
        self.add_timer(move_time + 0.5, self.wheels_controller.rotate, (-1.0, 1.0))
        self.add_timer(move_time + 1.5, self.wheels_controller.rotate, (1.0, 0.5))

    def move_4(self, move_time):
        print("[DANCE] Move 4: Move neck")
        self.move_duration = 2.5
        self.add_timer(move_time, self.joints_controller.position_neck, (25, ))
        self.add_timer(move_time + 1.0, self.joints_controller.position_neck, (40, ))
        self.add_timer(move_time + 1.25, self.joints_controller.position_neck, (25, ))
        self.add_timer(move_time + 1.5, self.joints_controller.position_neck, (40, ))

    def move_5(self, move_time):
        print("[DANCE] Move 5: Move head side to side and wag tail")
        self.move_duration = 4.5
        self.add_timer(move_time, self.cosmetics_controller.wagging, (4.0, 15))
        self.add_timer(move_time, self.joints_controller.position_neck, (-55, ))
        self.add_timer(move_time + 1.0, self.joints_controller.position_yaw, (55, ))
        self.add_timer(move_time + 2.0, self.joints_controller.position_yaw, (-55, ))
        self.add_timer(move_time + 3.0, self.joints_controller.position_yaw, (55, ))
        self.add_timer(move_time + 4.0, self.joints_controller.position_yaw, (0, ))


    ##Exit###

    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            stop_words_to_check = ['stop']
            try:
                self.remote_data = receive_data()
            except Exception as e:
                pass
            if self.aruco_detect.exit_behaviour or self.remote_data[4]==2:
                self.stop_flag = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/i_will_stop.mp3',))
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
                self.speech_to_text.stop = True
                print("[DANCE] Exit behaviour detected, stopping the dance.")
                
                # Turn LEDs orange to indicate a transition period
                self.current_color = (255, 165, 0)  # Orange
                self.led_controller.turn_on_led(self.current_color, 250)

            time.sleep(0.1)

    def safe_execute(self, func, args):
        """Execute function only if stop flag is not set and parent thread is still alive."""
        if not self.stop_flag and (self.parent_thread and self.parent_thread.is_alive()):
            func(*args)
        else:
            print(f"[DANCE] Skipping {func.__name__} because parent thread has exited.")

    # Function to safely start actions
    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: self.safe_execute(func, args))
        self.timers.append(timer)
        timer.start()

    def rand_color(self):
        # Generate a random color
        rand_num = random.randint(0, 8)
        if rand_num == 0: # red
            r = 255
            g = 0
            b = 0
        elif rand_num == 1: #blue
            r = 0
            g = 0
            b = 255
        elif rand_num == 2: #green
            r = 0
            g = 255
            b = 0
        elif rand_num == 3:
            r = 255
            g = 255
            b = 0
        elif rand_num == 4:
            r = 255
            g = 165
            b = 0
        elif rand_num == 5:
            r = 255
            g = 0
            b = 255
        elif rand_num == 6:
            r = 255
            g = 255
            b = 255
        elif rand_num == 7:
            r = 0
            g = 255
            b = 255
        else:
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
        return (r, g, b)
