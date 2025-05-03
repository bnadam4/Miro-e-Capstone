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


    def run(self):
        print("\n\n")
        print("****************************\n\n")
        print("DANCE STARTED\n\n")
        print("****************************\n\n")
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
            dance_num = random.randint(0, 3)
            if dance_num == 0:
                self.move_1(move_time)
            elif dance_num == 1:
                self.move_2(move_time)
            elif dance_num == 2:
                self.move_3(move_time)
            elif dance_num == 3:
                self.move_4(move_time)
            move_time += self.move_duration
                

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
        threading.Timer(move_time, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(1.0, 0.5)).start()).start()
        threading.Timer(move_time, lambda: threading.Thread(target=self.joints_movement.nod, args=(2, 2)).start()).start()
        threading.Timer(move_time + 0.5, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(-1.0, 1.0)).start()).start()
        threading.Timer(move_time + 1.5, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(1.0, 0.5)).start()).start()

    def move_2(self, move_time):
        print("[DANCE] Move 2: Turn and wiggle ears")
        self.move_duration = 2.0
        # threading.Timer(move_time, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(-2.0, 2.0)).start()).start()
        threading.Timer(move_time, lambda: threading.Thread(target=self.cosmetics_movement.ear_outwards, args=(1,)).start()).start()
        threading.Timer(move_time + 1.0, lambda: threading.Thread(target=self.cosmetics_movement.ears_inwards, args=(1, )).start()).start()
        threading.Timer(move_time, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(1.0, 0.5)).start()).start()
        threading.Timer(move_time + 0.5, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(-1.0, 1.0)).start()).start()
        threading.Timer(move_time + 1.5, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(1.0, 0.5)).start()).start()

    def move_3(self, move_time):
        print("[DANCE] Move 3: Shake head")
        self.move_duration = 2.0
        threading.Timer(move_time, lambda: threading.Thread(target=self.joints_movement.shake, args=(1, 2)).start()).start()
        threading.Timer(move_time, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(1.0, 0.5)).start()).start()
        threading.Timer(move_time + 0.5, lambda: threading.Thread(target=self.wheels_controller.rotate, args=(-1.0, 1.0)).start()).start()

    def move_4(self, move_time):
        print("[DANCE] Move 4: Move neck")
        self.move_duration = 2.5
        threading.Timer(move_time, lambda: threading.Thread(target=self.joints_controller.position_neck, args=(15, )).start()).start()
        threading.Timer(move_time + 1.0, lambda: threading.Thread(target=self.joints_controller.position_neck, args=(40, )).start()).start()
        threading.Timer(move_time + 1.5, lambda: threading.Thread(target=self.joints_controller.position_neck, args=(15, )).start()).start()
        threading.Timer(move_time + 2.0, lambda: threading.Thread(target=self.joints_controller.position_neck, args=(40, )).start()).start()

    def head_bop(self,t,t0):
        print("head bop")
        freq = 2

        pitch = abs(self.new_sine_generator(0.26,-0.26,freq,0,t,t0))-0.2
        self.kinematic_joint_cmd.position = [0,0,0,pitch]


    ##Exit###

    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            try:
                self.remote_data = receive_data()
            except Exception as e:
                print(f"Failed to send data: {e}")
            if self.aruco_detect.exit_behaviour or self.remote_data[4]==2:
                self.stop_flag = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/i_will_stop.mp3',))
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
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
