#!/usr/bin/python3

# ----------------------------------------------
# Title: breath_ex.py
# Description: Do the pre-programmed breathing exercise behaviour
# Author: Bryce
# Date created: Jan 8, 2024
# Date modified: Jan 28, 2024
# ----------------------------------------------

# Misc useful python libraries
import time
import math
import os
import sys
import numpy as np
import threading

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, Int16MultiArray, String, UInt16MultiArray
from sensor_msgs.msg import JointState
import miro2 as miro

# Sensor nodes to import
from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *

# Actuactor controllers
from actuators.cosmetics_controller import CosmeticsController
from actuators.cosmetics_movement import CosmeticsMovement
from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

# Custom python files with useful functions
from IS_modules.pose_interp import *
from actuators.play_audio import play_audio  # function

# to generate LED colour and brightness
def generate_illum(r, g, b, bright):
	return (int(bright) << 24) | (r << 16) | (g << 8) | b

def error(msg):
	print(msg)
	sys.exit(0)

# Generate enum and constants for the breathing exercise states and joints
intro, ready, breath_in, hold_1, breath_out, hold_2, silent_cycles, outro = range(8)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
# Generate enum and constants for the LEDS
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)


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

state_duration = 4.0 # Should be 4.0
NUM_CYCLES = 3

BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

class breath_ex:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def __init__(self):
        self.pitch_speed = ((pitch_lower - pitch_upper)/(state_duration/self.TICK))*1.75# Increase neck speed by 25%
        self.eyelid_speed = (1.0/(state_duration/self.TICK))*1.75 # Increase eyelid speed by 25%
        self.led_speed = ((led_upper - led_lower)/(state_duration/self.TICK))*1.75 # Increase LED speed by 25%

        self.last_time = time.time()
        self.current_time = time.time()
        self.neck_pos = 40.0
        self.pitch_pos = pitch_upper
        self.eyelid_pos = 0.0
        self.led_brightness = led_lower

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Publishers
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)
        self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
        self.pub_stream = rospy.Publisher(topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        # Publishers
        self.pub_illum = rospy.Publisher(topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0)
        self.illum = UInt32MultiArray()
        self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

        #subscribe
        self.sub_log = rospy.Subscriber(topic_base_name + "/platform/log", String, self.callback_log, queue_size=5, tcp_nodelay=True)
        self.sub_stream = rospy.Subscriber(topic_base_name + "/sensors/stream", UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

        # Set up variables for the kinetic joints
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(40.0), 0.0, math.radians(pitch_upper)]
        self.pub_kin.publish(self.kin_joints) # Set neck to initial position

        # Set up variables for the cosmetic joints
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cos_joints.data[left_eye] = 0.0
        self.cos_joints.data[right_eye] = 0.0

        # Make detector nodes
        self.aruco_detect = NodeDetectAruco()
        self.touch_detect = see_touch()
        self.track_face = False

        # Make all the positions for MiRo's neck to move through
        step_num = state_duration/self.TICK
        self.neck_positions = interp_pos(neck_upper, neck_lower, step_num)
        #print(f"neck positions are: {self.neck_positions}")
        self.step = 0

        # Make behaviour tracking variable
        self.behaviour = BREATHING_EXERCISE

        # Initialize controllers
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()

        # rospy.sleep(2.0)


    def run(self):
        """
        Main control loop
        """

        global state_duration

        # Main control loop iteration counter
        self.counter = 0

        self.state = intro
        self.last_state = None
        
        self.silent_cycle_count = 0

        # Play the intro audio
        BE_intro_thread = threading.Thread(target=play_audio, args=('mp3_files/Breath_ex.mp3',))
        BE_intro_thread.start()
        head_thread = threading.Thread(target=self.joints_movement.nod, args=(2, 2, ))
        head_thread.start()
        BE_intro_thread.join() # Wait for intro to finish
        rospy.sleep(1.0)
        intro_thread = threading.Thread(target=play_audio, args=('mp3_files/intro.mp3',))
        intro_thread.start()

        print("MiRo does some deep breathing")
        self.aruco_detect.breath_ex_ON = True

        # Initialize state timers
        state_start_time = time.time()

        while not rospy.core.is_shutdown():

            self.behaviour = BREATHING_EXERCISE

            # Detect aruco markers
            self.aruco_detect.tick_camera()
            # Detect touch
            self.touch_detect.check_touch()

            if self.touch_detect.head_touched:
                print("Head touched!")

            if self.aruco_detect.exit_behaviour == True:
                print("Exiting breathing exercise")
                break

            if self.aruco_detect.breath_ex_ON:
                # Check if state duration has elapsed
                if time.time() > (state_start_time + state_duration):
                    state_start_time = time.time()  # Reset state timer
                    # State transitions dictionary
                    state_transitions = {
                        intro: ready,
                        ready: breath_in,
                        breath_in: hold_1,
                        hold_1: breath_out,
                        breath_out: hold_2,
                        hold_2: breath_in if self.silent_cycle_count < NUM_CYCLES else outro
                    }
                        
                    # Transition to the next state
                    self.state = state_transitions.get(self.state, self.state)

                    if self.state == breath_in:
                        print("Switched to breath in")
                    elif self.state == hold_1:
                        print("Switch to hold 1")
                    elif self.state == breath_out:
                        print("Switch to breath out")
                    elif self.state == hold_2:
                        print("Switch to hold 2")
                    elif self.state == outro:
                        print("outro")

                    # Add delay between intro and ready prompt
                    if self.state == ready and self.last_state == intro:
                        rospy.sleep(3.0)
                        # Play the ready prompt and wait for touch
                        ready_thread = threading.Thread(target=play_audio, args=('mp3_files/BrEx_Ready_Prompt.mp3',))
                        ready_thread.start()

                        state_start_time = time.time()  # Reset state timer

                        # Wait for touch
                        while not self.touch_detect.head_touched:
                            self.touch_detect.check_touch()
                            rospy.sleep(self.TICK)

                        rospy.sleep(0.5)
                        init_intro_thread = threading.Thread(target=play_audio, args=('mp3_files/init_intro.mp3',))
                        init_intro_thread.start()
                        init_intro_thread.join() # Wait for init_intro to finish
                        rospy.sleep(1.0)
                        self.touch_detect.head_touched = False # Reset head touch


                # Check if state has changed
                if self.state != self.last_state:
                    self.last_state = self.state
                    if self.state == breath_in:
                        breath_in_thread = threading.Thread(target=play_audio, args=('mp3_files/breatheIn.mp3',))
                        self.silent_cycle_count += 1
                        breath_in_thread.start()
                    elif self.state == hold_1:
                        hold_1_thread = threading.Thread(target=play_audio, args=('mp3_files/hold.mp3',))
                        hold_1_thread.start()
                    elif self.state == breath_out:
                        breath_out_thread = threading.Thread(target=play_audio, args=('mp3_files/breatheOut.mp3',))
                        breath_out_thread.start()
                    elif self.state == hold_2:
                        hold_2_thread = threading.Thread(target=play_audio, args=('mp3_files/holdAgain.mp3',))
                        hold_2_thread.start()
                    elif self.state == outro:
                        outro_thread = threading.Thread(target=play_audio, args=('mp3_files/nice_job.mp3',))
                        outro_thread.start()
                        rospy.sleep(4.0)

                        outro_thread = threading.Thread(target=play_audio, args=('mp3_files/BrEx_Go_again.mp3',))
                        outro_thread.start()

                        state_start_time = time.time()  # Reset state timer

                        # Wait for touch
                        self.touch_detect.check_touch()
                        while not self.touch_detect.head_touched and (time.time() - state_start_time) < 10.0:
                            self.touch_detect.check_touch()
                            print("Waiting for touch")
                            rospy.sleep(self.TICK)

                        if self.touch_detect.head_touched:
                            self.silent_cycle_count = 0
                            self.state = breath_in
                            self.step = 0
                            self.last_state = breath_in # Make sure the state stays consistent
                            print("Head touched!")
                            breath_in_thread = threading.Thread(target=play_audio, args=('mp3_files/breatheIn.mp3',))
                            breath_in_thread.start()
                            self.silent_cycle_count += 1
                        else:
                            print("Timed out")
                            self.behaviour = INTERACTIVE_STANDBY
                            breath_out_thread= threading.Thread(target=play_audio, args=('mp3_files/BrEx_Timeout.mp3',))
                            breath_out_thread.start()
                            breath_out_thread.join()
                            break

                        state_start_time = time.time()  # Reset state timer
                        self.touch_detect.head_touched = False # Reset head touch

                # Perform movements based on the current state
                if self.state == breath_in:
                    # Make it look like MiRo is breathing in
                    self.neck_pos = self.neck_positions[len(self.neck_positions) - 1 - self.step]
                    self.pitch_pos = self.pitch_pos + self.pitch_speed  # Swapped
                    self.led_brightness = min(max(self.led_brightness + self.led_speed, led_lower), led_upper)                    
                    self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, math.radians(self.pitch_pos)]
                    if self.eyelid_pos < 1.0:
                        self.eyelid_pos = self.eyelid_pos + self.eyelid_speed
                    else:
                        self.eyelid_pos = 1.0

                    self.step += 1

                elif self.state == hold_1:
                    # Hold the fully inhaled position
                    # self.neck_pos = neck_upper
                    # self.pitch_pos = pitch_lower
                    # self.eyelid_pos = 1.0
                    self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, math.radians(self.pitch_pos)]
                    self.step = 0

                elif self.state == breath_out:
                    # Make it look like MiRo is breathing out
                    self.neck_pos = self.neck_positions[self.step]
                    self.pitch_pos = self.pitch_pos - self.pitch_speed  # Swapped
                    self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, math.radians(self.pitch_pos)]
                    if self.eyelid_pos > 0.0:
                        self.eyelid_pos = self.eyelid_pos - self.eyelid_speed
                    else:
                        self.eyelid_pos = 0.0
                    self.led_brightness = min(max(self.led_brightness - self.led_speed, led_lower), led_upper)

                    self.step += 1

                elif self.state == hold_2:
                    # Hold the fully exhaled position
                    # self.neck_pos = neck_lower
                    # self.pitch_pos = pitch_upper
                    # self.eyelid_pos = 0.0
                    # self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, math.radians(self.pitch_pos)]
                    self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, math.radians(self.pitch_pos)]
                    self.step = 0

                self.cos_joints.data[left_eye] = self.eyelid_pos
                self.cos_joints.data[right_eye] = self.eyelid_pos
                # print("eyelid_pos = ", self.eyelid_pos)
                
                self.pub_kin.publish(self.kin_joints)
                self.pub_cos.publish(self.cos_joints)

                self.illum.data[front_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[front_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                self.illum.data[mid_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[mid_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                self.illum.data[rear_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[rear_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                self.pub_illum.publish(self.illum)


            if self.aruco_detect.breath_ex_reset:
                self.aruco_detect.breath_ex_reset = False
                self.aruco_detect.breath_ex_ON = False
                self.behaviour = INTERACTIVE_STANDBY

                self.state = intro
                self.last_state = None
                self.silent_cycle_count = 0

                self.eyelid_pos = 0.0
                self.cos_joints.data[left_eye] = self.eyelid_pos
                self.cos_joints.data[right_eye] = self.eyelid_pos
                self.pub_cos.publish(self.cos_joints)

                exit_thread= threading.Thread(target=play_audio, args=('mp3_files/BrEx_Timeout.mp3',))
                exit_thread.start()
                exit_thread.join()
                # end audio stream
            
            # Yield
            rospy.sleep(self.TICK)

            if self.behaviour == INTERACTIVE_STANDBY:
                print("Hit the break")
                break