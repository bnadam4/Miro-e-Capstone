#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 8, 2024
# Last modified: December 16, 2024
#
# Main default program for MiRo when it is not undergoing some pre-programmed activity

# Misc useful python libraries
import time
import math
import os
import sys
import numpy as np

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, Int16MultiArray, String, UInt16MultiArray
from sensor_msgs.msg import JointState
import miro2 as miro

# Sensor nodes to import
from node_detect_aruco import *
from detect_touch import *
from attend_face import *

# Custom python files with useful funstions
from pose_interp import *

# messages larger than this will be dropped by the receiver
MAX_STREAM_MSG_SIZE = (4096 - 48)

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

# to generate LED colour and brightness
def generate_illum(r, g, b, bright):
	return (int(bright) << 24) | (r << 16) | (g << 8) | b

def error(msg):
	print(msg)
	sys.exit(0)

# Generate enum and constants for the breathing exercise states and joints
intro, breath_in, hold_1, breath_out, hold_2, silent_cycles, outro = range(7)
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

state_duration = 4.0
NUM_CYCLES = 3

# Audio file paths for different states
AUDIO_FILES = {
    "intro": 'intro.mp3',
    "breathe_in": 'breatheIn.mp3',
    "breathe_out": 'breatheOut.mp3',
    "hold_1": 'hold.mp3',
    "hold_2": 'holdAgain.mp3',
    "outro": 'one_more_time.mp3',
    "nice_job": 'nice_job.mp3'
}

class breath_ex:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def play_audio(self, track_file):
        decoded_file_path = f"../../../../share/media/{track_file}.decode"
        
        if not os.path.isfile(decoded_file_path):
            print(f"Error: No decoded file found at {decoded_file_path}.")
            print("Make sure to run ../../../share/media/decode_audio.py before trying again.")
            print(f"The current directory is: {os.getcwd()}")
            return
        
        try:
            with open(decoded_file_path, 'rb') as f:
                dat = f.read()
                #print(f"Read {len(dat)} bytes from {decoded_file_path}")
        except Exception as e:
            print(f"Error while reading {decoded_file_path}: {e}")
            return
        
        self.data_r = 0
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)
        dat = dat.astype(np.float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()
        self.data = dat

        print(f"Playing {track_file}")
        # Reset buffer space for streaming
        self.buffer_space = 0
        self.buffer_total = 0

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

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("Capstone")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

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
        self.face_detect = AttendFace()

        # Make all the positions for MiRo's neck to move through
        step_num = state_duration/self.TICK
        self.neck_positions = interp_pos(neck_upper, neck_lower, step_num)
        print(f"neck positions are: {self.neck_positions}")
        self.step = 0


    def loop(self):
        """
        Main control loop
        """

        global state_duration

        # periodic reports
        count = 0

        # safety dropout if receiver not present
        dropout_data_r = -1
        dropout_count = 3

        # Main control loop iteration counter
        self.counter = 0

        self.state = intro
        self.last_state = None
        self.audio_finished = False  # Reset flag on loop start
        
        self.silent_cycle_count = 0

        self.play_audio(AUDIO_FILES["intro"])

        print("MiRo does some deep breathing")

        # Initialize state timer
        state_start_time = time.time()

        while not rospy.core.is_shutdown():

            # Detect aruco markers
            self.aruco_detect.tick_camera()
            # Detect touch
            self.touch_detect.check_touch()

            if self.aruco_detect.breath_ex_ON or self.touch_detect.breath_ex_ON:
                # Check if state duration has elapsed
                if time.time() > (state_start_time + state_duration) and self.audio_finished:
                    self.audio_finished = False
                    state_start_time = time.time()  # Reset state timer
                    # State transitions dictionary
                    state_transitions = {
                        intro: breath_in,
                        breath_in: hold_1,
                        hold_1: breath_out,
                        breath_out: hold_2,
                        hold_2: breath_in if self.silent_cycle_count < NUM_CYCLES else outro
                    }
                        
                    # Transition to the next state
                    self.state = state_transitions.get(self.state, self.state)

                    # Add delay between intro and breathe_in
                    if self.state == breath_in and self.last_state == intro:
                        rospy.sleep(2.0)

                # Check if state has changed
                if self.state != self.last_state:
                    # Play the audio for the current state
                    if self.state == intro:
                        self.play_audio(AUDIO_FILES["intro"])
                    elif self.state == breath_in:
                        self.play_audio(AUDIO_FILES["breathe_in"])
                        state_start_time = time.time()  # Reset state timer
                    elif self.state == hold_1:
                        self.play_audio(AUDIO_FILES["hold_1"])
                    elif self.state == breath_out:
                        self.play_audio(AUDIO_FILES["breathe_out"])
                    elif self.state == hold_2:
                        self.play_audio(AUDIO_FILES["hold_2"])
                        self.silent_cycle_count += 1
                    elif self.state == outro:
                        self.play_audio(AUDIO_FILES["nice_job"])
                    self.last_state = self.state

                # Perform movements based on the current state
                if self.state == breath_in:
                    # Make it look like MiRo is breathing in
                    print(f"self.step = {self.step}")
                    self.neck_pos = self.neck_positions[len(self.neck_positions) - 1 - self.step]
                    print(f"self.neck_pos = {self.neck_pos}")
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
                    print(f"self.step = {self.step}")
                    self.neck_pos = self.neck_positions[self.step]
                    print(f"self.neck_pos = {self.neck_pos}")
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

                # print(self.led_brightness)

                # audio stream

                # if we've received a report
                if self.buffer_total > 0:

                    # compute amount to send
                    buffer_rem = self.buffer_total - self.buffer_space
                    n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                    n_bytes = max(n_bytes, 0)
                    n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

                    # if amount to send is non-zero
                    if n_bytes > 0:

                        msg = Int16MultiArray(data = self.data[self.data_r:self.data_r+n_bytes])
                        self.pub_stream.publish(msg)
                        self.data_r += n_bytes

                # break
                if self.data_r <= len(self.data):
                    # report once per second
                    if count == 0:
                        count = 10
                        #print ("streaming:", self.data_r, "/", len(self.data), "bytes")

                        # check at those moments if we are making progress, also
                        if dropout_data_r == self.data_r:
                            if dropout_count == 0:
                                print ("dropping out because of no progress...")
                                break
                            print ("dropping out in", str(dropout_count) + "...")
                            dropout_count -= 1
                        else:
                            dropout_data_r = self.data_r

                    # count tenths
                    count -= 1
                else:
                    # Print only once when audio playback finishes
                    if not self.audio_finished:
                        print("audio playback finished")
                        self.audio_finished = True  # Set flag to true
                        if self.silent_cycle_count > NUM_CYCLES:
                            self.touch_detect.breath_ex_reset = True
                            self.aruco_detect.breath_ex_reset = True
            else:
                # Detect faces. Only do so if the Breathing Exercise is not active
                self.face_detect.check_face()
                self.kin_joints.position = [0.0, math.radians(self.neck_pos), self.face_detect.yaw, math.radians(self.pitch_pos)]
                self.pub_kin.publish(self.kin_joints)


            if self.aruco_detect.breath_ex_reset or self.touch_detect.breath_ex_reset:
                self.aruco_detect.breath_ex_reset = False
                self.touch_detect.breath_ex_reset = False
                self.aruco_detect.breath_ex_ON = False
                self.touch_detect.breath_ex_ON = False

                self.state = intro
                self.last_state = None
                self.audio_finished = False
                self.silent_cycle_count = 0
                # end audio stream
            
            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = breath_ex()
	main.loop()
