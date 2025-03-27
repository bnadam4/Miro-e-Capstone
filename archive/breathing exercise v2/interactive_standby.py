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
import random

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
idle, setup, get_ready, are_you_ready, lets_go, breath_in, hold_1, breath_out, hold_2, silent_cycles, go_again, outro = range(12)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
# Generate enum and constants for the LEDS
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)

LIFT_MAX = math.degrees(miro.constants.LIFT_RAD_MAX)
LIFT_MIN = math.degrees(miro.constants.LIFT_RAD_MIN)

lift_upper = 15.0 # deg
lift_lower = 40.0 # deg

PITCH_MAX = math.degrees(miro.constants.PITCH_RAD_MAX)
PITCH_MIN = math.degrees(miro.constants.PITCH_RAD_MIN)
pitch_upper = -5.0 # deg
pitch_lower = -20.0 # deg

YAW_MAX = math.degrees(miro.constants.YAW_RAD_MAX)
YAW_MIN = math.degrees(miro.constants.YAW_RAD_MIN)

led_lower = 20
led_upper = 250

state_duration = 4.0
NUM_CYCLES = 3

# Audio file paths for different states
AUDIO_FILES = {
    "setting_up": 'setting_up.mp3',
    "powered_on": 'powered_on.mp3',
    "get_ready": 'get_ready.mp3',
    "are_you_ready": 'are_you_ready.mp3',
    "lets_go": 'lets_go.mp3',
    "breathe_in": 'breatheIn.mp3',
    "breathe_out": 'breatheOut.mp3',
    "hold_1": 'hold.mp3',
    "hold_2": 'holdAgain.mp3',
    "outro": 'one_more_time.mp3',
    "nice_job": 'nice_job.mp3',
    "go_again": 'go_again.mp3'
}

class breath_ex:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def callback_package(self, msg):
        self.input_package = msg

    def play_audio(self, track_file):
        decoded_file_path = os.path.join(os.path.dirname(__file__), '../Miro-e-Capstone/decoded_audio_files', f"{track_file}.decode")
        
        #print(f"Absolute decoded file path: {os.path.abspath(decoded_file_path)}")

        if not os.path.isfile(decoded_file_path):
            print(f"Error: No decoded file found at {decoded_file_path}.")
            print("Make sure to run .../decoded_audio_files/decode_audio.py before trying again.")
            print(f"The current directory is: {os.getcwd()}")
            return
        
        try:
            with open(decoded_file_path, 'rb') as f:
                dat = f.read()
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
        self.eyelid_speed = (1.0/(state_duration/self.TICK))*5 # Increase eyelid speed by 25%
        self.led_speed = ((led_upper - led_lower)/(state_duration/self.TICK))*1.75 # Increase LED speed by 25%

        self.last_time = time.time()
        self.current_time = time.time()
        self.eyelid_pos = 0.0
        self.led_brightness = led_lower
        self.tail_wag_pos = 0.0

        self.lift_sensor = 0
        self.yaw_sensor = 0
        self.pitch_sensor = 0

        self.lift_pos =  0
        self.yaw_pos = 0
        self.pitch_pos = 0

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
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
            miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)
        self.sub_log = rospy.Subscriber(topic_base_name + "/platform/log", String, self.callback_log, queue_size=5, tcp_nodelay=True)
        self.sub_stream = rospy.Subscriber(topic_base_name + "/sensors/stream", UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)


        # Set up variables for the kinetic joints
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(40.0), math.radians(0.0), math.radians(pitch_upper)]
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

        self.interp_lift_traj = []
        self.interp_yaw_traj = []
        self.interp_pitch_traj = []

        self.input_package = None

        self.go_again = False

        self.blink = False
        self.blink_start_time = 0

        self.wag_right = False

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

        self.state = idle
        self.last_state = None
        self.audio_finished = False  # Reset flag on loop start
        
        self.silent_cycle_count = 0

        # Initialize state timer
        state_start_time = time.time()

        self.play_audio(AUDIO_FILES["powered_on"])

        while not rospy.core.is_shutdown():

            # Detect aruco markers
            self.aruco_detect.tick_camera()
            # Detect touch
            self.touch_detect.check_touch()

            if not self.input_package is None:
                p = self.input_package
                self.input_package = None
                x = p.kinematic_joints.position

                self.lift_sensor = math.degrees(x[1])
                self.yaw_sensor = math.degrees(x[2])
                self.pitch_sensor = math.degrees(x[3])

                # print(f"Lift: {self.lift_sensor}, Yaw: {self.yaw_sensor}, Pitch: {self.pitch_sensor}")

            if self.aruco_detect.breath_ex_ON or self.touch_detect.breath_ex_ON:
                # Check if state duration has elapsed
                if time.time() > (state_start_time + state_duration) and self.audio_finished:
                    self.audio_finished = False
                    state_start_time = time.time()  # Reset state timer
                    # State transitions dictionary
                    state_transitions = {
                        setup: get_ready,
                        get_ready: are_you_ready,
                        are_you_ready: lets_go,
                        lets_go: breath_in,
                        breath_in: hold_1,
                        hold_1: breath_out,
                        breath_out: hold_2,
                        hold_2: breath_in if self.silent_cycle_count < NUM_CYCLES else go_again,
                        go_again: breath_in if self.go_again == True else outro, ###### need to code go_again, set default to false
                        outro: idle,
                        idle: setup
                    }
                    
                    # Transition to the next state
                    self.state = state_transitions.get(self.state, self.state)

                # Check if state has changed
                if self.state != self.last_state:
                    # Play the audio for the current state
                    if self.state == setup:
                        print("Setting up")
                        self.play_audio(AUDIO_FILES["setting_up"])

                        self.interp_lift_traj = interp_pos(self.lift_sensor, (LIFT_MAX + LIFT_MIN)/2, 100)
                        self.interp_yaw_traj = interp_pos(self.yaw_sensor, (YAW_MAX + YAW_MIN)/2, 100)
                        self.interp_pitch_traj = interp_pos(self.pitch_sensor, (PITCH_MAX + PITCH_MIN)/2, 100)

                        state_duration = 2.0

                    elif self.state == get_ready:
                        print("Get ready")
                        self.play_audio(AUDIO_FILES["get_ready"])
                        state_duration = 4
                    elif self.state == are_you_ready:
                        print("Are you ready")
                        self.play_audio(AUDIO_FILES["are_you_ready"])
                        state_duration = 4
                    elif self.state == lets_go:
                        print("Lets go")
                        self.play_audio(AUDIO_FILES["lets_go"])
                        state_duration = 8

                    elif self.state == breath_in:
                        print("Breath in")
                        self.play_audio(AUDIO_FILES["breathe_in"])
                        state_duration = 4

                        self.interp_lift_traj = interp_pos(self.lift_sensor, lift_upper, 100)
                        self.interp_yaw_traj = interp_pos(self.yaw_sensor, 0.0, 100)
                        self.interp_pitch_traj = interp_pos(self.pitch_sensor, pitch_lower, 100)


                    elif self.state == hold_1:
                        print("Hold 1")
                        self.play_audio(AUDIO_FILES["hold_1"])
                        state_duration = 4

                    elif self.state == breath_out:
                        print("Breath out")
                        self.play_audio(AUDIO_FILES["breathe_out"])
                        state_duration = 4

                        self.interp_lift_traj = interp_pos(self.lift_sensor, lift_lower, 100)
                        self.interp_yaw_traj = interp_pos(self.yaw_sensor, 0.0, 100)
                        self.interp_pitch_traj = interp_pos(self.pitch_sensor, pitch_upper, 100)

                    elif self.state == hold_2:
                        print("Hold 2")
                        self.play_audio(AUDIO_FILES["hold_2"])
                        state_duration = 4

                        self.silent_cycle_count += 1
                    elif self.state == go_again:
                        print("Go again")
                        self.play_audio(AUDIO_FILES["go_again"])
                        state_duration = 4

                    elif self.state == outro:
                        print("Outro")
                        self.play_audio(AUDIO_FILES["nice_job"])
                        state_duration = 4
                    elif self.state == idle:
                        self.touch_detect.breath_ex_reset = True
                        self.aruco_detect.breath_ex_reset = True
                        print("Idle")

                    self.last_state = self.state

                if self.state == setup:
                    x = time.time() - state_start_time
                    y = np.interp(x, [0, state_duration], [0, 100])
                    y = math.floor(y)

                    self.lift_pos = self.interp_lift_traj[y]
                    self.yaw_pos = self.interp_yaw_traj[y]
                    self.pitch_pos = self.interp_pitch_traj[y]

                # Perform movements based on the current state
                if self.state == breath_in:
                    # Make it look like MiRo is breathing in

                    x = time.time() - state_start_time
                    y = np.interp(x, [0, state_duration], [0, 100])
                    y = math.floor(y)

                    self.lift_pos = self.interp_lift_traj[y]
                    self.yaw_pos = self.interp_yaw_traj[y]
                    self.pitch_pos = self.interp_pitch_traj[y]

                    self.led_brightness = min(max(self.led_brightness + self.led_speed, led_lower), led_upper)                    
                    self.kin_joints.position = [0.0, math.radians(self.lift_pos), math.radians(self.yaw_pos), math.radians(self.pitch_pos)]

                    if self.eyelid_pos < 1.0:
                        self.eyelid_pos = self.eyelid_pos + self.eyelid_speed
                    else:
                        self.eyelid_pos = 1.0

                elif self.state == hold_1:
                    self.kin_joints.position = [0.0, math.radians(self.lift_pos), math.radians(self.yaw_pos), math.radians(self.pitch_pos)]

                elif self.state == breath_out:
                    # Make it look like MiRo is breathing out

                    x = time.time() - state_start_time
                    y = np.interp(x, [0, state_duration], [0, 100])
                    y = math.floor(y)

                    self.lift_pos = self.interp_lift_traj[y]
                    self.yaw_pos = self.interp_yaw_traj[y]
                    self.pitch_pos = self.interp_pitch_traj[y]

                    self.kin_joints.position = [0.0, math.radians(self.lift_pos), math.radians(self.yaw_pos), math.radians(self.pitch_pos)]
                    if self.eyelid_pos > 0.0:
                        self.eyelid_pos = self.eyelid_pos - self.eyelid_speed
                    else:
                        self.eyelid_pos = 0.0
                    self.led_brightness = min(max(self.led_brightness - self.led_speed, led_lower), led_upper)

                elif self.state == hold_2:
                    self.kin_joints.position = [0.0, math.radians(self.lift_pos), math.radians(self.yaw_pos), math.radians(self.pitch_pos)]

                self.cos_joints.data[left_eye] = self.eyelid_pos
                self.cos_joints.data[right_eye] = self.eyelid_pos
                
                self.pub_kin.publish(self.kin_joints)
                self.pub_cos.publish(self.cos_joints)

                self.illum.data[front_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[front_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                self.illum.data[mid_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[mid_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                self.illum.data[rear_left] = generate_illum(0, 0, 255, int(self.led_brightness))
                self.illum.data[rear_right] = generate_illum(0, 0, 255, int(self.led_brightness))

                
                self.pub_illum.publish(self.illum)

            else:
                # Detect faces. Only do so if the Breathing Exercise is not active

                ## random idle movements
                blink_chance = 0.01
                blink_duration = 0.25
                if random.random() < blink_chance:
                    self.blink = True
                    self.eyelid_pos = 0.75
                    self.blink_start_time = time.time()
                if self.blink and (time.time() - self.blink_start_time) > blink_duration:
                    self.eyelid_pos = 0.0
                    self.blink = False

                wag_chance = 0.05
                if random.random() < wag_chance:
                    if self.wag_right == False:
                        self.wag_right = True
                    else:
                        self.wag_right = False

                if self.wag_right:
                    self.tail_wag_pos = 0.75
                else:
                    self.tail_wag_pos = 0.25
                ## end random idle movements

                self.face_detect.check_face()
                self.kin_joints.position = [0.0, math.radians(self.lift_pos), self.face_detect.yaw, math.radians(self.pitch_pos)]
                self.pub_kin.publish(self.kin_joints)

                self.cos_joints.data[left_eye] = self.eyelid_pos
                self.cos_joints.data[right_eye] = self.eyelid_pos
                self.cos_joints.data[wag] = self.tail_wag_pos

                self.pub_kin.publish(self.kin_joints)
                self.pub_cos.publish(self.cos_joints)


            if self.aruco_detect.breath_ex_reset or self.touch_detect.breath_ex_reset:
                self.aruco_detect.breath_ex_reset = False
                self.touch_detect.breath_ex_reset = False
                self.aruco_detect.breath_ex_ON = False
                self.touch_detect.breath_ex_ON = False

                self.state = idle
                self.last_state = None
                self.audio_finished = False
                self.silent_cycle_count = 0
                # end audio stream
            
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

                    # check at those moments if we are making progress, also
                    if dropout_data_r == self.data_r:
                        if dropout_count == 0:
                            print ("dropping out because of no progress...")
                            #break
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

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = breath_ex()
	main.loop()
