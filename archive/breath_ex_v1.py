#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: August 29, 2024
# Last modifiec: August 29, 2024
#
# Draft code for the Miro-e breathing exercise

# Misc useful python libraries
import time
import math
import os
import sys
import numpy as np

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String, UInt16MultiArray
from sensor_msgs.msg import JointState
import miro2 as miro

# messages larger than this will be dropped by the receiver
MAX_STREAM_MSG_SIZE = (4096 - 48)

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

def error(msg):
	print(msg)
	sys.exit(0)

# Generate enum and constants for the breathing exercise states and joints
breath_in, hold_1, breath_out, hold_2 = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
NECK_MAX = miro.constants.LIFT_RAD_MAX
NECK_MIN = miro.constants.LIFT_RAD_MIN
neck_upper = 15.0 # deg
neck_lower = 40.0 # deg
state_duration = 4.0

# Audio file paths for different states
AUDIO_FILES = {
    "breathe_in": 'alice_breathe_in.mp3',
    "breathe_out": 'alice_breathe_out.mp3',
    "hold_1": 'alice_hold.mp3',
    "hold_2": 'alice_hold.mp3'
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
        decoded_file_path = f"../../../share/media/{track_file}.decode"
        
        if not os.path.isfile(decoded_file_path):
            print(f"Error: No decoded file found at {decoded_file_path}.")
            print("Make sure to run ../../../share/media/decode_audio.py before trying again.")
            return
        
        try:
            with open(decoded_file_path, 'rb') as f:
                dat = f.read()
                print(f"Read {len(dat)} bytes from {decoded_file_path}")
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
        self.neck_speed = (neck_lower - neck_upper)/(state_duration/self.TICK)
        self.eyelid_speed = 1.0/(state_duration/self.TICK)
        self.state = breath_in
        self.last_time = time.time()
        self.current_time = time.time()
        self.neck_pos = 40.0
        self.eyelid_pos = 0.0

        self.audio_finished = False

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("move_neck")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)
        self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
        self.pub_stream = rospy.Publisher(topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        #subscribe
        self.sub_log = rospy.Subscriber(topic_base_name + "/platform/log", String, self.callback_log, queue_size=5, tcp_nodelay=True)
        self.sub_stream = rospy.Subscriber(topic_base_name + "/sensors/stream", UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)


        # Set up variables for the kinetic joints
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(40.0), 0.0, 0.0]
        self.pub_kin.publish(self.kin_joints) # Set neck to initial position

        # Set up variables for the cosmetic joints
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cos_joints.data[left_eye] = 0.0
        self.cos_joints.data[right_eye] = 0.0


    def loop(self):
        """
        Main control loop
        """

        # periodic reports
        count = 0

        # safety dropout if receiver not present
        dropout_data_r = -1
        dropout_count = 3

        # Main control loop iteration counter
        self.counter = 0
        self.last_state = None
        self.audio_finished = False  # Reset flag on loop start

        print("MiRo does some deep breathing")

        while not rospy.core.is_shutdown():

            # Create breathing cycle state machine
            if time.time() > self.last_time + 4.0:
                self.last_time = time.time()
                if self.state == breath_in:
                    self.state = hold_1
                elif self.state == hold_1:
                    self.state = breath_out
                elif self.state == breath_out:
                    self.state = hold_2
                elif self.state == hold_2:
                    self.state = breath_in

            # Check if state has changed
            if self.state != self.last_state:
                # Play the appropriate audio for the current state
                if self.state == breath_in:
                    self.play_audio(AUDIO_FILES["breathe_in"])
                elif self.state == hold_1:
                    self.play_audio(AUDIO_FILES["hold_1"])
                elif self.state == breath_out:
                    self.play_audio(AUDIO_FILES["breathe_out"])
                elif self.state == hold_2:
                    self.play_audio(AUDIO_FILES["hold_2"])

                # Update last_state to the current state
                self.last_state = self.state
                self.audio_finished = False  # Reset flag when changing state

            if self.state == breath_in:
                 # Make it look like MiRo is breathing in
                 self.neck_pos = self.neck_pos - self.neck_speed
                 self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, 0.0]
                 if self.eyelid_pos < 1.0:
                    self.eyelid_pos = self.eyelid_pos + self.eyelid_speed
                 else:
                    self.eyelid_pos = 1.0
            elif self.state == hold_1:
                 # Hold the fully inhaled position
                 self.neck_pos = neck_upper
                 self.kin_joints.position = [0.0, math.radians(neck_upper), 0.0, 0.0]
            elif self.state == breath_out:
                 # Make it look like MiRo is breathing out
                 self.neck_pos = self.neck_pos + self.neck_speed
                 self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, 0.0]
                 if self.eyelid_pos > 0.0:
                    self.eyelid_pos = self.eyelid_pos - self.eyelid_speed
                 else:
                    self.eyelid_pos = 0.0
            elif self.state == hold_2:
                 # Hold the fully exhaled position
                 self.neck_pos = neck_lower
                 self.kin_joints.position = [0.0, math.radians(neck_lower), 0.0, 0.0]

            self.cos_joints.data[left_eye] = self.eyelid_pos
            self.cos_joints.data[right_eye] = self.eyelid_pos
            # print("eyelid_pos = ", self.eyelid_pos)
             
            self.pub_kin.publish(self.kin_joints)
            self.pub_cos.publish(self.cos_joints)


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
                    print ("streaming:", self.data_r, "/", len(self.data), "bytes")

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


            # end audio stream
            

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = breath_ex()
	main.loop()
