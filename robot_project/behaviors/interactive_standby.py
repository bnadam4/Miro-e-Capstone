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
from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *
from IS_modules.attend_face import *

# Custom python files with useful funstions
# from pose_interp import *

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

BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

class interactive_standby:

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

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
        self.face_detect = AttendFace()
        self.track_face = True

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

            # Detect faces. Only do so if the Breathing Exercise is not active
            if self.track_face:
                self.face_detect.check_face()
                self.kin_joints.position = [0.0, math.radians(self.neck_pos), self.face_detect.yaw, math.radians(self.pitch_pos)]
                self.pub_kin.publish(self.kin_joints)
            
            # Yield
            rospy.sleep(self.TICK)

        print("Exited the loop")
