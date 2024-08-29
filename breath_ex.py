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
import numpy as np

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import miro2 as miro

# Generate enum and constants for the breathing exercise states and joints
breath_in, hold_1, breath_out, hold_2 = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
NECK_MAX = miro.constants.LIFT_RAD_MAX
NECK_MIN = miro.constants.LIFT_RAD_MIN
neck_upper = 15.0 # deg
neck_lower = 40.0 # deg
state_duration = 4.0

class breath_ex:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

        self.neck_speed = (neck_lower - neck_upper)/(state_duration/self.TICK)
        self.eyelid_speed = 1.0/(state_duration/self.TICK)
        self.state = breath_in
        self.last_time = time.time()
        self.current_time = time.time()
        self.neck_pos = 40.0
        self.eyelid_pos = 0.0

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

        # Main control loop iteration counter
        self.counter = 0

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
            print("eyelid_pos = ", self.eyelid_pos)
             
            self.pub_kin.publish(self.kin_joints)
            self.pub_cos.publish(self.cos_joints)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = breath_ex()
	main.loop()