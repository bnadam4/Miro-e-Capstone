#!/usr/bin/python3
#
# Author: Jasmine
# Date created: August 29, 2024
# Last modifiec: August 29, 2024
#
# Draft code for the Miro-e tail
#
# Tails has 2 varaibles to control: wag= right and left, droop: up and down
# Note: I commented out the droop because it look weird for the tail to move diagonally 

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

# Generate enum and constants
move_1, move_2 = range(2)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
state_duration = 1.0 #adjust the speed

class move_tail:

    TICK = 0.01 # Update interval for the main controller loop in seconds

    def __init__(self):

        self.tail_speed = 1.0/(state_duration/self.TICK)
        self.state = move_1
        self.last_time = time.time()
        self.current_time = time.time()
        self.tail_wag_pos = 0.0
        self.tail_droop_pos = 0.0

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("move_tail")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)

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

        while not rospy.core.is_shutdown():

            # Create cycle state machine
            if time.time() > self.last_time + state_duration:
                self.last_time = time.time()
                if self.state == move_1:
                    self.state = move_2
                elif self.state == move_2:
                    self.state = move_1
            ##### wag ######
            #move tail left
            if self.state == move_1:
                 if self.tail_wag_pos < 1.0:
                    self.tail_wag_pos = self.tail_wag_pos + self.tail_speed
                 else:
                    self.tail_wag_pos = 1.0
            #move tail left
            elif self.state == move_2:
                 if self.tail_wag_pos > 0.0:
                    self.tail_wag_pos = self.tail_wag_pos - self.tail_speed
                 else:
                    self.tail_wag_pos = 0.0

            self.cos_joints.data[wag] = self.tail_wag_pos
            print("tail_wag_pos = ", self.tail_wag_pos)


            # ##### droop ######
            # #move tail up
            # if self.state == move_1:
            #      if self.tail_droop_pos < 1.0:
            #         self.tail_droop_pos = self.tail_droop_pos + self.tail_speed
            #      else:
            #         self.tail_droop_pos = 1.0
            # #move tail down
            # elif self.state == move_2:
            #      if self.tail_droop_pos > 0.0:
            #         self.tail_droop_pos = self.tail_droop_pos - self.tail_speed
            #      else:
            #         self.tail_droop_pos = 0.0
            # self.cos_joints.data[droop] = self.tail_droop_pos
            # print("tail_wag_pos = ", self.tail_droop_pos)

            
            self.pub_cos.publish(self.cos_joints)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = move_tail()
	main.loop()