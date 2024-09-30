#!/usr/bin/python3
#
# Author: Jasmine
# Date created: September 29, 2024
# Last modifiec: September 29, 2024
#
# Will move the miro's head
#
# Head has 2 varaibles to control: yaw: right (-55) and left (55), pitch: up (-22) and down (8)
# Note: I commented out the pitch because it look weird for the head to move diagonally 

# Misc useful python libraries
import time
import math
import os
import numpy as np

# Robot specific libraries
import rospy
from sensor_msgs.msg import JointState
import miro2 as miro

# Generate enum and constants for the neck states
move_1, move_2 = range(2)
neck_right = -55 # deg
neck_left = 55 # deg

neck_up = -22 # deg
neck_down = 8 # deg

state_duration = 4.0


class move_head:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

        self.neck_yaw_speed = (neck_left - neck_right)/(state_duration/self.TICK)
        self.neck_pitch_speed = (neck_down - neck_up)/(state_duration/self.TICK)
        self.state = move_1
        self.last_time = time.time()
        self.current_time = time.time()
        self.neck_yaw_pos = 0
        self.neck_picth_pos = -7

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("move_head")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)

        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, 0.0, math.radians(0), math.radians(-7)]
        self.pub_kin.publish(self.kin_joints) # Set neck to initial position


    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo moves its head")

        while not rospy.core.is_shutdown():

            # Create cycle state machine
            if time.time() > self.last_time + state_duration:
                print("Entered state machine")
                self.last_time = time.time()
                if self.state == move_1:
                    self.state = move_2
                elif self.state == move_2:
                    self.state = move_1
                
            #### yaw ######
            #move head to left
            if self.state == move_1:
                 self.neck_yaw_pos = self.neck_yaw_pos - self.neck_yaw_speed
                 self.kin_joints.position = [0.0, 0.0, math.radians(self.neck_yaw_pos),0.0]
            #move head to right
            elif self.state == move_2:
                 self.neck_yaw_pos = self.neck_yaw_pos + self.neck_yaw_speed
                 self.kin_joints.position = [0.0,  0.0, math.radians(self.neck_yaw_pos), 0.0]
            print("neck_yaw_pos = ", self.neck_yaw_pos)

            # ##### pitch ######
            # #move head to left
            # if self.state == move_1:
            #      self.neck_picth_pos = self.neck_picth_pos - self.neck_pitch_speed
            #      self.kin_joints.position = [0.0, 0.0, 0.0,math.radians(self.neck_picth_pos)]
            # #move head to right
            # elif self.state == move_2:
            #      self.neck_picth_pos = self.neck_picth_pos + self.neck_pitch_speed
            #      self.kin_joints.position = [0.0,  0.0, 0.0, math.radians(self.neck_picth_pos)]
            
            print("neck_picth_pos = ", self.neck_picth_pos)
            self.pub_kin.publish(self.kin_joints)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = move_head()
	main.loop()

