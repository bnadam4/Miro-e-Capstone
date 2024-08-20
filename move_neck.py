#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: August 18, 2024
# Last modifiec: August 18, 2024
#
# Will move the miro's neck in accordance to the breathing exercise guidlines

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
breath_in, hold_1, breath_out, hold_2 = range(4)
NECK_MAX = miro.constants.LIFT_RAD_MAX
NECK_MIN = miro.constants.LIFT_RAD_MIN
neck_upper = 15.0 # deg
neck_lower = 40.0 # deg
state_duration = 4.0

class move_neck:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

        self.neck_speed = (neck_lower - neck_upper)/(state_duration/self.TICK)
        self.state = breath_in
        self.last_time = time.time()
        self.current_time = time.time()
        self.neck_pos = 40.0

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("move_neck")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)

        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(40.0), 0.0, 0.0]
        self.pub_kin.publish(self.kin_joints) # Set neck to initial position


    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo moves its neck")

        while not rospy.core.is_shutdown():

            # Create breathing cycle state machine
            if time.time() > self.last_time + 4.0:
                print("Entered state machine")
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
                 # Gradually raise the neck to upper position
                 self.neck_pos = self.neck_pos - self.neck_speed
                 self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, 0.0]
            elif self.state == hold_1:
                 # Hold the neck at the upper position
                 self.neck_pos = neck_upper
                 self.kin_joints.position = [0.0, math.radians(neck_upper), 0.0, 0.0]
            elif self.state == breath_out:
                 # Gradually lower the neck to lower position
                 self.neck_pos = self.neck_pos + self.neck_speed
                 self.kin_joints.position = [0.0, math.radians(self.neck_pos), 0.0, 0.0]
            elif self.state == hold_2:
                 # Hold the neck at the lower position
                 self.neck_pos = neck_lower
                 self.kin_joints.position = [0.0, math.radians(neck_lower), 0.0, 0.0]
            print("neck_pos = ", self.neck_pos)
             
            self.pub_kin.publish(self.kin_joints)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = move_neck()
	main.loop()

