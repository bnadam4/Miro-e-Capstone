#!/usr/bin/python3
#
# Author: Jasmine
# Date created: September 14, 2024
# Last modifiec: September 14, 2024
#
# Draft code for the Miro-e ears movement

#Change the speed start and stop

# Misc useful python libraries
import time
import math
import os
import numpy as np

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray
import miro2 as miro

# Generate enum and constants
outward, inward= range(2)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
state_duration = 2.0 #change the speed movement

class move_ears:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

        self.ears_speed = 1.0/(state_duration/self.TICK)
        self.state = outward
        self.last_time = time.time()
        self.current_time = time.time()
        self.ears_pos = 0.0

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("move_ears")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)


        # Set up variables for the cosmetic joints
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cos_joints.data[left_ear] = 0.0
        self.cos_joints.data[right_ear] = 0.0 


    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        while not rospy.core.is_shutdown():

            # Create breathing cycle state machine
            if time.time() > self.last_time + state_duration:
                self.last_time = time.time()
                if self.state == outward:
                    self.state = inward
                elif self.state == inward:
                    self.state = outward
                
	        #more ears outwords
            if self.state == outward:

                 if self.ears_pos < 1.0:
                    self.ears_pos = self.ears_pos + self.ears_speed
                 else:
                    self.ears_pos = 1.0
            #more ears inwords
            elif self.state == inward:

                 if self.ears_pos > 0.0:
                    self.ears_pos = self.ears_pos - self.ears_speed
                 else:
                    self.ears_pos = 0.0


            self.cos_joints.data[left_ear] = self.ears_pos
            self.cos_joints.data[right_ear] = self.ears_pos
            print("ears_pos = ", self.ears_pos)
            
            self.pub_cos.publish(self.cos_joints)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = move_ears()
	main.loop()
