#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: September 17, 2024
# Last modifiec: September 17, 2024
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

class see_touch:

    TICK = 1.0 # Update interval for the main controller loop in seconds

    def __init__(self):

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Subscribers
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
					miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("touch_detector")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Subscribers
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
					miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)


    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo detects whether it has been touched")

        while not rospy.core.is_shutdown():

            if not self.input_package is None:
                 #aquire
                 p = self.input_package
                 self.input_package = None

                 # the sensor package includes: battery, sonar, light, and touch sensors
                 print("Update battery: ", f"{p.battery.voltage:.2f}", " V")
                 print("Raw data for body touch is: ", bin(p.touch_body.data))
                 print("Ram data for head touch is: ", bin(p.touch_head.data))

            # Yield
            rospy.sleep(self.TICK)

    # Callback for touch sensor data
    def callback_package(self, msg):
         # store data from sensor package for processing
         self.input_package = msg


if __name__ == "__main__":
	main = see_touch()
	main.loop()