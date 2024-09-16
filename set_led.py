#!/usr/bin/python3
#
# Author: Kento Sasaki / Jimmy Wang
# Date created: September 15, 2024
# Last modifiec: September 15, 2024
#
# Will adjust the Miro's LEDs in accordance to the breathing exercise guidelines

# Misc useful python libraries

# to generate LED colour and brightness
def generate_illum(r, g, b, bright):
	return (int(bright) << 24) | (r << 16) | (g << 8) | b


import os
import numpy as np

# Robot specific libraries
import rospy
import miro2 as miro
from std_msgs.msg import UInt32MultiArray

# Generate enum and constants for the LEDS
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)


class set_leds:

    TICK = 0.02 # Update interval for the main controller loop in seconds

    def __init__(self):

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("set_leds")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Publishers
        self.pub_illum = rospy.Publisher(topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0)
        self.illum = UInt32MultiArray()
        self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
        self.led_brightness = 250

    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo sets LED colour and brightness")

        while not rospy.core.is_shutdown():

            self.illum.data[front_left] = generate_illum(255, 0, 0, int(self.led_brightness))
            self.illum.data[front_right] = generate_illum(255, 0, 0, int(self.led_brightness))

            self.illum.data[mid_left] = generate_illum(255, 0, 0, int(self.led_brightness))
            self.illum.data[mid_right] = generate_illum(255, 0, 0, int(self.led_brightness))

            self.illum.data[rear_left] = generate_illum(255, 0, 0, int(self.led_brightness))
            self.illum.data[rear_right] = generate_illum(255, 0, 0, int(self.led_brightness))

             
            self.pub_illum.publish(self.illum)
            print("led_brightness = ", self.led_brightness)

            # Yield
            rospy.sleep(self.TICK)


if __name__ == "__main__":
	main = set_leds()
	main.loop()
