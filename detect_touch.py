#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 14th, 2024
# Last modifiec: October 14th, 2024
#
# Extracts touch data from MiRo and adjusts MiRo behaviour accordingly

# Misc useful python libraries
import time
import os

# Robot specific libraries
import rospy
import miro2 as miro

class see_touch:


    def __init__(self):

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Subscribers
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
					miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)
        
        self.breath_ex_ON = False
        self.breath_ex_reset = False
        self.head_touched = False
        self.time_touch = time.time() # Used to determine time between touches
        print("MiRo detects whether it has been touched")


    def check_touch(self):

        if not self.input_package is None:
            #aquire
            p = self.input_package
            self.input_package = None

            # Check touch data
            if int(p.touch_body.data) > 0:
                print("Body touched!")
            if int(p.touch_head.data) > 0:
                print("Head touched!")
                self.time_touch = time.time()
                self.touched = True

            # To activate tap the miro's head twice. 
            # The second tap should come within 
                 


    # Callback for touch sensor data
    def callback_package(self, msg):
         # store data from sensor package for processing
         self.input_package = msg