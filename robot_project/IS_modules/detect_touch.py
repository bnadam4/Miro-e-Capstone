#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 14th, 2024
# Last modifiec: January 28th, 2025
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
        
        self.head_touched = False
        self.time_touch = time.time() # Used to determine time between touches
        print("MiRo detects whether it has been touched")

        # Initialize a ROS node to communicate with MiRo
        # rospy.init_node("node_aruco_detector")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)


    def check_touch(self):

        if not self.input_package is None:
            #aquire
            p = self.input_package
            self.input_package = None

            # Check touch data
            if int(p.touch_body.data) > 0:
                print("Body touched!")
                pass
            if int(p.touch_head.data) > 0 and self.head_touched == False:
                print("Head touched!")
                self.time_touch = time.time()
                self.head_touched = True

            if int(p.touch_head.data) > 0 and self.head_touched == True:
                print("Head touched")
                if time.time() > self.time_touch + 5.0:
                    self.head_touched = False
                    print("Timeout touch")
                
                if time.time() > self.time_touch + 0.5:
                    self.head_touched = False
                 


    # Callback for touch sensor data
    def callback_package(self, msg):
         # store data from sensor package for processing
         self.input_package = msg