#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: September 21, 2024
# Last modifiec: September 21, 2024
#
# Displays MiRo camera feed and aruco detection

# Misc useful python libraries
import time
import sys
import os
import numpy as np

# Robot specific libraries
import rospy
from sensor_msgs.msg import CompressedImage
import miro2 as miro

import cv2
from cv_bridge import CvBridge, CvBridgeError

class camera_test:

    def __init__(self):

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Variables to store input data
        self.input_camera = [None, None]
        self.t_input_camera = [[], []]

        #Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()

        # Image stitcher
        self.image_stitcher = None

        # Subscribers to left camera (caml) and right camera (camr)
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed", 
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed", 
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("camera_node")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)


    def loop(self):
        """
        Main control loop
        """

        # state
        channels_to_process = [0, 1]
        if not self.image_stitcher is None:
            channels_to_process = [2]
        outfile = [None, None, None]
        outcount = [0] * len(outfile)
        t0 = time.time()
        cam_names = ['left', 'right', 'stitched']

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo camera feed")

        while not rospy.core.is_shutdown():

            ## stitching ##

            # if stitching
            if not self.image_stitcher is None:

                # stitch
                if not self.input_camera[0] is None and not self.input_camera[1] is None:
                    images = [self.input_camera[0], self.input_camera[1]]
                    self.input_camera[2] = cv2.hconcat(images)
                    self.input_camera[0] = None
                    self.input_camera[1] = None

            # for each channel to process
            for index in channels_to_process:

                # get image
                image = self.input_camera[index]

                # if present
                if not image is None:

                    # handle
                    self.input_camera[index] = None

                    # show
                    cv2.imshow("Camera Feed: " + cam_names[index], image)
                    cv2.waitKey(1)

            # state
            time.sleep(0.02)


    def callback_cam(self, ros_image, index):
        
        # silently (ish) handle corrupted JPEG frames
        try:
             
            # convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")

            # store image for display
            self.input_camera[index] = image

        except CvBridgeError as e:
             # swallow error, silently
             #print(e)
             pass
        
         
    # Callback for left camera feed
    def callback_caml(self, ros_image):
         
         self.callback_cam(ros_image, 0)

    # Callback for right camera feed
    def callback_camr(self, ros_image):
         
         self.callback_cam(ros_image, 1)


if __name__ == "__main__":

    main = camera_test()
    main.loop()