#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: November 5, 2024
# Last modifiec: November 5, 2024
#
# Face detection routine. Locates faces so that MiRo can look at them

import time
import cv2
import os

import miro2 as miro

# Robot specific libraries
import rospy
from sensor_msgs.msg import CompressedImage
import miro2 as miro

import cv2
from cv_bridge import CvBridge, CvBridgeError


class AttendFace:

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

        # state
        self.channels_to_process = [0, 1]
        if not self.image_stitcher is None:
            self.channels_to_process = [2]
        outfile = [None, None, None]
        self.outcount = [0] * len(outfile)
        self.t0 = time.time()
        self.cam_names = ['left', 'right', 'stitched']

        # Face detection and attending variables
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.yaw = 0
        self.yaw_speed = 0.02

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo camera feed")

        # Initialize a ROS node to communicate with MiRo
        # rospy.init_node("node_aruco_detector")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

    def check_face(self):

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
        for index in self.channels_to_process:

            # get image
            image = self.input_camera[index]

            # if present
            if not image is None:

                # handle
                self.input_camera[index] = None

                # Convert the image to grayscale for better accuracy
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # Detect faces in the image
                faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=6)
                # Draw rectangles around the detected faces
                for (x, y, w, h) in faces:
                    cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    face_center = int(x + w/2)
                    # Draw a line at the rectangle center
                    cv2.line(image, (face_center, y), (face_center, y + h), (0, 255, 0), 2)
                    # print("face center for index ", index, " is ", face_center)
                    if index == 0:
                        if face_center < 465 and self.yaw < 55.0:
                            self.yaw += self.yaw_speed
                        elif face_center > 515 and self.yaw > -55.0:
                            self.yaw -= self.yaw_speed
                    elif index == 1:
                        if face_center < 150 and self.yaw < 55.0:
                            self.yaw += self.yaw_speed
                        elif face_center > 200 and self.yaw > -55.0:
                            self.yaw -= self.yaw_speed

                # show
                cv2.imshow("Camera Feed: " + self.cam_names[index], image)
                cv2.waitKey(1)

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