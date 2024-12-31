#!/usr/bin/python3

# ----------------------------------------------
# Title: distance_test.py
# Description: 
# - Add on from attend_face.py
# - Gets the distance between the person and miro
# Author: Jasmine
# Date created: Dec 30, 2024
# Date modified: Dec 30, 2024
# ----------------------------------------------

import time
import sys
import os
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import miro2 as miro
import cv2.aruco as aruco

class CameraAndFaceDetection:

    def __init__(self):
        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Variables to store input data
        self.input_camera = [None, None]
        self.t_input_camera = [[], []]

        # Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()

        # Image stitcher
        self.image_stitcher = None

        # Subscribers to left camera (caml) and right camera (camr)
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed", 
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed", 
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

        # Face detection
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.yaw = 0
        self.yaw_speed = 0.02

        # Aruco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Real-world reference: average width of a human face in cm
        self.face_width_cm = 15  # average face width in cm

        # Focal length in pixels (this needs to be calibrated for your camera)
        self.focal_length = 500  # Placeholder for camera calibration

        # state
        self.channels_to_process = [0, 1]
        if not self.image_stitcher is None:
            self.channels_to_process = [2]
        self.cam_names = ['left', 'right', 'stitched']

        # Main control loop iteration counter
        self.counter = 0

        # Initialize ROS node
        rospy.init_node("camera_face_aruco_node")
        rospy.sleep(2.0)

    def process_camera(self):
        # Process both Aruco markers and face detection
        for index in self.channels_to_process:
            try:
                # Get image
                image = self.input_camera[index]
                
                # If image is available
                if image is not None:
                    self.input_camera[index] = None

                    # Handle Aruco detection
                    #self.detect_aruco(image, index)

                    # Handle face detection
                    self.detect_faces(image, index)

                    # Show the image feed
                    cv2.imshow("Camera Feed: " + self.cam_names[index], image)
                    cv2.waitKey(1)

            except Exception as e:
                print(f"Error during camera processing for index {index}: {e}")

    def detect_faces(self, image, index):
        try:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=6)

            for (x, y, w, h) in faces:
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                face_center = int(x + w / 2)
                cv2.line(image, (face_center, y), (face_center, y + h), (0, 255, 0), 2)
                print(f"Face center for index {index} is {face_center}")

                # Calculate the distance to the face (in cm)
                distance = self.estimate_distance(w)
                print(f"Estimated distance to face: {distance:.2f} cm")

                self.adjust_yaw(face_center, index)

        except Exception as e:
            print(f"Error during face detection for index {index}: {e}")

    def estimate_distance(self, face_width_pixels):
        """
        Estimate the distance to the face based on its width in the image.
        Formula: Distance = (Real Face Width * Focal Length) / Face Width in Image (in pixels)
        """
        try:
            # Avoid division by zero
            if face_width_pixels == 0:
                return 0.0
            distance = ((self.face_width_cm * self.focal_length) / face_width_pixels)-20
            return distance
        except Exception as e:
            print(f"Error in distance estimation: {e}")
            return 0.0

    def adjust_yaw(self, face_center, index):
        try:
            if index == 0:  # Left camera
                if face_center < 465 and self.yaw < 55.0:
                    self.yaw += self.yaw_speed
                elif face_center > 515 and self.yaw > -55.0:
                    self.yaw -= self.yaw_speed
            elif index == 1:  # Right camera
                if face_center < 150 and self.yaw < 55.0:
                    self.yaw += self.yaw_speed
                elif face_center > 200 and self.yaw > -55.0:
                    self.yaw -= self.yaw_speed
        except Exception as e:
            print(f"Error in yaw adjustment for index {index}: {e}")

    def callback_cam(self, ros_image, index):
        try:
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
            self.input_camera[index] = image
        except CvBridgeError as e:
            print(f"Error in converting ROS image for index {index}: {e}")

    def callback_caml(self, ros_image):
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):
        self.callback_cam(ros_image, 1)

    def loop(self):
        while not rospy.core.is_shutdown():
            self.process_camera()
            time.sleep(0.02)

if __name__ == "__main__":
    main = CameraAndFaceDetection()
    main.loop()

