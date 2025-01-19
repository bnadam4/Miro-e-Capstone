#!/usr/bin/python3
#
# Author: Kento
# Date created: January 10, 2025
# Last modifiec: January 11, 2025
#
# Displays MiRo camera feed and aruco detection

# Misc useful python libraries
import time
import sys
import os
import threading
import math  # Add this import

# Robot specific libraries
import rospy
from sensor_msgs.msg import CompressedImage
import miro2 as miro
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Stereovision:
    def __init__(self):
        self.l_cam = None
        self.frame_left = None
        self.r_cam = None
        self.frame_right = None
        self.image_size = 640, 360

        self.left_gray = None
        self.right_gray = None

        self.face_detected = False
        self.face_distance = 0

        self.shift_x_left = 275
        self.shift_y_left = 0
        self.shift_x_right = 85
        self.shift_y_right = 0 
        self.stop = False

        # Remove mutex
        # self.mutex = threading.Lock()

        # Load calibration parameters
        self.load_cam_param()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Variables to store input data
        self.input_camera = [None, None]
        self.t_input_camera = [[], []]

        # Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()

        # Subscribers to left camera (caml) and right camera (camr)
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed", 
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed", 
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)
        
        

        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Face detection and attending variables
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

        # initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.baseline_x = abs(self.T[0])  # The horizontal baseline
        self.focal_length = self.K1[0, 0] + self.K2[0,0] / 2  # Average focal length of each camera

    def load_cam_param(self):
        """Load calibration parameters from YAML file."""
        fs = cv2.FileStorage('cam_stereo.yml', cv2.FILE_STORAGE_READ)

        # Check if the file was opened correctly
        if not fs.isOpened():
            print("Error: Could not open calibration file!")
            exit()

        # Read the calibration parameters
        self.K1 = fs.getNode('K1').mat()
        self.K2 = fs.getNode('K2').mat()
        self.D1 = fs.getNode('D1').mat()
        self.D2 = fs.getNode('D2').mat()
        self.R = fs.getNode('R').mat()
        self.T = fs.getNode('T').mat()
        self.R1 = fs.getNode('R1').mat()
        self.R2 = fs.getNode('R2').mat()
        self.P1 = fs.getNode('P1').mat()
        self.P2 = fs.getNode('P2').mat()
        self.Q = fs.getNode('Q').mat()

        fs.release()  # Close the file

    def loop(self):
        # state
        channels_to_process = [0, 1]

        # Main control loop iteration counter
        self.counter = 0
        print("MiRo stereo camera feed")

        # Perform stereo rectification using fisheye-specific function
        self.R1, self.R2, self.P1, self.P2, self.Q = cv2.fisheye.stereoRectify(self.K1, self.D1, self.K2, self.D2, self.image_size, self.R, self.T, flags=cv2.CALIB_ZERO_DISPARITY)

        # Adjust the principal point in the P1 matrix to shift the left view
        self.P1[0, 2] += self.shift_x_left
        self.P1[1, 2] += self.shift_y_left

        # Adjust the principal point in the P2 matrix to shift the right view
        self.P2[0, 2] += self.shift_x_right
        self.P2[1, 2] += self.shift_y_right

        # Generate the undistort and rectify maps using fisheye model
        stereoMapL_x, stereoMapL_y = cv2.fisheye.initUndistortRectifyMap(self.K1, self.D1, self.R1, self.P1, self.image_size, cv2.CV_16SC2)
        stereoMapR_x, stereoMapR_y = cv2.fisheye.initUndistortRectifyMap(self.K2, self.D2, self.R2, self.P2, self.image_size, cv2.CV_16SC2)


        while not rospy.core.is_shutdown():
            if self.stop:
                break
            # for each channel to process
            for index in channels_to_process:
                # get image
                image = self.input_camera[index]
                if image is not None and index == 0:
                    l_cam = image
                    self.frame_left = cv2.remap(l_cam, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                    self.left_gray = cv2.cvtColor(self.frame_left, cv2.COLOR_BGR2GRAY)
                elif image is not None and index == 1:
                    r_cam = image
                    self.frame_right = cv2.remap(r_cam, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                    self.right_gray = cv2.cvtColor(self.frame_right, cv2.COLOR_BGR2GRAY)

            # Detect person in both images
            # person_left_center_x, person_left_center_y, person_left_box, person_left_confidence = self.detect_person(left_gray, self.hog)
            # person_right_center_x, person_right_center_y, person_right_box, person_right_confidence = self.detect_person(right_gray, self.hog)

            # if person_left_center_x and person_right_center_x:
            #     # Calculate disparity and estimate distance
            #     distance = self.calculate_distance(person_left_center_x, person_right_center_x)
            #     if distance:
            #         print(f"PERSON - Distance: {distance} meters")
            #         # Visualize detections
            #         cv2.rectangle(frame_left, (person_left_box[0], person_left_box[1]), 
            #                     (person_left_box[0] + person_left_box[2], person_left_box[1] + person_left_box[3]), (0, 255, 0), 2)
            #         cv2.putText(frame_left, f"PERSON - Conf: {person_left_confidence:.2f}", 
            #                     (person_left_box[0], person_left_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            #         cv2.rectangle(frame_right, (person_right_box[0], person_right_box[1]), 
            #                     (person_right_box[0] + person_right_box[2], person_right_box[1] + person_right_box[3]), (0, 255, 0), 2)
            #         cv2.putText(frame_right, f"PERSON - Conf: {person_right_confidence:.2f}", 
            #                     (person_right_box[0], person_right_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #     else:
            #         print("PERSON - Disparity is zero or negative; unable to calculate distance.")


            face_left_center_x, face_left_center_y, face_left_box, face_left_confidence = self.detect_face(self.left_gray, self.face_cascade)
            face_right_center_x, face_right_center_y, face_right_box, face_right_confidence = self.detect_face(self.right_gray, self.face_cascade)


            if face_left_center_x and face_right_center_x:
                # Calculate disparity and estimate distance
                
                self.face_detected = True
                self.face_distance = self.calculate_distance(face_left_center_x, face_right_center_x)
                #print(f"FACE - Distance: {self.face_distance} meters")
                # if self.face_distance:
                #     # # Calculate 3D position
                #     # X, Y, Z = self.calculate_3d_position(face_left_center_x, face_left_center_y, face_right_center_x, face_right_center_y)
                #     # if X is not None and Y is not None and Z is not None:
                #     #     print(f"FACE - 3D Position: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")

                #     # # Visualize detections
                #     # cv2.rectangle(self.frame_left, (face_left_box[0], face_left_box[1]), 
                #     #             (face_left_box[0] + face_left_box[2], face_left_box[1] + face_left_box[3]), (0, 255, 0), 2)

                #     # cv2.rectangle(self.frame_right, (face_right_box[0], face_right_box[1]), 
                #     #             (face_right_box[0] + face_right_box[2], face_right_box[1] + face_right_box[3]), (0, 255, 0), 2)
                # else:
                #     print("FACE - Disparity is zero or negative; unable to calculate distance.")
            else:
                self.face_detected = False

    def calculate_3d_position(self, x_left, y_left, x_right, y_right):
        """
        Calculate the 3D position of an object given its pixel coordinates in the left and right images.
        
        Args:
        - x_left, y_left: Pixel coordinates of the object in the left image.
        - x_right, y_right: Pixel coordinates of the object in the right image.
        
        Returns:
        - X, Y, Z: 3D coordinates of the object with respect to the left camera.
        """
        # Focal length (average of both cameras, in pixels)
        f_x = self.focal_length
        
        # Baseline (horizontal distance between the two cameras, in meters)
        B = self.baseline_x
        
        # Principal points (in pixels)
        o_x = self.K1[0, 2]
        o_y = self.K1[1, 2]
        
        # Disparity
        disparity = x_left - x_right
        
        if disparity == 0:
            return None, None, None  # Avoid division by zero
        
        # Calculate 3D coordinates
        Z = (B * f_x) / disparity  # Depth (Z-coordinate)
        X = (B * (x_left - o_x)) / disparity  # X-coordinate
        Y = (B * f_x * (y_left - o_y)) / (self.K1[1, 1] * disparity)  # Y-coordinate
        
        return X, Y, Z

    def detect_person(self, image, hog):
        """Detect person in the image using HOG."""
        boxes, weights = hog.detectMultiScale(image, winStride=(8, 8))
        filtered_boxes = [box for i, box in enumerate(boxes) if weights[i] > 0.5]
        if len(filtered_boxes) > 0:
            # Assume first detected person is the target
            (x, y, w, h) = filtered_boxes[0]
            center_x = x + w // 2
            center_y = y + h // 2
            return center_x, center_y, (x, y, w, h), weights[0]
        return None, None, None, None
    
    def detect_face(self, grayimage, face_cascade):
        """Detect face in the image using HOG."""
        faces = face_cascade.detectMultiScale(grayimage, scaleFactor=1.1, minNeighbors=6)
        weights = [1.0] * len(faces)
        if len(faces) > 0:
            # Assume first detected person is the target
            (x, y, w, h) = faces[0]
            center_x = x + w // 2
            center_y = y + h // 2
            return center_x, center_y, (x, y, w, h), weights[0]
        return None, None, None, None

    def calculate_distance(self, left_center_x, right_center_x):
        # Adjust the left and right center x by their respective shift values
        adjusted_left_center_x = left_center_x - self.shift_x_left
        adjusted_right_center_x = right_center_x - self.shift_x_right
        disparity = adjusted_left_center_x - adjusted_right_center_x
        if disparity > 0:
            distance = (self.focal_length * self.baseline_x) / disparity
            return distance
        return -1

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

    def draw_frames(self):
        # Remove mutex usage
        # with self.mutex:
        if self.frame_left is not None and self.frame_right is not None:
            cv2.imshow("Left Camera", self.frame_left)
            cv2.imshow("Right Camera", self.frame_right)
            cv2.waitKey(1)


if __name__ == "__main__":
    main = Stereovision()
    main.loop()