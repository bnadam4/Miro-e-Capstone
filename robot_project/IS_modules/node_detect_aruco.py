#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: October 8, 2024
# Last modifiec: April 8, 2024
#
# Aruco detection routine. Used to find arucos that start or end activities

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

# Global variables
MIN_ARUCO_AREA = 1500

START_BREATH_ARUCO = 22
START_DANCE_ARUCO = 30

START_AUDIOBOOK_EMPEROR_ARUCO = 29
START_AUDIOBOOK_RUMPELSTILTSKIN_ARUCO = 28
START_AUDIOBOOK_FROG_ARUCO = 32
SHUT_DOWN_ARUCO = 33

START_RELAX_FULL = 36

START_ARMS_RELAX = 23
START_BACK_RELAX = 24
START_LEGS_RELAX = 25
START_TUMMY_RELAX = 26

START_SHUTDOWN_ARUCO = 33

END_ARUCO = 27

class NodeDetectAruco:

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
        self.breath_ex_ON = False
        self.aruco_seen = False
        #self.muscle_relax_ON = False
        #self.audiobook_ON = False
        self.exit_behaviour = False
        self.shut_down_ON= False
        self.relax_all = False

        self.rupelstiltskin = False
        self.emperor = False
        self.frog = False
        self.dance = False

        self.relax_arms = False
        self.relax_back = False
        self.relax_legs = False
        self.relax_tummy = False

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo aruco dector camera feed")

        # Initialize a ROS node to communicate with MiRo
        # rospy.init_node("node_aruco_detector")
        # Give it some time to make sure everything is initialised
        # rospy.sleep(2.0)

    def tick_camera(self):

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

                # Perform Aruco detection
                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                parameters = cv2.aruco.DetectorParameters()

                # Create the ArUco detector
                detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
                # Detect the markers
                corners, ids, rejected = detector.detectMarkers(gray)
                # Print the detected markers
                # if len(ids) > 0:
                #    print("Detected markers:", ids)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(image, corners, ids)
                    if self.aruco_seen == False:
                        self.aruco_seen = True


                    for i, id in enumerate(ids):
                        # Calculate the area of the detected marker
                        x1, y1 = corners[i][0][0]
                        x2, y2 = corners[i][0][1]
                        x3, y3 = corners[i][0][2]
                        x4, y4 = corners[i][0][3]
                        track_area = abs((x1 * y2 - y1 * x2) + (x2 * y3 - y2 * x3) + (x3 * y4 - y3 * x4) + (x4 * y1 - y4 * x1)) / 2


                        # Check if the area is greater than the minimum threshold
                        if track_area > MIN_ARUCO_AREA:


                            if id == START_SHUTDOWN_ARUCO:
                                self.shut_down_ON = True
                                print("START_SHUTDOWN_ARUCO has been seen")
                            elif id == START_BREATH_ARUCO:
                                self.breath_ex_ON = True
                                print("START_BREATH_ARUCO has been seen")
                            elif id == START_RELAX_FULL:
                                self.relax_all = True
                                #self.muscle_relax_ON = True
                                print("START_RELAX_PROMPT has been seen")
                            elif id == START_ARMS_RELAX:
                                self.relax_arms = True
                                #self.muscle_relax_ON = True
                                #self.relax_prompt = True
                                print("START_ARMS_RELAX has been seen")
                            elif id == START_BACK_RELAX:
                                self.relax_back = True
                                #self.muscle_relax_ON = True
                                #self.relax_prompt = True
                                print("START_BACK_RELAX has been seen")
                            elif id == START_LEGS_RELAX:
                                self.relax_legs = True
                                #self.muscle_relax_ON = True
                                #self.relax_prompt = True
                                print("START_LEGS_RELAX has been seen")
                            elif id == START_TUMMY_RELAX:
                                self.relax_tummy = True
                                #self.muscle_relax_ON = True
                                #self.relax_prompt = True
                                print("START_TUMMY_RELAX has been seen")
                            elif id == START_AUDIOBOOK_EMPEROR_ARUCO:
                                self.emperor = True
                                #self.audiobook_ON = True
                                print("START_AUDIOBOOK_EMPEROR_ARUCO has been seen")
                            elif id == START_AUDIOBOOK_RUMPELSTILTSKIN_ARUCO:
                                self.rupelstiltskin = True
                                #self.audiobook_ON = True
                                print("START_AUDIOBOOK_RUMPELSTILTSKIN_ARUCO has been seen")
                            elif id == START_AUDIOBOOK_FROG_ARUCO:
                                self.frog = True
                                #self.audiobook_ON = True
                                print("START_AUDIOBOOK_FROG_ARUCO has been seen")
                            elif id == END_ARUCO:
                                self.exit_behaviour = True
                                print("END_ARUCO has been seen")
                            elif id == SHUT_DOWN_ARUCO:
                                self.shut_down = True
                                print("SHUT_DOWN_ARUCO has been seen")
                            elif id == START_DANCE_ARUCO:
                                self.dance = True
                                print("START_DANCE_ARUCO has been seen")
                else:
                    self.aruco_seen = False

                # show
                #cv2.imshow("Camera Feed: " + self.cam_names[index], image)
                #cv2.waitKey(1)

    def callback_cam(self, ros_image, index):
        # silently (ish) handle corrupted JPEG frames
        try:
            # convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")

            # store image for display
            self.input_camera[index] = image

        except CvBridgeError as e:
            # swallow error, silently
            # print(e)
            pass

    # Callback for left camera feed
    def callback_caml(self, ros_image):
         self.callback_cam(ros_image, 0)

    # Callback for right camera feed
    def callback_camr(self, ros_image):
         self.callback_cam(ros_image, 1)