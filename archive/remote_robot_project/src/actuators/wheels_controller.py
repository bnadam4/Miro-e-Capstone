#!/usr/bin/python3

# ----------------------------------------------
# Title: wheels_controller.py
# Description: Actuator for controlling the wheels of MiRo
# - Rotates the robot with a given angular velocity for a specified duration
# Author: Bryce
# Date created: Mar 1, 2025
# Date modified: Mar 1, 2025
# ----------------------------------------------

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
import time
import os
import math
import miro2 as miro
import numpy as np

ANGLE_VEL_MAX = miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S


class WheelsController:

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        #rospy.sleep(2.0)

        # Define ROS publisher
        self.pub_cmd = rospy.Publisher(topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=10)

        # Create object to hold published data
        self.velocity = TwistStamped()        # Publish the initial head position

        self.dr = 0
        self.dtheta = 0
        # Convert to command velocity
        # (self.dr, self.dtheta) = miro.utils.wheel_speed2cmd_vel(self.wheel_speed)

        # Set velocity values
        self.velocity.twist.linear.x = self.dr
        self.velocity.twist.angular.z = self.dtheta

        self.pub_cmd.publish(self.velocity)

# ----------------------------------------------
#  Updates Positions
# ----------------------------------------------
    def set_ang_vel(self, ang_vel):
        """Updates the ang vel of the wheels"""

        # Update the angular velocity
        self.velocity.twist.angular.z = ang_vel
        
        # Publish the updated velocity
        self.pub_cmd.publish(self.velocity)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed

    def rotate(self, ang_vel, duration):
        """Rotates the robot with a given angular velocity for a specified duration"""
        print("Rotating the robot with angular velocity: {} for {} seconds".format(ang_vel, duration))
        # Set the angular velocity
        start_time = time.time()
        while time.time() - start_time < duration:
            self.set_ang_vel(ang_vel)
            rospy.sleep(0.01)  # Sleep briefly to avoid overwhelming the system

        # Stop the rotation by setting angular velocity to 0
        #self.set_ang_vel(0)
        print("Rotation complete")

    def test(self):
        """Test the robot by rotating it"""
        # Rotate the robot
        print("Wheel max speed is {}".format(ANGLE_VEL_MAX))
        self.rotate(ANGLE_VEL_MAX/4, 2)
        self.rotate(-ANGLE_VEL_MAX/4, 2)
        self.rotate(0, 1)
        




