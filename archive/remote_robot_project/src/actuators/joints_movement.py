#!/usr/bin/python3

# ----------------------------------------------
# Title: joints_movement.py
# Description: Controls the head movements for nodding (yes) and shaking (no)
# Author: Jasmine
# Date created: Dec 24, 2024
# Date modified: Feb 5, 2024
# ----------------------------------------------

import rospy
from actuators.joints_controller import JointsController

class JointsMovement:
    def __init__(self):
        # Initialize the JointsController to control the robot's head and neck
        self.controller = JointsController()

    def nod(self, duration, repetitions):
        """Simulates a 'yes' gesture by moving the head up and down.
        
        Args:
            duration (float): The time in seconds for each pitch movement (up/down).
            repetitions (int): The number of times to repeat the nodding motion.
        """
        rospy.loginfo("Starting nod (yes) gesture.")
        
        for _ in range(repetitions):
            # Move the head pitch up (nodding 'yes')
            self.controller.move_pitch(duration/2/repetitions, -4.0)  # Move to -15 degrees pitch (looking up)
            rospy.sleep(0.05)
            
            # Move the head pitch down (nodding 'yes')
            self.controller.move_pitch(duration/2/repetitions, 4.0)  # Move to the neutral position (looking straight)
            

        rospy.loginfo(f"Nod (yes) gesture completed with {repetitions} repetitions.")

    def shake(self, duration, repetitions):
        """Simulates a 'no' gesture by moving the head left and right.
        
        Args:
            duration (float): The time in seconds for each yaw movement (left/right).
            repetitions (int): The number of times to repeat the shaking motion.
        """
        rospy.loginfo("Starting shake (no) gesture.")
        
        for _ in range(repetitions):
            # Move the head yaw left (shaking 'no')
            self.controller.move_yaw(duration/2/repetitions, -10.0)  # Move yaw to the left (-10 degrees)
            rospy.sleep(0.05)

            # Move the head yaw right (shaking 'no')
            self.controller.move_yaw(duration/2/repetitions, 10.0)  # Move yaw to the right (10 degrees)
        
        self.controller.move_yaw(duration/2/repetitions/10, 0.0)  # Move yaw to the right (10 degrees)
            

        rospy.loginfo(f"Shake (no) gesture completed with {repetitions} repetitions.")



