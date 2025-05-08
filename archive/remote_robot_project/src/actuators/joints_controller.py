#!/usr/bin/python3

# ----------------------------------------------
# Title: joints_controller.py
# Description: Fundation code
# - Moves head and neck polynomials and sine movement
# - Updates and publishs those movements
# Author: Bryce, Jasmine 
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import os
import math

# Constants for head movements
HEAD_YAW_LEFT = -55.0  # Minimum yaw (left)
HEAD_YAW_RIGHT = 55.0  # Maximum yaw (right)

HEAD_PITCH_UP = -22.0  # Minimum pitch (up)
HEAD_PITCH_DOWN = 8.0  # Maximum pitch (down)

# Constants for neck positions
NECK_MAX = 40.0  # Maximum neck position (in degrees)
NECK_MIN = 15.0  # Minimum neck position (in degrees)

class JointsController:

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        #rospy.sleep(2.0)

        # Initialize ROS publisher for kinematic joints (yaw and pitch for head movement)
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)

        # Initialize the kinematic joint positions (tilt, lift, yaw, pitch)
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(20.0), math.radians(0), math.radians(-7)]  # Default head position (yaw=0, pitch=-7)

        # Publish the initial head position
        self.pub_kin.publish(self.kin_joints)  # Set head to initial position

        # Track current positions
        self.current_yaw_position = 0.0  # Yaw position in degrees
        self.current_pitch_position = -7.0  # Pitch position in degrees
        self.current_neck_position = 20.0  # Neck position in degrees

# ----------------------------------------------
#  Updates Positions
# ----------------------------------------------
    def position_yaw(self, position):
        """Updates the head yaw to a specific position."""
        #print(f"Moving head yaw to position {position} degrees...")
        
        # Ensure the yaw position is within the valid range (HEAD_YAW_LEFT to HEAD_YAW_RIGHT)
        position = min(max(position, HEAD_YAW_LEFT), HEAD_YAW_RIGHT)
        self.current_yaw_position = position

        # Update the kinematic joint position (yaw is at index 2)
        self.kin_joints.position[1] = math.radians(self.current_neck_position)
        self.kin_joints.position[2] = math.radians(position)
        self.kin_joints.position[3] = math.radians(self.current_pitch_position)

        self.pub_kin.publish(self.kin_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        
        #print(f"Head yaw moved to {position} degrees.")

    def position_pitch(self, position):
        """Updates the head pitch to a specific position."""
        #print(f"Moving head pitch to position {position} degrees...")
        
        # Ensure the pitch position is within the valid range (HEAD_PITCH_UP to HEAD_PITCH_DOWN)
        position = min(max(position, HEAD_PITCH_UP), HEAD_PITCH_DOWN)
        self.current_pitch_position = position

        # Update the kinematic joint position (pitch is at index 3)
        self.kin_joints.position[1] = math.radians(self.current_neck_position)
        self.kin_joints.position[2] = math.radians(self.current_yaw_position)
        self.kin_joints.position[3] = math.radians(position)

        self.pub_kin.publish(self.kin_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        
        #print(f"Head pitch moved to {position} degrees.")
        
    def position_neck(self, position):
        """Updates the neck to a specific position."""
        #print(f"Moving neck to position {position} degrees...")
        
        # Ensure the position is within a valid range (NECK_MIN to NECK_MAX)
        position = min(max(position, NECK_MIN), NECK_MAX)
        self.current_neck_position = position
        
        # Update the kinematic joint position (we modify the lift position)
        self.kin_joints.position[1] = math.radians(position)
        self.kin_joints.position[2] = math.radians(self.current_yaw_position)
        self.kin_joints.position[3] = math.radians(self.current_pitch_position)

        #self.kin_joints.position = [0.0, math.radians(position), 0.0, 0.0]  
        self.pub_kin.publish(self.kin_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        
        #print(f"Neck moved to {position} degrees.")

# ----------------------------------------------
#  Calculates Smooth Movement
# ----------------------------------------------
    def move_yaw(self, duration, target_position):
        """Move yaw from current_yaw_position to target_position over the specified duration."""
        #print(f"Moving head yaw from {self.current_yaw_position} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_position = self.current_yaw_position  # Save the starting position
        last_sent_position = initial_position  # Track the last sent position
        
        while time.time() - start_time < duration:
            # Calculate the elapsed time and normalize it
            elapsed_time = time.time() - start_time
            normalized_time = elapsed_time / duration

            # Apply cosine easing for smooth transition (ease-in-out)
            ease_in_out = 0.5 - 0.5 * math.cos(math.pi * normalized_time)
            step = initial_position + ease_in_out * (target_position - initial_position)
            
            # Clamp the yaw position to the allowed range [-55, 55]
            step = max(min(step, 55), -55)
            
            # Check if the position has changed before sending it
            if abs(step - last_sent_position) > 0.01:  # Use a small threshold to avoid sending minor updates
                self.position_yaw(step)
                last_sent_position = step  # Update the last sent position
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure the final position is exactly the target position
        if abs(target_position - last_sent_position) > 0.01:  # Ensure final position is updated if needed
            self.position_yaw(target_position)
        #print(f"Head yaw fully moved to {target_position}.")
        self.current_yaw_position = target_position  # Update the current position to target


    def move_pitch(self, duration, target_position):
        """Move pitch from current_pitch_position to target_position over the specified duration."""
        #print(f"Moving head pitch from {self.current_pitch_position} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_position = self.current_pitch_position  # Save the starting position
        last_sent_position = initial_position  # Track the last sent position
        
        while time.time() - start_time < duration:
            # Calculate the elapsed time and normalize it
            elapsed_time = time.time() - start_time
            t = elapsed_time / duration
            
            # Apply quadratic easing for smooth transition (ease-in-out)
            ease_t = t * t * (3 - 2 * t)  # Smooth ease function (starts slow, speeds up, slows down)
            
            # Calculate the intermediate pitch position based on eased time
            step = initial_position + ease_t * (target_position - initial_position)
            
            # Clamp the pitch position to the allowed range [-22, 8]
            step = max(min(step, 8), -22)
            
            # Only update position if there is a significant change
            if abs(step - last_sent_position) > 0.01:  # Using a threshold to avoid small updates
                self.position_pitch(step)
                last_sent_position = step  # Update the last sent position
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure the final position is exactly the target position
        if abs(target_position - last_sent_position) > 0.01:  # Ensure the final position is updated if needed
            self.position_pitch(target_position)
        #print(f"Head pitch fully moved to {target_position}.")
        self.current_pitch_position = target_position  # Update the current position to target


    def move_neck(self, duration, target_position):
        """Move neck from current_neck_position to target_position over the specified duration."""
        #print(f"Moving neck from {self.current_neck_position} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_position = self.current_neck_position  # Save the starting position
        last_sent_position = initial_position  # Track the last sent position
        
        while time.time() - start_time < duration:
            # Calculate the elapsed time and normalize it
            elapsed_time = time.time() - start_time
            t = elapsed_time / duration
            
            # Apply cubic easing for smooth transition (ease-in-out)
            ease_t = t * t * (3 - 2 * t)  # You can adjust this for more natural behavior
            
            # Calculate the intermediate neck position based on eased time
            step = initial_position + ease_t * (target_position - initial_position)
            
            # Only update position if there is a significant change
            if abs(step - last_sent_position) > 0.01:  # Using a threshold to avoid small updates
                self.position_neck(step)
                last_sent_position = step  # Update the last sent position
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure the final position is exactly the target position
        if abs(target_position - last_sent_position) > 0.01:  # Ensure the final position is updated if needed
            self.position_neck(target_position)
        #print(f"Neck fully moved to {target_position}.")
        self.current_neck_position = target_position  # Update the current position to target

    # ----------------------------------------------
    def move_all(self, duration, target_yaw, target_pitch, target_neck):
        """Move yaw, pitch, and neck together to their target positions over the specified duration."""
        #print(f"Moving head yaw to {target_yaw}, pitch to {target_pitch}, and neck to {target_neck} over {duration} seconds...")
        
        start_time = time.time()
        initial_yaw = self.current_yaw_position
        initial_pitch = self.current_pitch_position
        initial_neck = self.current_neck_position
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = elapsed_time / duration
            
            # Apply easing for smooth transition (ease-in-out)
            ease_t = t * t * (3 - 2 * t)
            
            # Calculate intermediate positions
            step_yaw = initial_yaw + ease_t * (target_yaw - initial_yaw)
            step_pitch = initial_pitch + ease_t * (target_pitch - initial_pitch)
            step_neck = initial_neck + ease_t * (target_neck - initial_neck)
            
            # Clamp positions to their allowed ranges
            step_yaw = max(min(step_yaw, HEAD_YAW_RIGHT), HEAD_YAW_LEFT)
            step_pitch = max(min(step_pitch, HEAD_PITCH_DOWN), HEAD_PITCH_UP)
            step_neck = max(min(step_neck, NECK_MAX), NECK_MIN)
            
            # Update positions
            self.position_yaw(step_yaw)
            self.position_pitch(step_pitch)
            self.position_neck(step_neck)
            
            rospy.sleep(0.02)
        
        # Ensure final positions are exactly the target positions
        self.position_yaw(target_yaw)
        self.position_pitch(target_pitch)
        self.position_neck(target_neck)
        
        #print(f"Head yaw moved to {target_yaw}, pitch to {target_pitch}, and neck to {target_neck}.")
