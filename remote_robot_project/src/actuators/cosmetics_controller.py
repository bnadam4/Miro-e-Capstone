#!/usr/bin/python3

# ----------------------------------------------
# Title: cosmetics_controller.py
# Description: Fundation code
# - Moves eyes, ears and tails polynomials and sine movement
# - Updates and publishs those movements
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------

import rospy
import miro2 as miro
from std_msgs.msg import Float32MultiArray
import math
import time
import os
import numpy as np

# Generate enum and constants
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)

class CosmeticsController:

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        #rospy.sleep(2.0)
        self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.current_droop = 0.0
        self.current_wag = 0.0

        self.current_left_eye = 0.0
        self.current_right_eye = 0.0
        
        self.current_left_ear = 0.0
        self.current_right_ear = 0.0    

# ----------------------------------------------
#  Updates Positions
# ---------------------------------------------- 
    
    def position_ears(self, left_position, right_position):
        """Updates both ears to a specific position."""
        #print(f"Moving ears to position {left_position} and {right_position}...")
        
        # Ensure the position is within a valid range (0.0 to 1.0)
        left_position = min(max(left_position, 0.0), 1.0)
        right_position = min(max(right_position, 0.0), 1.0)
        
        self.current_left_ear = left_position
        self.current_right_ear = right_position
                
        # Set the position for both ears (left and right)
        self.cos_joints.data[droop] = self.current_droop
        self.cos_joints.data[wag] = self.current_wag
        self.cos_joints.data[left_eye] = self.current_left_eye
        self.cos_joints.data[right_eye] = self.current_right_eye   
        self.cos_joints.data[left_ear] = left_position
        self.cos_joints.data[right_ear] = right_position
        
        # Publish the updated positions
        self.pub_cos.publish(self.cos_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        #print("Ears moved.")

    
    def position_eyes(self, left_position, right_position):
        """Updates both eyes to a specific position."""
        #print(f"Moving ears to position {left_position} and {right_position}...")
        
        # Ensure the position is within a valid range (0.0 to 1.0)
        left_position = min(max(left_position, 0.0), 1.0)
        right_position = min(max(right_position, 0.0), 1.0)
        
        self.current_left_eye = left_position
        self.current_right_eye = right_position
                
        # Set the position for both eyes (left and right)
        self.cos_joints.data[droop] = self.current_droop
        self.cos_joints.data[wag] = self.current_wag
        self.cos_joints.data[left_eye] = left_position
        self.cos_joints.data[right_eye] = right_position
        self.cos_joints.data[left_ear] = self.current_left_ear
        self.cos_joints.data[right_ear] = self.current_right_ear
        
        # Publish the updated positions
        self.pub_cos.publish(self.cos_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        #print("Eyes moved.")       

    
    def position_tail(self, wag_position, droop_position):
        """Updates tail to a specific position."""
        #print(f"Moving tail to position {wag_position} and {droop_position}...")
        
        # Ensure the position is within a valid range (0.0 to 1.0)
        wag_position = min(max(wag_position, 0.0), 1.0)
        droop_position = min(max(droop_position, 0.0), 1.0)
        
        self.current_wag = wag_position
        self.current_droop = droop_position
                
        # Set the position for both eyes (left and right)
        self.cos_joints.data[droop] = droop_position
        self.cos_joints.data[wag] = wag_position
        self.cos_joints.data[left_eye] = self.current_left_eye
        self.cos_joints.data[right_eye] = self.current_right_eye
        self.cos_joints.data[left_ear] = self.current_left_ear
        self.cos_joints.data[right_ear] = self.current_right_ear
        
        # Publish the updated positions
        self.pub_cos.publish(self.cos_joints)
        rospy.sleep(0.02)  # Sleep briefly to ensure the command is processed
        #print("Tail moved.")        

        
# ----------------------------------------------
#  Calculates Smooth Movement
# ---------------------------------------------- 
###Ears###
    def move_ears(self, duration, target_position):
        """Move BOTH ears from current_position to target_position over the specified duration."""
        #print(f"Moving ears from {self.current_left_ear} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_left_position = self.current_left_ear  # Save the starting position
        initial_right_position = self.current_right_ear  # Save the starting position
        last_left_sent_position = initial_left_position  # Track the last sent position for left ear
        last_right_sent_position = initial_right_position  # Track the last sent position for right ear
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            easing_factor = 3 * t**2 - 2 * t**3  # Cubic ease-in-out
            
            # Calculate the intermediate position based on easing factor
            left_step = initial_left_position + easing_factor * (target_position - initial_left_position)
            right_step = initial_right_position + easing_factor * (target_position - initial_right_position)
            
            # Only update position if there is a significant change
            if abs(left_step - last_left_sent_position) > 0.01 or abs(right_step - last_right_sent_position) > 0.01:
                self.position_ears(left_step, right_step)
                last_left_sent_position = left_step  # Update the last sent position for left ear
                last_right_sent_position = right_step  # Update the last sent position for right ear
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        self.position_ears(target_position, target_position)
        #print(f"Ears fully moved to {target_position}.")
        self.current_left_ear = target_position  # Update the current position to target
        self.current_right_ear = target_position  # Update the current position to target

    def move_one_ear(self, duration, target_position, select):
        """Move ONE ear from current_position to target_position over the specified duration."""
        #print(f"Moving {select} ear from {self.current_left_ear if select == 'left' else self.current_right_ear} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_left_position = self.current_left_ear  # Save the starting position
        initial_right_position = self.current_right_ear  # Save the starting position
        last_left_sent_position = initial_left_position  # Track the last sent position for left ear
        last_right_sent_position = initial_right_position  # Track the last sent position for right ear
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            easing_factor = 3 * t**2 - 2 * t**3  # Cubic ease-in-out
            
            if select == "left":
                left_step = initial_left_position + easing_factor * (target_position - initial_left_position)
                # Only update position if there is a significant change
                if abs(left_step - last_left_sent_position) > 0.01:
                    self.position_ears(left_step, self.current_right_ear)
                    last_left_sent_position = left_step  # Update the last sent position for left ear
            else:
                right_step = initial_right_position + easing_factor * (target_position - initial_right_position)
                # Only update position if there is a significant change
                if abs(right_step - last_right_sent_position) > 0.01:
                    self.position_ears(self.current_left_ear, right_step)
                    last_right_sent_position = right_step  # Update the last sent position for right ear

            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        #print(f"{select.capitalize()} ear fully moved to {target_position}.")
###EYES###
    def move_eyes(self, duration, target_position):
        """Move BOTH eyes from current_position to target_position over the specified duration."""
        #print(f"Moving eyes from {self.current_left_eye} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_left_position = self.current_left_eye  # Save the starting position
        initial_right_position = self.current_right_eye  # Save the starting position
        last_left_sent_position = initial_left_position  # Track the last sent position for left eye
        last_right_sent_position = initial_right_position  # Track the last sent position for right eye
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            easing_factor = 3 * t**2 - 2 * t**3  # Cubic ease-in-out
            
            # Calculate the intermediate position based on easing factor
            left_step = initial_left_position + easing_factor * (target_position - initial_left_position)
            right_step = initial_right_position + easing_factor * (target_position - initial_right_position)
            
            # Only update position if there is a significant change
            if abs(left_step - last_left_sent_position) > 0.01 or abs(right_step - last_right_sent_position) > 0.01:
                self.position_eyes(left_step, right_step)
                last_left_sent_position = left_step  # Update the last sent position for left eye
                last_right_sent_position = right_step  # Update the last sent position for right eye
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        self.position_eyes(target_position, target_position)
        #print(f"Eyes fully moved to {target_position}.")
        self.current_left_eye = target_position  # Update the current position to target
        self.current_right_eye = target_position  # Update the current position to target

    def move_one_eye(self, duration, target_position, select):
        """Move ONE eye from current_position to target_position over the specified duration."""
        #print(f"Moving {select} eye from {self.current_left_eye if select == 'left' else self.current_right_eye} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_left_position = self.current_left_eye  # Save the starting position
        initial_right_position = self.current_right_eye  # Save the starting position
        last_left_sent_position = initial_left_position  # Track the last sent position for left eye
        last_right_sent_position = initial_right_position  # Track the last sent position for right eye
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            easing_factor = 3 * t**2 - 2 * t**3  # Cubic ease-in-out
            
            if select == "left":
                left_step = initial_left_position + easing_factor * (target_position - initial_left_position)
                # Only update position if there is a significant change
                if abs(left_step - last_left_sent_position) > 0.01:
                    self.position_eyes(left_step, self.current_right_eye)
                    last_left_sent_position = left_step  # Update the last sent position for left eye
            else:
                right_step = initial_right_position + easing_factor * (target_position - initial_right_position)
                # Only update position if there is a significant change
                if abs(right_step - last_right_sent_position) > 0.01:
                    self.position_eyes(self.current_left_eye, right_step)
                    last_right_sent_position = right_step  # Update the last sent position for right eye

            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        #print(f"{select.capitalize()} eye fully moved to {target_position}.")
###TAIL###
    def move_wag_tail(self, duration, target_position):
        """Move wag tail from current_position to target_position over the specified duration."""
        #print(f"Moving wag tail from {self.current_wag} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_position = self.current_wag  # Save the starting position
        last_sent_position = initial_position  # Track the last sent position for wag tail
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            sine_factor = (math.sin(math.pi * (t - 0.5)) + 1)
            
            # Calculate the intermediate position based on the sine wave scaling factor
            step = initial_position + sine_factor * (target_position - initial_position)
            
            # Only update position if there is a significant change
            if abs(step - last_sent_position) > 0.01:
                self.position_tail(step, self.current_droop)
                last_sent_position = step  # Update the last sent position for wag tail
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        self.position_tail(target_position, self.current_droop)
        #print(f"Tail moved to {target_position}.")
        self.current_wag = target_position  # Update the current position to target

    def move_droop_tail(self, duration, target_position):
        """Move droop tail from current_position to target_position over the specified duration."""
        #print(f"Moving droop tail from {self.current_droop} to {target_position} over {duration} seconds...")
        
        start_time = time.time()
        initial_position = self.current_droop  # Save the starting position
        last_sent_position = initial_position  # Track the last sent position for droop tail
        
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            t = min(elapsed_time / duration, 1)  # Normalize time (max 1)
            sine_factor = (math.sin(math.pi * (t - 0.5)) + 1)
            
            # Calculate the intermediate position based on the sine wave scaling factor
            step = initial_position + sine_factor * (target_position - initial_position)
            
            # Only update position if there is a significant change
            if abs(step - last_sent_position) > 0.01:
                self.position_tail(self.current_wag, step)
                last_sent_position = step  # Update the last sent position for droop tail
            
            rospy.sleep(0.02)  # Sleep briefly for smooth transition
        
        # Ensure final position is exactly the target position
        self.position_tail(self.current_wag, target_position)
        #print(f"Droop tail moved to {target_position}.")
        self.current_droop = target_position  # Update the current position to target

    # Input a wag rate between 30.0 and 7.0. Lower rates give faster wagging
    def wagging(self, duration, wag_rate):

        start_time = time.time()
        wag_phase = 0.0

        while time.time() - start_time < duration:
            wag_phase += np.pi / wag_rate

            if wag_phase >= 2 * np.pi:
                wag_phase -= 2 * np.pi

            self.position_tail(np.sin(wag_phase)*0.5 + 0.5, 0.0)

            rospy.sleep(0.02)  # Sleep briefly for smooth transition

        # Set the tail to the correct ending position
        self.position_tail(0.5, 0.0)