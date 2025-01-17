#!/usr/bin/python3

# ----------------------------------------------
# Title: cosmetics_movement.py
# Description: Controls various cosmetic movements for robot (ears, eyes, tail)
# Author:  Jasmine
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------

import rospy
from actuators.cosmetics_controller import CosmeticsController

class CosmeticsMovement:
    def __init__(self):
        # Initialize the CosmeticsController to control the robot's cosmetic features
        self.controller = CosmeticsController()

    def ears_facing_front(self, duration):
        """Moves both ears to face the front using smooth movement."""
        rospy.loginfo("Moving ears to face the front.")
        self.controller.move_ears(duration, 0.4)  # Ears facing forward (default position)

    def ear_outwards(self, duration):
        """Moves both ears outwards using smooth movement."""
        rospy.loginfo("Moving ears outwards.")
        self.controller.move_ears(duration, 1.0)  # Ears fully outwards

    def ears_inwards(self, duration):
        """Moves both ears inwards using smooth movement."""
        rospy.loginfo("Moving ears inwards.")
        self.controller.move_ears(duration, 0)  # Ears fully inwards

    def ears_facing_same_direction(self, duration, side):
        """Moves both ears to face the same direction using smooth movement."""
        rospy.loginfo("Moving ears to face the same direction.")
        if side=="right":
            self.controller.move_one_ear(duration, 0, "left") 
            self.controller.move_one_ear(duration, 1, "right")
        else:
            self.controller.move_one_ear(duration, 1, "left") 
            self.controller.move_one_ear(duration, 0, "right")

    def eye_wink(self, duration, repetitions, side):
        """Performs a wink on either the left or right eye, with optional repetition."""
        rospy.loginfo(f"Winking the {side} eye {repetitions} times.")
        for _ in range(repetitions):
            if side == "left":
                self.controller.move_one_eye(duration/2, 1.0, "left")  # Close left eye
                rospy.sleep(0.05)  # Pause before repeating
                self.controller.move_one_eye(duration/2, 0.0, "left")  # Open left eye
            elif side == "right":
                self.controller.move_one_eye(duration/2, 1.0, "right")  # Close right eye
                rospy.sleep(0.05)  # Pause before repeating
                self.controller.move_one_eye(duration/2, 0.0, "right")  # Open right eye
            

    def eyes_squint(self, duration):
        """Moves both eyes to a squinting position using smooth movement."""
        self.controller.move_eyes(duration, 0.4)  # Squint both eyes

    def open_eyes(self, duration):
        """Opens both eyes to their full position using smooth movement."""
        rospy.loginfo("Opening both eyes.")
        self.controller.move_eyes(duration, 1.0)  # Open both eyes fully

    def close_eyes(self, duration=1.0):
        """Closes both eyes using smooth movement."""
        rospy.loginfo("Closing both eyes.")
        self.controller.move_eyes(duration, 0.0)  # Close both eyes

    def wagging_tail(self, duration, repetitions):
        """Performs a wagging motion for the tail with optional repetition."""
        rospy.loginfo(f"Wagging tail {repetitions} times for {duration} seconds each.")
        for _ in range(repetitions):
            self.controller.move_wag_tail(duration, 1.0)  # Wag the tail fully
            self.controller.move_wag_tail(duration, 0.0)  # Return to neutral position



