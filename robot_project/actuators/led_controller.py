#!/usr/bin/python3

# ----------------------------------------------
# Title: led_controller.py
# Description: Changes the color of lights
# Author: Kento, Jimmy, Jasmine
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------


import rospy
from std_msgs.msg import UInt32MultiArray
import time
import os

# Generate enum and constants for the LEDS
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)

def generate_illum(r, g, b, bright):
    return (int(bright) << 24) | (r << 16) | (g << 8) | b

class LEDController:
    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        #rospy.sleep(2.0)
        self.pub_illum = rospy.Publisher(topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0)
        self.illum = UInt32MultiArray()
        self.illum.data = [0xFFFFFFFF] * 6
        
        self.current_color=(0,0,0)
        self.current_brightness=250
        self.fade_steps = 100  # Number of steps to fade in and out
        
# ----------------------------------------------
#  Theses functions change all the leds
# ----------------------------------------------       
    
    #turn on leds for certain duration then turns them off
    def toggle_led(self, duration, color, brightness):
        print(f"LED on ({color[0]}, {color[1]}, {color[2]}) with brightness {brightness}...")
        self.current_color=color
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, brightness))
        self.current_brightness=brightness
        start_time = time.time()
        while time.time() - start_time < duration:
            r, g, b = color
            self.illum.data = [generate_illum(int(r), int(g),int(b), int(brightness))] * 6
            self.pub_illum.publish(self.illum)
            rospy.sleep(0.02)
        print("LED off.")
        self.illum.data = [generate_illum(0, 0, 0, 0)] * 6  # LED off
        self.pub_illum.publish(self.illum)
        
    #turn on leds indefinitely
    def turn_on_led(self, color, brightness):
        print(f"LED on ({color[0]}, {color[1]}, {color[2]}) with brightness {brightness}...")
        self.current_color=color
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, brightness))
        self.current_brightness=brightness
        r, g, b = color
        self.illum.data = [generate_illum(int(r), int(g),int(b), int(brightness))] * 6
        self.pub_illum.publish(self.illum)

    #turn off leds indefinitely
    def turn_off_led(self):
        print("LED off.")
        self.illum.data = [generate_illum(0, 0, 0, 0)] * 6  # LED off
        self.pub_illum.publish(self.illum)
    
    #fade in leds indefinitely
    def fade_in_led(self, duration, color, target_brightness):
        print(f"LED on ({color[0]}, {color[1]}, {color[2]}) with brightness {target_brightness}...")
        self.current_color=color
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, target_brightness))
        self.current_brightness=brightness
        r, g, b = color
        start_brightness = 0  # Start at 0 brightness
        step_duration = duration / self.fade_steps  # Duration per step
        for step in range(self.fade_steps + 1):
            calculated_brightness = int(start_brightness + (target_brightness - start_brightness) * (step / self.fade_steps))
            # Ensure brightness stays within bounds
            calculated_brightness = max(0, min(255, calculated_brightness))
            self.current_brightness=calculated_brightness
            self.illum.data = [generate_illum(int(r), int(g),int(b), int(calculated_brightness))] * 6
            self.pub_illum.publish(self.illum)
            rospy.sleep(step_duration)  # Wait for the duration of each step
        print("Fade-in complete.")
        
    #fade out leds indefinitely
    def fade_out_led(self, duration):
        print(f"LED on ({self.current_color[0]}, {self.current_color[1]}, {self.current_color[2]}) with brightness {self.current_brightness}...") 
        r, g, b = self.current_color
        start_brightness = self.current_brightness   # Start at 0 brightness
        end_brightness = 0  # We always fade to 0
        step_duration = duration / self.fade_steps  # Duration per step
        for step in range(self.fade_steps + 1):
            calculated_brightness = int(start_brightness - (start_brightness - end_brightness) * (step / self.fade_steps))
            # Ensure brightness stays within bounds
            calculated_brightness = max(0, min(255, calculated_brightness))
            self.current_brightness=calculated_brightness
            self.illum.data = [generate_illum(int(r), int(g),int(b), int(calculated_brightness))] * 6
            self.pub_illum.publish(self.illum)
            rospy.sleep(step_duration)  # Wait for the duration of each step
        print("Fade-out complete.")
        
# ----------------------------------------------
#  Theses functions change sections the leds (front, mid and rear)
# ---------------------------------------------- 

    #turn on leds for certain duration then turns them off
    def turn_on_led_sections(self, duration, color_front, color_mid, color_rear, brightness):
        print(f"LED on ({color_front[0]}, {color_front[1]}, {color_front[2]})")
        print(f"LED on ({color_mid[0]}, {color_mid[1]}, {color_mid[2]})")
        print(f"LED on ({color_rear[0]}, {color_rear[1]}, {color_rear[2]}) with brightness {brightness}...")
        self.current_color=color_front
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, brightness))
        self.current_brightness=brightness
        start_time = time.time()
        while time.time() - start_time < duration:
            r_f, g_f, b_f = color_front
            r_m, g_m, b_m = color_mid
            r_r, g_r, b_r = color_rear
            
            self.illum.data[front_left] = generate_illum(int(r_f), int(g_f),int(b_f), int(brightness))
            self.illum.data[front_right] = generate_illum(int(r_f), int(g_f),int(b_f), int(brightness))

            self.illum.data[mid_left] = generate_illum(int(r_m), int(g_m),int(b_m), int(brightness))
            self.illum.data[mid_right] = generate_illum(int(r_m), int(g_m),int(b_m), int(brightness))

            self.illum.data[rear_left] = generate_illum(int(r_r), int(g_r),int(b_r), int(brightness))
            self.illum.data[rear_right] = generate_illum(int(r_r), int(g_r),int(b_r), int(brightness))
            
            self.pub_illum.publish(self.illum)
            rospy.sleep(0.02)
        print("LED off.")
        self.illum.data = [generate_illum(0, 0, 0, 0)] * 6  # LED off
        self.pub_illum.publish(self.illum)

    #turn on leds for certain duration then turns them off
    def toggle_led_sections(self, color_front, color_mid, color_rear, brightness):
        self.current_color=color_front
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, brightness))
        self.current_brightness=brightness
        r_f, g_f, b_f = color_front
        r_m, g_m, b_m = color_mid
        r_r, g_r, b_r = color_rear
        
        self.illum.data[front_left] = generate_illum(int(r_f), int(g_f),int(b_f), int(brightness))
        self.illum.data[front_right] = generate_illum(int(r_f), int(g_f),int(b_f), int(brightness))

        self.illum.data[mid_left] = generate_illum(int(r_m), int(g_m),int(b_m), int(brightness))
        self.illum.data[mid_right] = generate_illum(int(r_m), int(g_m),int(b_m), int(brightness))

        self.illum.data[rear_left] = generate_illum(int(r_r), int(g_r),int(b_r), int(brightness))
        self.illum.data[rear_right] = generate_illum(int(r_r), int(g_r),int(b_r), int(brightness))
        
        self.pub_illum.publish(self.illum)
        self.illum.data = [generate_illum(0, 0, 0, 0)] * 6  # LED off
        self.pub_illum.publish(self.illum)

    def wave_led(self, duration, iterations, color, brightness):
        print(f"LED wave ({color[0]}, {color[1]}, {color[2]}) with brightness {brightness} for {iterations} iterations over {duration} seconds...")
        self.current_color = color
        # Ensure brightness is within the 0-255 range
        brightness = max(0, min(255, brightness))
        self.current_brightness = brightness
        r, g, b = color
        
        total_steps = iterations * 3  # Each iteration has 3 steps (front, mid, rear)
        step_duration = duration / total_steps  # Duration per step
        
        for i in range(total_steps):
            # Determine which section to light up
            section = i % 3
            if section == 0:
                self.toggle_led_sections((r, g, b), (0, 0, 0), (0, 0, 0), brightness)
            elif section == 1:
                self.toggle_led_sections((0, 0, 0), (r, g, b), (0, 0, 0), brightness)
            else:
                self.toggle_led_sections((0, 0, 0), (0, 0, 0), (r, g, b), brightness)
            rospy.sleep(step_duration)  # Wait for the duration of each step
        
        print("LED wave complete.")
        self.illum.data = [generate_illum(0, 0, 0, 0)] * 6  # LED off
        self.pub_illum.publish(self.illum)
