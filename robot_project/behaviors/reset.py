#!/usr/bin/python3

# ----------------------------------------------
# Title: reset.py
# Description: Reset actuators to default positions
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------



import threading
import time
from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class

from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class

from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class

from actuators.play_audio import AudioPlayer  # function


class ResetBehavior:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()

    def run(self):
        print("Resetting actuators to default positions")
        print(f"Thread count: {threading.active_count()}")

        led_thread = threading.Thread(target=self.led_controller.turn_off_led, args=())
        led_thread.start()
        
        
        eyes_thread = threading.Thread(target=self.cosmetics_controller.move_eyes, args=(2, 0))
        eyes_thread.start()  

        ears_thread = threading.Thread(target=self.cosmetics_controller.move_ears, args=(2, 0))
        ears_thread.start()
        
        #tail_thread = threading.Thread(target=self.cosmetics_controller.move_wag_tail, args=(2, 0))
        #tail_thread.start()
        
        head_thread = threading.Thread(target=self.joints_controller.move_all, args=(2, 0,-10,20))
        head_thread.start()
        
        
        """
        head_pitch_thread = threading.Thread(target=self.joints_controller.move_pitch, args=(2, 0))
        head_pitch_thread.start()
        head_pitch_thread.join()
        
        neck_thread = threading.Thread(target=self.joints_controller.move_neck, args=(2, 20))
        neck_thread.start()
        neck_thread.join()        
        """
        led_thread.join()
        eyes_thread.join() 
        ears_thread.join()
        head_thread.join()
        #tail_thread.join()
        print("Reset complete")
        print(f"Thread count: {threading.active_count()}")
        
          
             
        
        
        

        
        

        
        
        




    

