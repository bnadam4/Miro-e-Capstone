#!/usr/bin/python3

# ----------------------------------------------
# Title: test.py
# Description: Test files before implemetation
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

from actuators.play_audio import play_audio  # function


class TestBehavior:
    def __init__(self):
        # Initialize all actuators
        
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()

    def run(self):
        # Run ear movement from 0-10s

        ear_thread_1 = threading.Thread(target=self.joints_movement.shake, args=(2,2))
        ear_thread_1.start()
        
        """
        time.sleep(2)
        ear_thread_2 = threading.Thread(target=self.joints_controller.move_yaw, args=(3,30)) 
        ear_thread_2.start()
        """
        ear_thread_1.join()
        #ear_thread_2.join()
        
        

        
        
        




    

