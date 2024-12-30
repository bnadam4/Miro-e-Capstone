#!/usr/bin/python3

# ----------------------------------------------
# Title: main.py
# Description: A brief overview of what this script does
# Author: Jasmine
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------


from behaviors.dance import DanceBehavior
from behaviors.audiobooks import AudiobooksBehavior
from behaviors.test import TestBehavior
from behaviors.reset import ResetBehavior
from actuators.node_actuators import NodeActuators  # assuming this import for NodeActuators

def main():
    node_actuators = NodeActuators()  # Initialize NodeActuators instance
    reset_behavior = ResetBehavior()
    reset_behavior.run()
    
    while True:  # Infinite loop that will continue until the user exits
        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= test \n2= dance \n3= audiobook \n"))
        
        if user_input == 0:
            print("Exiting program...")
            break  # Exit the loop if user chooses 0
        elif user_input == 1:
            print("1) test")
            test_behavior = TestBehavior()
            test_behavior.run()
            reset_behavior.run()
        elif user_input == 2:
            print("2) dance")
            dance_behavior = DanceBehavior()
            dance_behavior.run()
            reset_behavior.run()
        elif user_input == 3:
            print("3) audiobook")
            audiobooks_behavior = AudiobooksBehavior()
            audiobooks_behavior.run()
            reset_behavior.run()
        else:
            print("Invalid choice or unrecognized number.")

if __name__ == "__main__":
    main()
