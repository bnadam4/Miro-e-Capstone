#!/usr/bin/python3

# ----------------------------------------------
# Title: main.py
# Description: A brief overview of what this script does
# Author: Jasmine, Bryce
# Date created: Dec 24, 2024
# Date modified: Jan 8, 2024
# ----------------------------------------------

from behaviors.dance import DanceBehavior
from behaviors.audiobooks import AudiobooksBehavior
from behaviors.test import TestBehavior
from behaviors.reset import ResetBehavior
from behaviors.breath_ex import breath_ex
from behaviors.interactive_standby import interactive_standby
from actuators.node_actuators import NodeActuators  # assuming this import for NodeActuators
# from actuators import node_actuators

# State constants
BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

def main():
    node_actuators = NodeActuators()  # Initialize NodeActuators instance
    reset_behaviour = ResetBehavior()
    reset_behaviour.run()
    interactive_standby_behaviour = interactive_standby()

    user_input = int(input("Enter a number: \n0= exit \n1= test \n2= dance \n3= audiobook \n4= breathing exercise \n5= interactive standby\n"))
    
    while True:  # Infinite loop that will continue until the user exits
        # Taking input from the user. For debugging only

        if user_input == 0:
            print("Exiting program...")
            break  # Exit the loop if user chooses 0
        elif user_input == 1:
            print("1) test")
            test_behaviour = TestBehavior()
            test_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
        elif user_input == 2:
            print("2) dance")
            dance_behaviour = DanceBehavior()
            dance_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
        elif user_input == 3:
            print("3) audiobook")
            audiobooks_behaviour = AudiobooksBehavior()
            audiobooks_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
        elif user_input == 4:
            print("4) breathing exercise")
            breath_ex_behaviour = breath_ex()
            breath_ex_behaviour.run()
            user_input = INTERACTIVE_STANDBY
        elif user_input == 5:
            print("5) interactice standby")
            interactive_standby_behaviour = interactive_standby()
            interactive_standby_behaviour.loop()
            user_input = interactive_standby_behaviour.behaviour
            print("Interactive standby ended.")
        else:
            print("Invalid choice or unrecognized number.")

if __name__ == "__main__":
    main()