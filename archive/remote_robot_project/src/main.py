#!/usr/bin/python3

# ----------------------------------------------
# Title: main.py
# Description: A brief overview of what this script does
# Author: Jasmine, Bryce
# Date created: Dec 24, 2024
# Date modified: Feb 25, 2024
# ----------------------------------------------

# from actuators import node_actuators
from actuators.node_actuators import NodeActuators
from behaviors.reset import ResetBehavior

from behaviors.audiobooks import AudiobooksBehavior
from behaviors.breath_ex import breath_ex
from behaviors.relax import RelaxBehavior
from behaviors.interactive_standby import interactive_standby

from behaviors.test import TestBehavior
from behaviors.dance import DanceBehavior

from com_esp32 import get_data, send_data  # Import functions from com-esp32.py


# State constants
INTERACTIVE_STANDBY = 1
BREATHING_EXERCISE = 2
MUSCLE_RELAXATION = 3
AUDIOBOOK = 4
CHECKLIST = 5


import time


def main():
    node_actuators = NodeActuators()  # Initialize NodeActuators instance
    reset_behaviour = ResetBehavior()
    #interactive_standby_behaviour = interactive_standby()
    audiobooks_behaviour = AudiobooksBehavior()
    relax_behaviour = RelaxBehavior()
    
    user_input = INTERACTIVE_STANDBY
    behavior_running = False
    previous_data = None

    while True:  # Infinite loop that will continue until the user exits
        # Fetch new data from the ESP32
        new_data = get_data()

        # Check if the new data is different from the previous data
        if new_data != previous_data or behavior_running:
            print(f"Data changed! Last data: {previous_data}, Current data: {new_data}")
            print("New data received!")
            # Update the previous data to the new data
            previous_data = new_data
            if behavior_running:
                behavior_running = False
                user_input = INTERACTIVE_STANDBY
            else:
                if new_data:
                    # Extract the relevant data values as integers (assuming data is a dict)
                    user_input = new_data['data1']
                    data2 = new_data['data2']
                    data3 = new_data['data3']
                    data4 = new_data['data4']
                    data5 = new_data['data5']

            # Start processing based on user_input
            if user_input == 0:
                print("Exiting program...")
                reset_behaviour.run()
                break  # Exit the loop if user chooses 0
            if user_input == MUSCLE_RELAXATION:
                print("1) Muscle relaxation")

                if data5 == 3:
                    print("Arms relax")
                    relax_behaviour.relax_arms_solo()  # Play arms
                elif data5 == 4:
                    print("Legs relax")
                    relax_behaviour.relax_legs_solo()  # Play legs
                elif data5 == 6:
                    print("Tummy relax")
                    relax_behaviour.relax_tummy_solo()  # Play tummy
                elif data5 == 5:
                    print("Back relax")
                    relax_behaviour.relax_back_solo()  # Play back
                
                elif data5 == 2:
                    print("Exit behaviour")
                    behavior_running = True
                elif data5 == 1:
                    relax_behaviour.intro()  # Start the relax intro
                
            elif user_input == AUDIOBOOK:
                print("2) audiobook")

                if data5 == 5:
                    print("Rumpelstiltskin")
                    audiobooks_behaviour.book1_solo()  # Play Rumpelstiltskin
                elif data5 == 4:
                    print("The emperor's new clothes")
                    audiobooks_behaviour.book2_solo()  # Play emperor
                elif data5 == 3:
                    print("Aladdin")
                    audiobooks_behaviour.book3_solo()  # Play aladdin
                elif data5 == 2:
                    print("Exit behaviour")
                    behavior_running = True
                elif data5 == 1:
                    audiobooks_behaviour.intro()  # Start the audiobook intro

            elif user_input == CHECKLIST:  # dance
                print("3) dance")
                dance_behaviour = DanceBehavior()
                dance_behaviour.run()

            
            elif user_input == BREATHING_EXERCISE:
                print("4) breathing exercise")
                breath_ex_behaviour = breath_ex()
                breath_ex_behaviour.run()
                if data5 == 2:
                    print("Exit behaviour")
                    behavior_running = True
            

            elif user_input == INTERACTIVE_STANDBY:
                print("5) interactive standby")
                interactive_standby_behaviour = interactive_standby()
                interactive_standby_behaviour.loop()
                user_input = interactive_standby_behaviour.behaviour
                print("Interactive standby ended.")
            
            else:
                print("Invalid choice or unrecognized number.")

        else:
            print("No new data received.")

if __name__ == "__main__":
    main()
