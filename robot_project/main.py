#!/usr/bin/python3

# ----------------------------------------------
# Title: main.py
# Description: A brief overview of what this script does
# Author: Jasmine, Bryce
# Date created: Dec 24, 2024
# Date modified: Feb 25, 2024
# ----------------------------------------------

import threading
import queue
from gui.main_gui import start_gui
from behaviors.dance import DanceBehavior
from behaviors.audiobooks import AudiobooksBehavior
from behaviors.test import TestBehavior
from behaviors.reset import ResetBehavior
from behaviors.breath_ex import breath_ex
from behaviors.relax import RelaxBehavior
from behaviors.interactive_standby import interactive_standby
from actuators.node_actuators import NodeActuators  # assuming this import for NodeActuators

# State constants
MUSCLE_RELAXATION = 1
AUDIOBOOK = 2
DANCE = 3
BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

def main():
    command_queue = queue.Queue()
    gui_thread = threading.Thread(target=start_gui, args=(command_queue,))
    gui_thread.start()

    node_actuators = NodeActuators()  # Initialize NodeActuators instance
    reset_behaviour = ResetBehavior()
    reset_behaviour.run()
    interactive_standby_behaviour = interactive_standby()

    user_input = INTERACTIVE_STANDBY  # Default starting value

    while True:
        if not command_queue.empty():
            user_input = command_queue.get()

        if user_input == 0:
            print("Exiting program...")
            break  # Exit the loop if user chooses 0
        elif user_input == 1:
            print("1) Muscle relaxation")
            test_behaviour = RelaxBehavior()
            test_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
            print("Muscle relaxation ended.")
        elif user_input == 2:
            print("2) audiobook")
            audiobooks_behaviour = AudiobooksBehavior()
            audiobooks_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
            print("Audiobook ended.")
        elif user_input == 3:
            print("3) dance")
            dance_behaviour = DanceBehavior()
            dance_behaviour.run()
            reset_behaviour.run()
            user_input = INTERACTIVE_STANDBY
            print("Dance ended.")
        elif user_input == 4:
            print("4) breathing exercise")
            breath_ex_behaviour = breath_ex()
            breath_ex_behaviour.run()
            user_input = INTERACTIVE_STANDBY
            print("Breathing exercise ended.")
        elif user_input == 5:
            print("5) interactive standby")
            interactive_standby_behaviour = interactive_standby()
            interactive_standby_behaviour.loop()
            user_input = interactive_standby_behaviour.behaviour
            print("Interactive standby ended.")
        else:
            print("Invalid choice or unrecognized number.")

if __name__ == "__main__":
    main()