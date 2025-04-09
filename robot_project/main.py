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
import rospy
import signal
import os
import sys

from actuators.status_handler import status_handler

# State constants
MUSCLE_RELAXATION = 1
AUDIOBOOK = 2
DANCE = 3
BREATHING_EXERCISE = 4
INTERACTIVE_STANDBY = 5

def check_ros_connection():
    """
    Check if the ROS node is connected and the required topics are available.
    Returns True if connected, False otherwise.
    """
    try:
        # Check if ROS master is running
        rospy.get_master().getSystemState()
        return True
    except rospy.ROSException:
        return False

def signal_handler(sig, frame):
    """
    Handle termination signals (e.g., Ctrl+C) to ensure the GUI shuts down.
    """
    print("Terminating program...")
    command_queue.put({"type": "shutdown"})  # Send shutdown signal to GUI
    os._exit(0)  # Exit the program

def main():
    global command_queue  # Make command_queue accessible in the signal handler
    command_queue = queue.Queue()

    # Set the command queue in the status handler
    status_handler.set_command_queue(command_queue)

    gui_thread = threading.Thread(target=start_gui, args=(command_queue,))
    gui_thread.daemon = True
    gui_thread.start()

    # Register the signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    # Register the signal handler for SIGTSTP (Ctrl+Z)
    signal.signal(signal.SIGTSTP, signal_handler)

    # Check ROS connection status
    ros_connected = check_ros_connection()
    print(f"ROS Connected: {ros_connected}")

    # Pass the connection status to the GUI via the command queue
    command_queue.put({"type": "connection_status", "connected": ros_connected})

    node_actuators = NodeActuators()  # Initialize NodeActuators instance
    reset_behaviour = ResetBehavior()
    reset_behaviour.run()
    interactive_standby_behaviour = interactive_standby()

    user_input = INTERACTIVE_STANDBY  # Default starting value
    sub_user_input=0

    try:
        while True:
            behavior_name = None  # Initialize behavior name for GUI updates

            if user_input == 0:
                print("Exiting program...")
                # Send a shutdown signal to the GUI
                command_queue.put({"type": "shutdown"})
                gui_thread.join()  # Wait for the GUI thread to finish
                sys.exit(0)  # Exit the program
                break  # Exit the loop if user chooses 0
            elif user_input == 1:
                behavior_name = "Muscle Relaxation"
                command_queue.put({"type": "behavior_update", "behavior_name": behavior_name})
                print("1) Muscle relaxation")
                relax_behaviour = RelaxBehavior()
                if sub_user_input== 0:
                    relax_behaviour.run()
                if sub_user_input== 1:
                    relax_behaviour.full_relaxation()
                if sub_user_input== 2:
                    relax_behaviour.relax_arms()
                if sub_user_input== 3:
                    relax_behaviour.relax_back()
                if sub_user_input== 4:
                    relax_behaviour.relax_tummy()
                if sub_user_input== 5:
                    relax_behaviour.relax_legs()
                #reset_behaviour.run()
                print("Muscle relaxation ended.")
                user_input = INTERACTIVE_STANDBY
                
            elif user_input == 2:
                behavior_name = "Audiobook"
                command_queue.put({"type": "behavior_update", "behavior_name": behavior_name})
                audiobooks_behaviour = AudiobooksBehavior()
                print("2) Audiobook")
                if sub_user_input== 0:
                    audiobooks_behaviour.run()
                if sub_user_input== 1:
                    audiobooks_behaviour.book1()
                if sub_user_input== 2:
                    audiobooks_behaviour.book2()
                #reset_behaviour.run()
                print("Audiobook ended.")
                user_input = INTERACTIVE_STANDBY
                
            elif user_input == 3:
                behavior_name = "Dance"
                command_queue.put({"type": "behavior_update", "behavior_name": behavior_name})
                print("3) Dance")
                dance_behaviour = DanceBehavior()
                dance_behaviour.run()
                #reset_behaviour.run()
                user_input = INTERACTIVE_STANDBY
                print("Dance ended.")
            elif user_input == 4:
                behavior_name = "Breathing Exercise"
                command_queue.put({"type": "behavior_update", "behavior_name": behavior_name})
                print("4) Breathing exercise")
                breath_ex_behaviour = breath_ex()
                breath_ex_behaviour.run()
                user_input = INTERACTIVE_STANDBY
                print("Breathing exercise ended.")
            elif user_input == 5:
                behavior_name = "Interactive Standby"
                command_queue.put({"type": "behavior_update", "behavior_name": behavior_name})
                print("5) Interactive standby")
                interactive_standby_behaviour = interactive_standby()
                interactive_standby_behaviour.loop()
                user_input = interactive_standby_behaviour.behaviour
                sub_user_input= interactive_standby_behaviour.sub_behaviour
                print("Interactive standby ended.")
            else:
                print("Invalid choice or unrecognized number.")

    finally:
        # Send a shutdown signal to the GUI
        command_queue.put({"type": "shutdown"})
        gui_thread.join()  # Wait for the GUI thread to finish
        sys.exit(0)


if __name__ == "__main__":
    main()