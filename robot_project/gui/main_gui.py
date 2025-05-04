import tkinter as tk
from queue import Queue, Empty
from gui.views.base_view import BaseView
from actuators.audio_history import AudioHistory

# Robot specific libraries
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, Int16MultiArray, String, UInt16MultiArray
from sensor_msgs.msg import JointState
import miro2 as miro

import os

class MainGUI:
    def __init__(self, root, command_queue):
        self.root = root
        self.command_queue = command_queue
        self.shutdown_flag = False  # Initialize the shutdown flag
        self.create_widgets()
        self.poll_queue()
        self.update_audio_history()
        self.update_speech_history()  # Add periodic speech history updates

        self.voltage = 0
        self.update_battery_voltage()

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
            miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)
        self.input_package = None

    def callback_package(self, msg):
        # store for processing in update_gui
        self.voltage = msg.battery.voltage

    def create_widgets(self):
        self.base_view = BaseView(self.root)
        self.base_view.pack(pady=5)

    def poll_queue(self):
        """
        Poll the command queue for updates and handle them.
        """
        try:
            while True:
                message = self.command_queue.get_nowait()
                if message["type"] == "connection_status":
                    self.base_view.update_connection_status(message["connected"])
                elif message["type"] == "behavior_update":
                    self.base_view.update_behavior(message["behavior_name"])
                elif message["type"] == "status_update":
                    self.base_view.update_status(message["status"])
                elif message["type"] == "shutdown":
                    self.base_view.shutdown()
        except Empty:
            pass
        finally:
            # Schedule the next poll
            self.root.after(100, self.poll_queue)

    def update_audio_history(self):
        """
        Periodically update the audio history in the GUI.
        """
        self.base_view.update_audio_history()
        self.root.after(1000, self.update_audio_history)  # Update every second

    def update_speech_history(self):
        """
        Periodically update the speech history in the GUI.
        """
        self.base_view.update_speech_history()
        self.root.after(1000, self.update_speech_history)  # Update every second

    def update_battery_voltage(self):
        """
        Periodically update the battery voltage in the GUI.
        """
        if self.voltage is not None:
            self.base_view.update_battery_voltage(self.voltage)
        self.root.after(1000, self.update_battery_voltage)  # Update every second

def start_gui(command_queue):
    root = tk.Tk()
    root.title("Robot Control GUI")
    root.geometry("700x500")  # Set the default window size to 800x800
    gui = MainGUI(root, command_queue)
    root.mainloop()