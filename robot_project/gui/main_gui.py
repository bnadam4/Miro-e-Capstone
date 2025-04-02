import tkinter as tk
from queue import Queue, Empty
from gui.views.base_view import BaseView
from actuators.audio_history import AudioHistory

class MainGUI:
    def __init__(self, root, command_queue):
        self.root = root
        self.command_queue = command_queue
        self.create_widgets()
        self.poll_queue()
        self.update_audio_history()

    def create_widgets(self):
        self.base_view = BaseView(self.root)
        self.base_view.pack(pady=10)

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

def start_gui(command_queue):
    root = tk.Tk()
    root.title("Robot Control GUI")
    root.geometry("400x400")  # Set the default window size to match BaseView
    gui = MainGUI(root, command_queue)
    root.mainloop()