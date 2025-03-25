import tkinter as tk
from tkinter import ttk
import threading
import queue

class MainGUI:
    def __init__(self, root, command_queue):
        self.root = root
        self.command_queue = command_queue
        self.create_widgets()

    def create_widgets(self):
        self.label = ttk.Label(self.root, text="Select an option:")
        self.label.pack(pady=10)

        self.options = ["Exit", "Muscle relaxation", "Audiobook", "Dance", "Breathing exercise", "Interactive standby"]
        self.combobox = ttk.Combobox(self.root, values=self.options)
        self.combobox.pack(pady=10)
        self.combobox.current(5)  # Default to "Interactive standby"

        self.button = ttk.Button(self.root, text="Run", command=self.send_command)
        self.button.pack(pady=10)

    def send_command(self):
        user_input = self.combobox.current()
        self.command_queue.put(user_input)

def start_gui(command_queue):
    root = tk.Tk()
    root.title("Robot Control GUI")
    gui = MainGUI(root, command_queue)
    root.mainloop()