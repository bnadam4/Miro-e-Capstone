import tkinter as tk
from actuators.audio_history import AudioHistory

class BaseView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.configure(width=400, height=400)
        self.pack_propagate(False)  # Prevent resizing to fit child widgets
        self.create_widgets()

    def create_widgets(self):
        self.label = tk.Label(self, text="Miro GUI", font=("Arial", 16))
        self.label.place(relx=0.5, rely=0.3, anchor=tk.CENTER)

        # Add a label for connection status
        self.connection_status_label = tk.Label(self, text="Connecting...", font=("Arial", 12))
        self.connection_status_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # Add a label for the current behavior
        self.behavior_label = tk.Label(self, text="Current Behavior: None", font=("Arial", 12))
        self.behavior_label.place(relx=0.5, rely=0.7, anchor=tk.CENTER)

        # Add a label for the last played audio files
        self.audio_history_label = tk.Label(self, text="Last Played Audio:\nNone", font=("Arial", 10), justify=tk.LEFT)
        self.audio_history_label.place(relx=0.5, rely=0.9, anchor=tk.CENTER)


    def update_connection_status(self, connected):
        """
        Update the connection status label.
        """
        status_text = "Connected to Robot" if connected else "Not Connected"
        self.connection_status_label.config(text=status_text)

    def update_behavior(self, behavior_name):
        """
        Update the behavior label with the current behavior name.
        """
        self.behavior_label.config(text=f"Current Behavior: {behavior_name}")

    def update_audio_history(self):
        """
        Update the audio history label with the last played audio files.
        """
        history = AudioHistory().get_history()
        history_text = "Last Played Audio:\n" + "\n".join(history) if history else "Last Played Audio:\nNone"
        self.audio_history_label.config(text=history_text)

    def shutdown(self):
        """
        Perform cleanup actions before shutting down the application.
        """
        # Example: Print a message to the console
        print("Shutting down the application...")

        # Destroy the tkinter window
        self.master.destroy()