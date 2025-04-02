import tkinter as tk
from PIL import Image, ImageTk  # Import for handling images
from actuators.audio_history import AudioHistory
from actuators.speech_history import SpeechHistory  # Import the SpeechHistory class

class BaseView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.configure(width=800, height=800)  # Set default size to 800x800
        self.pack_propagate(False)  # Prevent resizing to fit child widgets
        self.create_widgets()

    def create_widgets(self):
        # Add an image to the top left
        self.image = Image.open("images/miro.png")  # Load the image
        self.image = self.image.resize((100, 100), Image.Resampling.LANCZOS)  # Resize the image
        self.photo = ImageTk.PhotoImage(self.image)
        self.image_label = tk.Label(self, image=self.photo)
        self.image_label.place(x=10, y=10)  # Place the image in the top left corner

        # Add a label for connection status in the top right
        self.connection_status_label = tk.Label(self, text="Connecting...", font=("Arial", 12))
        self.connection_status_label.place(x=650, y=20)  # Place the label in the top right corner

        # Add an image for the current behavior in the middle
        self.behavior_image_label = tk.Label(self)
        self.behavior_image_label.place(relx=0.5, rely=0.1, anchor=tk.CENTER)

        # Add a label for the current behavior in the middle
        self.behavior_label = tk.Label(self, text="Current Behavior: None", font=("Arial", 14))
        self.behavior_label.place(relx=0.5, rely=0.2, anchor=tk.CENTER)

        # Create a frame for the "Last Played Audio" section
        self.audio_frame = tk.Frame(self, width=400, height=150, relief=tk.SOLID, borderwidth=1)
        self.audio_frame.place(relx=0.5, rely=0.6, anchor=tk.CENTER)  # Center the frame
        self.audio_title = tk.Label(self.audio_frame, text="Last Played Audio", font=("Arial", 12), anchor="w")
        self.audio_title.place(x=5, y=5)  # Place the title at the top left of the frame
        self.audio_history_label = tk.Label(self.audio_frame, text="None", font=("Arial", 10), justify=tk.LEFT, wraplength=580)
        self.audio_history_label.place(x=5, y=30)  # Place the content inside the frame

        # Create a frame for the "Last Speech Texts" section
        self.speech_frame = tk.Frame(self, width=400, height=150, relief=tk.SOLID, borderwidth=1)
        self.speech_frame.place(relx=0.5, rely=0.8, anchor=tk.CENTER)  # Center the frame
        self.speech_title = tk.Label(self.speech_frame, text="Last Speech Texts", font=("Arial", 12), anchor="w")
        self.speech_title.place(x=5, y=5)  # Place the title at the top left of the frame
        self.speech_history_label = tk.Label(self.speech_frame, text="None", font=("Arial", 10), justify=tk.LEFT, wraplength=580)
        self.speech_history_label.place(x=5, y=30)  # Place the content inside the frame

        # Add a status label at the bottom of the GUI
        self.status_label = tk.Label(self, text="Status: Standby", font=("Arial", 12), anchor="w")
        self.status_label.place(relx=0.5, rely=0.9, anchor=tk.CENTER)  # Place it at the bottom center

    def update_connection_status(self, connected):
        """
        Update the connection status label.
        """
        status_text = "Connected to Robot" if connected else "Not Connected"
        self.connection_status_label.config(text=status_text)

    def update_behavior(self, behavior_name):
        """
        Update the behavior label with the current behavior name and display the respective image.
        """
        # Define a mapping of behavior names to image paths
        behavior_images = {
            "Muscle Relaxation": "images/muscle_relaxation.png",
            "Audiobook": "images/audiobook.png",
            "Dance": "images/checklist.png",
            "Breathing Exercise": "images/breathing_exercise.png",
            # Add more behaviors and their corresponding images here
        }
        behavior_image_path = behavior_images.get(behavior_name, None)  # Get the image path or None if not found
        self.behavior_label.config(text=f"Current Behavior: {behavior_name}")
        if behavior_image_path:
            behavior_image = Image.open(behavior_image_path)
            behavior_image = behavior_image.resize((100, 100), Image.Resampling.LANCZOS)  # Resize the image

            self.behavior_photo = ImageTk.PhotoImage(behavior_image)
            self.behavior_image_label.config(image=self.behavior_photo)
        else:
            self.behavior_image_label.config(image="")  # Clear the image if no path is provided

    def update_audio_history(self):
        """
        Update the audio history label with the last played audio files.
        """
        history = AudioHistory().get_history()
        history_text = "Last Played Audio:\n" + "\n".join(history) if history else "Last Played Audio:\nNone"
        self.audio_history_label.config(text=history_text)

    def update_speech_history(self):
        """
        Update the speech history label with the last received texts.
        """
        history = SpeechHistory().get_history()
        # Extract only the text from each tuple
        history_texts = [item[0] for item in history]  # Assuming each tuple is (text, confidence)
        history_text = "Last Speech Texts:\n" + "\n".join(history_texts) if history_texts else "Last Speech Texts:\nNone"
        self.speech_history_label.config(text=history_text)

    def update_status(self, status):
        """
        Update the status label with the current status.
        """
        self.status_label.config(text=f"Status: {status}")

    def shutdown(self):
        """
        Perform cleanup actions before shutting down the application.
        """
        # Example: Print a message to the console
        print("Shutting down the application...")

        # Destroy the tkinter window
        self.master.destroy()