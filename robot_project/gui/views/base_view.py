import tkinter as tk
from PIL import Image, ImageTk  # Import for handling images
from actuators.audio_history import AudioHistory
from actuators.speech_history import SpeechHistory  # Import the SpeechHistory class

class BaseView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.configure(width=800, height=800)  # Set default size to 800x800
        self.pack_propagate(False)  # Prevent resizing to fit child widgets

        # Define reusable constants
        self.frame_relwidth = 0.9
        self.frame_relheight = 0.2
        self.label_wrap_factor = 0.80
        self.padding_x = 0.02
        self.padding_y = 0.05

        self.create_widgets()

    def create_widgets(self):
        # Add an image to the top left
        self.image = Image.open("images/miro.png")  # Load the image
        self.image = self.image.resize((100, 100), Image.Resampling.LANCZOS)  # Resize the image
        self.photo = ImageTk.PhotoImage(self.image)
        self.image_label = tk.Label(self, image=self.photo)
        self.image_label.place(relx=0.01, rely=0.01, anchor=tk.NW)  # Relative position in the top left corner

        # Add a label for connection status in the top right
        self.connection_status_label = tk.Label(self, text="Connecting...", font=("Arial", 12))
        self.connection_status_label.place(relx=0.95, rely=0.02, anchor=tk.NE)  # Relative position in the top right corner

        # Add an image for the current behavior in the middle
        self.behavior_image_label = tk.Label(self)
        self.behavior_image_label.place(relx=0.5, rely=0.1, anchor=tk.CENTER)  # Centered horizontally

        # Add a label for the current behavior in the middle
        self.behavior_label = tk.Label(self, text="Current Behavior: None", font=("Arial", 14))
        self.behavior_label.place(relx=0.5, rely=0.2, anchor=tk.CENTER)  # Centered horizontally

        # Create a frame for the "Last Played Audio" section
        self.audio_frame = tk.Frame(self, relief=tk.SOLID, borderwidth=1)
        self.audio_frame.place(relx=0.5, rely=0.44, relwidth=self.frame_relwidth, relheight=0.27, anchor=tk.CENTER)  # Moved up and increased height
        self.audio_title = tk.Label(self.audio_frame, text="Last Played Audio", font=("Arial", 12), anchor="w")
        self.audio_title.place(relx=self.padding_x, rely=self.padding_y, anchor=tk.NW)
        self.audio_history_label = tk.Label(
            self.audio_frame,
            text="None",
            font=("Arial", 10),
            justify=tk.LEFT,
            wraplength=int(self.label_wrap_factor * self.audio_frame.winfo_width())  # Dynamically set wraplength
        )
        self.audio_history_label.place(relx=self.padding_x, rely=0.3, anchor=tk.NW)

        # Create a frame for the "Last Speech Texts" section
        self.speech_frame = tk.Frame(self, relief=tk.SOLID, borderwidth=1)
        self.speech_frame.place(relx=0.5, rely=0.69, relwidth=self.frame_relwidth, relheight=0.27, anchor=tk.CENTER)  # Moved up and increased height
        self.speech_title = tk.Label(self.speech_frame, text="Last Speech Texts", font=("Arial", 12), anchor="w")
        self.speech_title.place(relx=self.padding_x, rely=self.padding_y, anchor=tk.NW)
        self.speech_history_label = tk.Label(
            self.speech_frame,
            text="None",
            font=("Arial", 10),
            justify=tk.LEFT,
            wraplength=int(self.label_wrap_factor * self.speech_frame.winfo_width())  # Dynamically set wraplength
        )
        self.speech_history_label.place(relx=self.padding_x, rely=0.3, anchor=tk.NW)

        # Add a status label at the bottom of the GUI
        self.status_label = tk.Label(self, text="Status: Standby", font=("Arial", 12), anchor="w")
        self.status_label.place(relx=0.5, rely=0.95, anchor=tk.CENTER)  # Centered at the bottom

        # Add a battery image label in the top right
        self.battery_image_label = tk.Label(self)
        self.battery_image_label.place(relx=0.95, rely=0.1, anchor=tk.NE)  # Positioned at the top right

        # Add a battery voltage label below the battery image label
        self.battery_label = tk.Label(self, text="Battery Voltage: -- V", font=("Arial", 12))
        self.battery_label.place(relx=0.95, rely=0.2, anchor=tk.NE)  # Positioned below the battery image label

        # Dynamically update wraplength after the GUI is rendered
        self.after(100, self.update_wraplengths)

    def update_wraplengths(self):
        """
        Dynamically update the wraplength of labels to match the frame width.
        """
        audio_frame_width = self.audio_frame.winfo_width()
        speech_frame_width = self.speech_frame.winfo_width()
        self.audio_history_label.config(wraplength=int(self.label_wrap_factor * audio_frame_width))
        self.speech_history_label.config(wraplength=int(self.label_wrap_factor * speech_frame_width))

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
            "Dance": "images/dance.png",
            "Breathing Exercise": "images/breathing_exercise.png",
            "Shutdown": "images/shutdown.png",
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
        history_text = "\n".join(history) if history else "None"
        self.audio_history_label.config(text=history_text)

    def update_speech_history(self):
        """
        Update the speech history label with the last received texts.
        """
        history = SpeechHistory().get_history()
        # Extract only the text from each tuple
        history_texts = [item[0] for item in history]  # Assuming each tuple is (text, confidence)
        history_text = "\n".join(history_texts) if history_texts else "None"
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

    def update_battery_voltage(self, voltage):
        """
        Update the battery voltage label and display the corresponding battery image.
        """
        self.battery_label.config(text=f"{voltage:.2f} V")  # Update the voltage text

        # Determine the battery image based on the voltage
        if voltage > 5.3:
            battery_image_path = "images/battery_full.png"
        elif voltage > 5.1:
            battery_image_path = "images/battery_medium.png"
        else:
            battery_image_path = "images/battery_low.png"

        # Load and display the battery image
        battery_image = Image.open(battery_image_path)
        battery_image = battery_image.resize((50, 50), Image.Resampling.LANCZOS)  # Resize the image
        self.battery_photo = ImageTk.PhotoImage(battery_image)
        self.battery_image_label.config(image=self.battery_photo)


