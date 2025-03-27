#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: March 23, 2025
# Last modified: March 23, 2025
#
# Amplifies the volume of all mp3 files in a folder using ffmpeg.
# The original files are replaced with the amplified versions.
# Written by ChatGPT

import os
import subprocess

# Input and output folder paths
input_folder = "mp3_files_unamplified"
output_folder = "mp3_files"

# Volume multiplier
volume_multiplier = "2.0"

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Iterate through all mp3 files in the input folder
for filename in os.listdir(input_folder):
    if filename.lower().endswith(".mp3"):
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename)

        # ffmpeg command to increase volume
        command = [
            "ffmpeg",
            "-i", input_path,
            "-filter:a", f"volume={volume_multiplier}",
            "-y",  # Overwrite output file if it exists
            output_path
        ]

        print(f"Amplifying {filename}...")
        subprocess.run(command, check=True)
        print(f"Saved amplified file to {output_path}\n")

print("All files processed.")