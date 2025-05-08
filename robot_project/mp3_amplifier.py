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
input_folder = "mp3_files_slushy"
output_folder = "mp3_files_slushy_4x"

# Volume multiplier
volume_multiplier = "4.0"

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Walk through input folder and all subfolders
for root, _, files in os.walk(input_folder):
    for filename in files:
        if filename.lower().endswith(".mp3"):
            input_path = os.path.join(root, filename)

            # Determine relative path to maintain subfolder structure
            relative_path = os.path.relpath(input_path, input_folder)
            output_subfolder = os.path.join(output_folder, os.path.dirname(relative_path))
            output_path = os.path.join(output_subfolder, filename)

            # Create output subfolder if it doesn't exist
            os.makedirs(output_subfolder, exist_ok=True)

            # ffmpeg command to increase volume
            command = [
                "ffmpeg",
                "-i", input_path,
                "-filter:a", f"volume={volume_multiplier}",
                "-y",  # Overwrite output file if it exists
                output_path
            ]

            print(f"Amplifying {input_path}...")
            subprocess.run(command, check=True)
            print(f"Saved amplified file to {output_path}\n")

print("All files processed.")
