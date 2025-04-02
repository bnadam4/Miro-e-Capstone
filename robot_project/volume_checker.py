#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: March 26, 2025
# Last modified: March 28, 2025
#
# Checks the volume of all mp3 files in a folder using ffmpeg.
# Written by ChatGPT

import os
import subprocess
import re

# Input folders
unamplified_folder = "mp3_files_unamplified"
amplified_4x_folder = "mp3_files_4x"
final_output_folder = "mp3_files"

# Regex patterns to extract volume info from ffmpeg output
mean_volume_pattern = re.compile(r"mean_volume: (-?\d+\.\d+) dB")
max_volume_pattern = re.compile(r"max_volume: (-?\d+\.\d+) dB")

def get_audio_volume(file_path):
    """Runs ffmpeg volumedetect and returns (mean_volume, max_volume) in dB."""
    command = [
        "ffmpeg",
        "-i", file_path,
        "-af", "volumedetect",
        "-f", "null",
        "-"
    ]

    result = subprocess.run(command, stderr=subprocess.PIPE, text=True)

    mean_volume_match = mean_volume_pattern.search(result.stderr)
    max_volume_match = max_volume_pattern.search(result.stderr)

    if mean_volume_match and max_volume_match:
        mean_volume = float(mean_volume_match.group(1))
        max_volume = float(max_volume_match.group(1))
        return mean_volume, max_volume
    else:
        return None, None

# Process each file in the unamplified folder
for filename in os.listdir(unamplified_folder):
    if filename.lower().endswith(".mp3"):
        unamplified_path = os.path.join(unamplified_folder, filename)
        amplified_4x_path = os.path.join(amplified_4x_folder, filename)
        final_output_path = os.path.join(final_output_folder, filename)

        print(f"Analyzing {filename}...")

        # Get volumes for unamplified file
        unamplified_mean, unamplified_max = get_audio_volume(unamplified_path)
        if unamplified_mean is None:
            print(f"  ❌ Could not analyze {filename} in unamplified folder.\n")
            continue

        print(f"  Unamplified  → Mean: {unamplified_mean:.2f} dB | Max: {unamplified_max:.2f} dB")

        # Get volumes for 4x amplified file (if it exists)
        if os.path.exists(amplified_4x_path):
            amplified_4x_mean, amplified_4x_max = get_audio_volume(amplified_4x_path)
            if amplified_4x_mean is None:
                print(f"  ❌ Could not analyze {filename} in 4x amplified folder.\n")
            else:
                mean_diff_4x = amplified_4x_mean - unamplified_mean
                max_diff_4x = amplified_4x_max - unamplified_max
                print(f"  4x Amplified → Mean: {amplified_4x_mean:.2f} dB | Max: {amplified_4x_max:.2f} dB")
                #print(f"  🔍 Increase  → Mean: {mean_diff_4x:.2f} dB | Max: {max_diff_4x:.2f} dB")

        else:
            print(f"  ⚠️ No 4x amplified version found for {filename}.")

        # Get volumes for final output file (if it exists)
        if os.path.exists(final_output_path):
            final_mean, final_max = get_audio_volume(final_output_path)
            if final_mean is None:
                print(f"  ❌ Could not analyze {filename} in final output folder.\n")
            else:
                mean_diff_final = final_mean - unamplified_mean
                max_diff_final = final_max - unamplified_max
                print(f"  Final Output → Mean: {final_mean:.2f} dB | Max: {final_max:.2f} dB")
                #print(f"  🔍 Increase  → Mean: {mean_diff_final:.2f} dB | Max: {max_diff_final:.2f} dB\n")

        else:
            print(f"  ⚠️ No final output version found for {filename}.\n")

print("Analysis complete.")


