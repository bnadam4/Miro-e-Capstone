#!/usr/bin/python3

import os
import sys
import shutil

def error(msg):
    print(msg)
    sys.exit(0)

# Get the current directory where the Python script is located
current_directory = os.path.dirname(os.path.realpath(__file__))

# Get all .mp3 files in the current directory
mp3_files = [f for f in os.listdir(current_directory) if f.endswith('.mp3')]

if not mp3_files:
    error('No mp3 files found in the directory.')

# Create the output directory if it doesn't exist, or clear it if it does
output_directory = os.path.join(current_directory, '../decoded_audio_files')
if os.path.exists(output_directory):
    shutil.rmtree(output_directory)
os.makedirs(output_directory)

# Loop through each .mp3 file and decode it
for TRACK_FILE in mp3_files:
    # Set output file path
    file = os.path.join(output_directory, TRACK_FILE + ".decode")
    
    # ffmpeg command to decode mp3
    cmd = f'ffmpeg -y -i "{TRACK_FILE}" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 "{file}"'
    
    # Execute the ffmpeg command (always overwrites the file)
    os.system(cmd)
    
    # Check if the file was created successfully
    if not os.path.isfile(file):
        error(f'Failed to decode {TRACK_FILE}')
    else:
        print(f'Successfully decoded {TRACK_FILE} to {file}')
