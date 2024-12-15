#!/usr/bin/python3

# This program listens through the MiRo's ear microphones and
# converts speech it hears to text. The desired features are:
#
# - Only record and store files when noise heard
# - Overwrite old wav files so audio is not stored
# - Convert voice to speech in near real time

import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray

import time
import os
import numpy as np
import wave, struct
import speech_recognition as sr

import miro2 as miro

# initialize the recognizer
r = sr.Recognizer()

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 5

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

class spch_to_text:

    def callback_stream(self, msg):

        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]
        self.buffer_stuff = self.buffer_total - self.buffer_space

    def callback_mics(self, msg):

        if not self.micbuf is None:
            # Convert the incoming microphone data to NumPy array for processing
            mic_data = np.array(msg.data, dtype=np.int16).reshape(-1,4)
            channel_to_process = 0 # Select the channel to monitor
            # Calculate RMS value for sound intensity
            single_channel_data = mic_data[:, channel_to_process]
            rms = int(np.sqrt(np.mean(single_channel_data.astype(np.float32) ** 2)))

            #print(f"The rms is {rms}")

            # Define a threshold for sound intensity (adjust as needed)
            THRESHOLD = 190  # Experiment with this value to suit your environment

            if rms > THRESHOLD or self.valid_time:
                # Append mic data to buffer if above the threshold

                self.micbuf = np.concatenate((self.micbuf, msg.data))
                self.noise_detected += 1
                #print(f"Sound detected! {self.noise_detected}")
                #print(f"rms = {rms}")

                # Logic to determine whether or not to record
                if self.valid_time:
                    if time.time() > self.record_time + RECORD_TIME:
                        self.valid_time = False
                elif not self.valid_time:
                    self.valid_time = True
                    self.record_time = time.time()

            # Check if recording chunk is complete
            if self.micbuf.shape[0] >= SAMPLE_COUNT:

                # Process the current chunk of audio data
                self.outbuf = self.micbuf[:SAMPLE_COUNT]  # Extract the first chunk
                # print("Recording complete! Processing the chunk...")
                self.record = True

                # Reset the buffer for continuous recording
                self.micbuf = self.micbuf[SAMPLE_COUNT:]  # Retain any extra samples
                # print("Ready to record the next chunk.")

    def loop(self):

        while not rospy.core.is_shutdown():
            if self.record:

                # write output file
                outfilename = '/home/bryce_cap/mdk/bin/shared/tmp/client_audio.wav'
                file = wave.open(outfilename, 'wb')
                file.setsampwidth(2)
                file.setframerate(MIC_SAMPLE_RATE)

                # write data
                print ("writing two channels to file (LEFT and RIGHT)...")
                file.setnchannels(2)
                x = np.reshape(self.outbuf[:, [0, 1]], (-1))
                for s in x:
                    file.writeframes(struct.pack('<h', s))

                # close file
                file.close()
                print ("wrote output file at", outfilename)
                self.record = False

                # send the audio to the Google Speech recognizer to decode it
                with sr.AudioFile(outfilename) as source:
                    # listen for the data (load audio to memory)
                    audio_data = r.record(source)
                    # recognize (convert from speech to text)
                    try:
                        text = r.recognize_google(audio_data)
                        print(f"\n{text}\n")
                    except sr.UnknownValueError:
                        print("\nAudio could not be understood.\n")
                    except sr.RequestError as e:
                        print(f"API error: {e}")

            # Sleep to avoid CPU overuse
            time.sleep(0.02)


    def __init__(self):

        # Create robot interface
        self.interface = miro.lib.RobotInterface()

        # state
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.buffer_stuff = 0
        self.playchan = 0
        self.playsamp = 0
        self.noise_detected = 0
        self.record = False
        self.valid_time = False
        self.record_time = 0.0

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        topic = topic_base_name + "/sensors/stream"
        print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

        #subscribe to mics using robot Interface
        self.interface.register_callback("microphones", self.callback_mics)

        # report
        print ("recording from 4 microphones for", RECORD_TIME, "seconds...")

if __name__ == "__main__":

    main = spch_to_text()
    main.loop()