#!/usr/bin/python3

# This program listens through the MiRo's ear microphones and
# converts speech it hears to text. The desired features are:
#
# - Only record and store files when noise heard
# - Overwrite old wav files so audio is not stored
# - Convert voice to speech in near real time

import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray
import threading

import time
import os
import numpy as np
import wave, struct, io
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
RECORD_TIME = 3

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

class SpeechToText:

    def callback_stream(self, msg):

        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]
        self.buffer_stuff = self.buffer_total - self.buffer_space

    def callback_mics(self, msg):
        if not self.micbuf is None:
            data = np.asarray(msg.data)
            new_data = np.transpose(data.reshape(4, 500))

            # Convert the incoming microphone data to NumPy array for processing
            mic_data = np.array(new_data, dtype=np.int16).reshape(-1,4)
            channel_to_process = 0 # Select the channel to monitor
            # Calculate RMS value for sound intensity
            single_channel_data = mic_data[:, channel_to_process]
            right_ear_data = mic_data[:, 1]
            tail_data = mic_data[:, 3]
            rms = int(np.sqrt(np.mean(single_channel_data.astype(np.float32) ** 2)))
            rms_right = int(np.sqrt(np.mean(right_ear_data.astype(np.float32) ** 2)))
            rms_tail = int(np.sqrt(np.mean(tail_data.astype(np.float32) ** 2)))

            THRESHOLD_CRINKLE_TAIL = 18000 
            THRESHOLD_CRINKLE_EARS = 9000

            if rms > THRESHOLD_CRINKLE_EARS:
                #print(f"The rms for the left ear is {rms}")
                self.left_crinkle = True
            else:
                self.left_crinkle = False

            if rms_right > THRESHOLD_CRINKLE_EARS:
                #print(f"The rms for the right ear is {rms_right}")
                self.right_crinkle = True
            else:
                self.right_crinkle = False

            if rms_tail > THRESHOLD_CRINKLE_TAIL:
                #print(f"The rms for the tail is {rms_tail}")
                self.tail_crinkle = True
            else:
                self.tail_crinkle = False

            #print("len(mic_data) = ", len(mic_data))
            #print("len(msg.data) = ", len(msg.data))

            # Define a threshold for sound intensity (adjust as needed)
            THRESHOLD = 280  # Experiment with this value to suit your environment

            if rms > THRESHOLD or self.valid_time:
                # Append mic data to buffer if above the threshold

                """
                if self.valid_time and rms > THRESHOLD:
                    print("Both tiggers are active")
                    print(f"rms = {rms}")
                elif rms > THRESHOLD:
                    print(f"Caused by rms = {rms}")
                elif self.valid_time:
                    print("self.valid_time activation")
                else:
                    print("That is weird")
                """

                self.micbuf = np.concatenate((self.micbuf, mic_data))
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
        while not rospy.core.is_shutdown() and self.listen:
            if self.stop:
                break
            
            if self.record:
                with io.BytesIO() as buffer:
                    with wave.open(buffer, 'wb') as file:
                        file.setsampwidth(2)
                        file.setframerate(MIC_SAMPLE_RATE)
                        file.setnchannels(2)
                        x = np.reshape(self.outbuf[:, [0, 1]], (-1))
                        for s in x:
                            file.writeframes(struct.pack('<h', s))

                    buffer.seek(0)
                    
                    with sr.AudioFile(buffer) as source:
                        audio_data = r.record(source)
                        try:
                            text = r.recognize_azure(audio_data, key="Ft27qAeJW6CJup9n2XJfsWdrnsz6mtTZygdG8SZx6BuJzktUEVM9JQQJ99ALACBsN54XJ3w3AAAYACOGhZ9C", location="canadacentral")
                            print(f"\n{text}\n")
                            with self.lock:
                                self.last_text = text[0]
                                self.last_confidence = text[1]
                        except sr.UnknownValueError:
                            print("\nAudio could not be understood.\n")
                        except sr.RequestError as e:
                            print(f"API error: {e}")


                self.record = False
                self.valid_time = False
            time.sleep(0.02)

    def __init__(self):

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
        self.stop = False
        self.listen = True

        # Make last_text and last_confidence members that contain the info given by the speech-to-text API
        self.last_text = "None"
        self.last_confidence = 0.0
        # Other variables to pass to interactive standby
        self.left_crinkle = False
        self.right_crinkle = False
        self.tail_crinkle = False

        # Create a lock for thread safety
        self.lock = threading.Lock()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        topic = topic_base_name + "/sensors/stream"
        print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

        #subscribe to mics. Previously used robot interface. Removed queue_size=1, 
        self.sub_mics = rospy.Subscriber(topic_base_name + "/sensors/mics", Int16MultiArray, self.callback_mics, tcp_nodelay=True)
    
        # report
        print ("recording from 4 microphones for", RECORD_TIME, "seconds...")

        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

if __name__ == "__main__":
    # Initialize a ROS node to communicate with MiRo
    rospy.init_node("speech_to_text")
    main = SpeechToText()
    main.loop()