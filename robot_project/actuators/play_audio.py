#!/usr/bin/python3

# ----------------------------------------------
# Title: play_audio.py
# Description: Plays an mp3 audio
# Author: Kento
# Date created: Dec 24, 2024
# Date modified: Dec 29, 2024
# ----------------------------------------------

import rospy
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray, String
import time
import sys
import os
import numpy as np
from actuators.audio_history import AudioHistory

# messages larger than this will be dropped by the receiver
MAX_STREAM_MSG_SIZE = (4096 - 48)

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

################################################################

def error(msg):
    print(msg)
    sys.exit(0)

################################################################

class AudioPlayer:
    def __init__(self):
        self.buffer_space = 0
        self.buffer_total = 0
        self.data_r = 0
        self.data = []
        self.stop_flag = False

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def loop(self, args):
        state_file = None
        if len(args):
            state_file = args[0]

        count = 0
        dropout_data_r = -1
        dropout_count = 3

        while not rospy.core.is_shutdown() and not self.stop_flag:
            if not state_file is None:
                if not os.path.isfile(state_file):
                    break

            if self.buffer_total > 0:
                buffer_rem = self.buffer_total - self.buffer_space
                n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                n_bytes = max(n_bytes, 0)
                n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

                if n_bytes > 0:
                    msg = Int16MultiArray(data=self.data[self.data_r:self.data_r + n_bytes])
                    self.pub_stream.publish(msg)
                    self.data_r += n_bytes

            if self.data_r >= len(self.data):
                break

            if count == 0:
                count = 10
                if dropout_data_r == self.data_r:
                    if dropout_count == 0:
                        print("dropping out because of no progress...")
                        break
                    print("dropping out in", str(dropout_count) + "...")
                    dropout_count -= 1
                else:
                    dropout_data_r = self.data_r

            count -= 1
            time.sleep(0.1)

    def play_audio(self, mp3_file_path):
        TRACK_PATH = f'{mp3_file_path}'

        if not os.path.isfile(TRACK_PATH):
            error('ERROR: file not found')

        # Log the audio file to the shared history
        AudioHistory().add_audio(TRACK_PATH)

        print("playing file", TRACK_PATH)

        file = "/tmp/" + os.path.basename(TRACK_PATH) + ".decode"
        print("decoding to", file)
        if not os.path.isfile(file):
            cmd = f"ffmpeg -y -i \"{TRACK_PATH}\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"{file}\" > /dev/null 2>&1"
            os.system(cmd)
            if not os.path.isfile(file):
                error('failed decode mp3')

        with open(file, 'rb') as f:
            dat = f.read()
        self.data_r = 0

        dat = np.fromstring(dat, dtype='int16').astype(np.int32)
        dat = dat.astype(np.float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()

        self.data = dat

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        topic = topic_base_name + "/control/stream"
        print("publish", topic)
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        topic = topic_base_name + "/platform/log"
        print("subscribe", topic)
        self.sub_log = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        topic = topic_base_name + "/sensors/stream"
        print("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

        self.stop_flag = False
        self.loop([])

    def stop(self):
        self.stop_flag = True

if __name__ == "__main__":
    # Example for direct execution: 
    # player = AudioPlayer()
    # player.play_audio("mp3_files/alices_adventures_ch1.mp3")
    pass

