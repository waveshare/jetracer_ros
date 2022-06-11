#!/usr/bin/python
# -*- coding:utf-8 -*-
'''
Requirements:
+ pyaudio - `pip3 install pyaudio`
+ py-webrtcvad - `pip3 install webrtcvad`
'''
import webrtcvad
import collections
import sys
import signal
import pyaudio

from array import array
from struct import pack
import wave
import time
import os
from subprocess import Popen, PIPE

import rospy
from std_msgs.msg import String

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK_DURATION_MS = 30       # supports 10, 20 and 30 (ms)
PADDING_DURATION_MS = 1500   # 1 sec jugement
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)  # chunk to read
CHUNK_BYTES = CHUNK_SIZE * 2  # 16bit = 2 bytes, PCM
NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS = int(400 / CHUNK_DURATION_MS)  # 400 ms/ 30ms  ge
NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 2

START_OFFSET = int(NUM_WINDOW_CHUNKS * CHUNK_DURATION_MS * 0.5 * RATE)

def record_to_file(path, data, sample_width):
    "Records from the microphone and outputs the resulting data to 'path'"
    # sample_width, data = record()
    data = pack('<' + ('h' * len(data)), *data)
    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()


def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 32767  # 16384
    times = float(MAXIMUM) / max(abs(i) for i in snd_data)
    r = array('h')
    for i in snd_data:
        r.append(int(i * times))
    return r

def run(cmd):
    p=Popen(cmd, stdout=PIPE, shell=True)
    p.wait()

if __name__ == '__main__':
    rospy.init_node('jetracer_vad_node')
    Mode = rospy.get_param('~Mode','play')
    Path = rospy.get_param('~Path','/home/jetson/catkin_ws/src/jetracer')
    textfile = Path + "/data/talk.txt"
    pub = rospy.Publisher('chatter', String, queue_size=10)
    print(textfile)
    print(__file__)
    vad = webrtcvad.Vad(2)
    pa = pyaudio.PyAudio()

    while not rospy.is_shutdown():
        stream = pa.open(format=FORMAT,
                         channels=CHANNELS,
                         rate=RATE,
                         input=True,
                         start=False,
                         frames_per_buffer=CHUNK_SIZE)

        got_a_sentence = False
        leave = False

        while not leave and not rospy.is_shutdown():
            ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
            triggered = False
            voiced_frames = []
            ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
            ring_buffer_index = 0

            ring_buffer_flags_end = [0] * NUM_WINDOW_CHUNKS_END
            ring_buffer_index_end = 0
            buffer_in = ''
            # WangS
            raw_data = array('h')
            index = 0
            start_point = 0
            StartTime = time.time()
            print("* recording: ")
            stream.start_stream()

            while not got_a_sentence and not leave and not rospy.is_shutdown():
                chunk = stream.read(CHUNK_SIZE,exception_on_overflow = False)
                # add WangS
                raw_data.extend(array('h', chunk))
                index += CHUNK_SIZE
                active = vad.is_speech(chunk, RATE)

                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index += 1
                
                ring_buffer_index %= NUM_WINDOW_CHUNKS

                ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
                ring_buffer_index_end += 1
                ring_buffer_index_end %= NUM_WINDOW_CHUNKS_END

                # start point detection
                if not triggered:
                    ring_buffer.append(chunk)
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.8 * NUM_WINDOW_CHUNKS:
                        sys.stdout.write(' Open ')
                        StartTime = time.time()
                        triggered = True
                        start_point = index - CHUNK_SIZE * 20  # start point
                        # voiced_frames.extend(ring_buffer)
                        ring_buffer.clear()
                # end point detection
                else:
                    # voiced_frames.append(chunk)
                    ring_buffer.append(chunk)
                    num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                    if num_unvoiced > 0.90 * NUM_WINDOW_CHUNKS_END or (time.time() - StartTime) > 10:
                        sys.stdout.write(' Close \n')
                        triggered = False
                        got_a_sentence = True

                sys.stdout.flush()

            stream.stop_stream()
            print("* done recording")
            got_a_sentence = False

            # write to file
            raw_data.reverse()
            for index in range(start_point):
                raw_data.pop()
            raw_data.reverse()
            raw_data = normalize(raw_data)
            record_to_file(Path+"/data/record.wav", raw_data, 2)
            leave = True

        stream.close()

        if Mode == "asr_cn":
            run("python " + Path + "/scripts/iat.py")
            if os.path.exists(textfile):
                s=None
                with open(textfile,"r") as f:
                    s=f.readlines()
                if s is not None:
                    print(s[0].strip('\n'))
                    pub.publish(s[0].strip('\n'))
                run("rm -r " +  textfile)
        elif Mode == "talk_cn":
            run("python " + Path + "/scripts/aiui.py")
            if os.path.exists(textfile):
                s=None
                with open(textfile,"r") as f:
                    s=f.readlines()
                if s is not None:
                    try: 
                        print(s[0]+s[1])
                        pub.publish(s[0]+s[1])
                    except: 
                        pass
                run("rm -r " + textfile)
                run("play -q " + Path + "/data/demo.mp3")
                run("rm -r " + Path + "/data/demo.mp3")
        elif Mode == "asr_en":
            run("~/env/bin/python3 " + Path + "/scripts/ginput.py -i " + Path + "/data/record.wav -o  " + Path + "/data/test.wav")
            if os.path.exists(textfile):
                s=None
                with open(textfile,"r") as f:
                    s=f.readlines()
                if s is not None:
                    print(s[0])
                    pub.publish(s[0].strip('\n'))
                run("rm -r " + textfile)
        elif Mode == "talk_en":
            run("~/env/bin/python3 " + Path + "/scripts/ginput.py -i " + Path + "/data/record.wav -o  " + Path + "/data/test.wav")
            if os.path.exists(textfile):
                s=None
                with open(textfile,"r") as f:
                    s=f.readlines()
                if s is not None:
                    try: 
                        print(s[0]+s[1])
                        pub.publish(s[0]+s[1])
                    except: 
                        pass
                run("rm -r " + textfile)
                run("aplay -q -r 16000 -f S16_LE " + Path + "/data/test.wav")
                run("rm -r " + Path + "/data/test.wav")
        else:
            run("play -q "+Path+"/data/record.wav")
            
        if os.path.exists(Path+"/data/record.wav"):
            run("rm -r " + Path + "/data/record.wav")