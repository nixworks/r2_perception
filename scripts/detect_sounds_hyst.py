#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import logging
import time
import pyaudio
import ctypes
import serial
import dynamic_reconfigure.client
from r2_perception.msg import Sound


# Generate unique serial number for the raw sounds 
serial_number = 0

def GenerateSoundID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectSoundHyst(object):


    def publish_angle(self, data):
        # map angle based on params
        # Converts raw data to the angle in degrees and publishes to ROS
        angle = ((data - 125) * self.config['max_angle'])/125
        if self.config['invert']:
            angle = 0-angle
        angle += self.config['center_angle']
        self.audio_pub.publish(angle)

    def start(self):
        while not rospy.is_shutdown():
                        x = ser.read()
                        #print ord(x)
                        # 255 no signal
                        if ord(x) < 250:
                            self.publish_angle(ord(x))
            except Exception as e:
                print(e)

    def __init__(self):

        self.device = rospy.get_param("device")
        self.threshold = rospy.get_param("threshold")
        self.linger = rospy.get_param("linger")
        self.rate = rospy.get_param("mic_rate")
        self.sound_detect_rate = rospy.get_param("sound_detect_rate")
        self.localization = rospy.get_param("localization")
        if self.localization:
            self.port = rospy.get_param("port","/dev/ttyUSB0")
            self.serial = serial.Serial(self.port,baudrate=2400)
        
        #self.dynparam = dynamic_reconfigure.client.Client("hearing_pipeline",timeout=30,config_callback=self.HandleConfig)

        self.audio = pyaudio.PyAudio()

        # figure out which 'device index' belongs to the device
        info = self.audio.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        self.devindex = -1
        for i in range(0, numdevices):
            if (self.audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                devname = self.audio.get_device_info_by_host_api_device_index(0, i).get('name')
                if self.device in devname:
                    self.devindex = i
                    break

        if not self.audio.is_format_supported(self.rate,input_channels=1,input_format=pyaudio.paInt16,input_device=self.devindex):
            print "audio format not supported"

        self.stream = self.audio.open(format=pyaudio.paInt16,channels=1,rate=int(self.rate),input=True,input_device_index=self.devindex,frames_per_buffer=int(self.rate) / 10)
        self.stream.start_stream()

        self.sound_pub = rospy.Publisher("raw_sound",Sound,queue_size=5)


    def HandleConfig(self,data):

        self.threshold = data.threshold
        self.linger = data.linger


    def Step(self):

        raw = ""
        # NOTE: try/except because of overflow error and documented exception_on_overflow parameter doesn't actually work
        try:
            raw = self.stream.read(int(self.rate) / 10)
        except:
            ()

        if not raw:
            return

        # TODO: read localization data

        # NOTE: convert data to shorts, and later back to whatever the message serializer does, this is potentially slow...
        data = []
        for i in range(0,len(raw) / 2):
            data.append(ctypes.c_short(ord(raw[i * 2]) | (ord(raw[i * 2 + 1]) << 8)).value)

        # TODO: do hysteresis thresholding stuff, like Alice

        # for now, just output the whole chunk
        msg = Sound()
        msg.sound_id = GenerateSoundID()
        msg.ts = rospy.get_rostime()
        msg.audio = data

        self.sound_pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('detect_sound')
    node = DetectSoundHyst()
    while not rospy.core.is_shutdown():
        node.Step()
        rospy.rostime.wallsleep(0.01)
