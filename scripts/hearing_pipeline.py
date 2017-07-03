#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import numpy
import time
import cv2
from dynamic_reconfigure.server import Server
from r2_perception.cfg import HearingConfig
from r2_perception.msg import Sound,Speech,CandidateSound,CandidateSpeech


class HearingPipeline(object):


    def __init__(self):

        self.sounds_base_dir = rospy.get_param("/sounds_dir")

        self.name = rospy.get_namespace().split('/')[-2]
        
        self.session_tag = str(rospy.get_param("/session_tag"))
        self.session_id = hash(self.session_tag) & 0xFFFFFFFF

        self.microphone_id = hash(self.name) & 0xFFFFFFFF

        today_tag = time.strftime("%Y%m%d")
        microphone_tag = self.name + "_%08X" % (self.microphone_id & 0xFFFFFFFF)
        self.sounds_dir = self.sounds_base_dir + "/" + today_tag + "/" + self.session_tag + "_%08X/" % (self.session_id & 0xFFFFFFFF) + microphone_tag + "/"
        if not os.path.exists(self.sounds_dir):
            os.makedirs(self.sounds_dir)

        self.sound_sub = rospy.Subscriber("raw_sound",Sound,self.HandleSound)
        self.speech_sub = rospy.Subscriber("raw_speech",Speech,self.HandleSpeech)

        self.sound_pub = rospy.Publisher("csound",CandidateSound,queue_size=5)
        self.speech_pub = rospy.Publisher("cspeech",CandidateSpeech,queue_size=5)

        self.config_srv = Server(HearingConfig,self.HandleConfig)


    def HandleConfig(self,data,level):

        print "{}".format(data)
        return data


    def HandleSound(self,data):

        sound_tag = "sound_%08X" % (data.sound_id & 0xFFFFFFFF)
        sound_file = self.sounds_dir + sound_tag + ".wav"
        # write sound


    def HandleSpeech(self,data):

    	() # write speech in CSV and transmit to fusion


if __name__ == '__main__':

    rospy.init_node('hearing_pipeline')
    node = HearingPipeline()
    rospy.spin()
