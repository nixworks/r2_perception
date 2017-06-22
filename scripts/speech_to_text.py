#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import logging
import time
import dynamic_reconfigure.client
from r2_perception.msg import Sound,Speech


# speech to text
class SpeechToText(object):


    def __init__(self):

        self.threshold = 0.2
        self.linger = 0.2
        
        #self.dynparam = dynamic_reconfigure.client.Client("hearing_pipeline",timeout=30,config_callback=self.HandleConfig)

        self.sound_sub = rospy.Subscriber("sound",Sound,self.HandleSound)
        self.speech_pub = rospy.Publisher("speech",Speech,queue_size=5)


    def HandleConfig(self,data,level):

        self.threshold = data.threshold
        self.linger = data.linger

        
    def HandleSound(self,data):

        ()


if __name__ == '__main__':

    rospy.init_node('speech_to_text')
    node = SpeechToText()
    rospy.spin()
