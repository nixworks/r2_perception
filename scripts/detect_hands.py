#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

# HAND DETECTION: template

# HAND DETECTION: analyze camera frame and output all found hands at a configurable rate
#     the camera is a ROS USB camera node under 'camera' in the local namespace
#     the raw hands are published to 'raw_hand' in the local namespace
#     parameter updates are gathered from the 'vision_pipeline' parameter server

# the node should be called 'detect_hands'

from __future__ import with_statement
import os
import rospy
import logging
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from r2_perception.msg import Hand
from cv_bridge import CvBridge
from threading import Lock


# create OpenCV-ROS bridge object
opencv_bridge = CvBridge()


# Generate unique serial number for the raw hands 
serial_number = 0
def GenerateHandID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectHands(object):


    # constructor
    def __init__(self):

        # create lock
        self.lock = Lock()

        # start possible debug window
        cv2.startWindowThread()

        # initialize current frame and timestamp
        self.cur_image = Image()
        self.cur_ts = 0.0

        # get pipeline name
        self.name = rospy.get_namespace().split('/')[-2]

        # get parameters
        self.debug_hand_detect_flag = rospy.get_param("debug_hand_detect_flag")
        if self.debug_hand_detect_flag:
            cv2.namedWindow(self.name + " hands")

        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")

        self.hand_detect_rate = rospy.get_param("hand_detect_rate")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hand_detect_rate),self.HandleTimer)

        self.hand_detect_work_width = rospy.get_param("hand_detect_work_width")
        self.hand_detect_work_height = rospy.get_param("hand_detect_work_height")

        # start dynamic reconfigure client
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.hand_pub = rospy.Publisher("raw_hand",Hand,queue_size=5)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):

        new_debug_hand_detect_flag = data.debug_hand_detect_flag
        if new_debug_hand_detect_flag != self.debug_hand_detect_flag:
            self.debug_hand_detect_flag = new_debug_hand_detect_flag
            if self.debug_hand_detect_flag:
                cv2.namedWindow(self.name + " hands")
            else:
                cv2.destroyWindow(self.name + " hands")

        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate

        new_hand_detect_rate = data.hand_detect_rate
        if new_hand_detect_rate != self.hand_detect_rate:
            self.hand_detect_rate = new_hand_detect_rate
            self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.hand_detect_rate),self.HandleTimer)            

        self.hand_detect_work_width = data.hand_detect_work_width
        self.hand_detect_work_height = data.hand_detect_work_height


    # when a camera image arrives
    def HandleImage(self,data):

        with self.lock:

            # copy the image and update the timestamp
            self.cur_image = data
            self.cur_ts = rospy.get_rostime()


    # at face detection rate
    def HandleTimer(self,event):

        with self.lock:

            # if no image is available, exit
            if self.cur_ts == 0.0:
                return

            # TODO: detect all the hands

            # TODO: if there are no hands detected, exit

            # TODO: publish all the hands


if __name__ == '__main__':

    rospy.init_node('detect_hands')
    node = DetectHands()
    rospy.spin()
