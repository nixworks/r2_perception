#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import time
import cv2

from sensor_msgs.msg import Image
from r2_perception.msg import FaceRequest,FaceResponse
from cv_bridge import CvBridge


opencv_bridge = CvBridge()


class FaceAnalysis(object):


    def __init__(self):

        self.request_sub = rospy.Subscriber("face_request",FaceRequest,self.HandleFaceRequest)
        self.response_pub = rospy.Publisher("face_response",FaceResponse,queue_size=5)


    def HandleFaceRequest(self,data):

        ()
        

if __name__ == '__main__':

    rospy.init_node('face_analysis')
    node = FaceAnalysis()
    rospy.spin()
