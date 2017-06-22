#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import time
import cv2
import csv

from sensor_msgs.msg import Image
from r2_perception.msg import FaceRequest,FaceResponse
from cv_bridge import CvBridge


opencv_bridge = CvBridge()


class FaceAnalysis(object):


    def __init__(self):

        self.analysis_dir = rospy.get_param("/analysis_dir")

        self.request_sub = rospy.Subscriber("face_request",FaceRequest,self.HandleFaceRequest)
        self.response_pub = rospy.Publisher("face_response",FaceResponse,queue_size=5)


    def HandleFaceRequest(self,data):

        name = str(data.camera_id)

        # save thumbnail for OpenBiometrics
        color_image = opencv_bridge.imgmsg_to_cv2(data.thumb)
        cv2.imwrite(self.analysis_dir + "/{}.png".format(name),color_image)

        # run OpenBiometrics commandline for age estimation
        os.system("br -algorithm AgeEstimation -enroll " + self.analysis_dir + "/{}.png ".format(name) + self.analysis_dir + "/{}.csv".format(name));

        # interpret csv result
        age = 0.0
        age_confidence = 0.0
        with open(self.analysis_dir + "/{}.csv".format(name),"rb") as file:
            reader = csv.reader(file)
            line = reader.next()
            line = reader.next()
            age = float(line[1])
            age_confidence = 1.0  # for now...

        # run OpenBiometrics commandline for gender estimation
        os.system("br -algorithm GenderEstimation -enroll " + self.analysis_dir + "/{}.png ".format(name) + self.analysis_dir + "/{}.csv".format(name));

        # interpret csv result
        gender = 1
        gender_confidence = 0.0
        with open(self.analysis_dir + "/{}.csv".format(name),"rb") as file:
            reader = csv.reader(file)
            line = reader.next()
            line = reader.next()
            if line[10] == "Female":
                gender = 2
            gender_confidence = 1.0  # for now...

        # and send message
        msg = FaceResponse()
        msg.session_id = data.session_id
        msg.camera_id = data.camera_id
        msg.cuser_id = data.cuser_id
        msg.face_id = data.face_id
        msg.ts = data.ts
        msg.age = age
        msg.age_confidence = age_confidence
        msg.gender = gender
        msg.gender_confidence = gender_confidence
        msg.identity = 0
        msg.identity_confidence = 0.0
        self.response_pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('face_analysis')
    node = FaceAnalysis()
    rospy.spin()
