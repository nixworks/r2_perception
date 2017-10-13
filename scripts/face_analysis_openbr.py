#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import time
import cv2
import csv
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from r2_perception.msg import FaceRequest,FaceResponse
from cv_bridge import CvBridge


# create OpenCV-ROS bridge object
opencv_bridge = CvBridge()


class FaceAnalysisOpenBiometrics(object):


    # constructor
    def __init__(self):

        # get fixed parameters
        self.face_analysis_temp_dir = rospy.get_param("/face_analysis_temp_dir")
        if not os.path.exists(self.face_analysis_temp_dir):
            os.makedirs(self.face_analysis_temp_dir)
        self.thumbs_ext = rospy.get_param("/thumbs_ext")

        # start dynamic reconfigure client from vision_pipeline
        #self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)

        # start subscriber and publisher
        self.request_sub = rospy.Subscriber("face_request",FaceRequest,self.HandleFaceRequest)
        self.response_pub = rospy.Publisher("face_response",FaceResponse,queue_size=5)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):
        ()


    # when a face request comes in
    def HandleFaceRequest(self,data):

        # create filename for face image
        filename_base = str(data.camera_id)
        image_path = self.face_analysis_temp_dir + "/" + filename_base + "." + self.thumbs_ext
        csv_path = self.face_analysis_temp_dir + "/" + filename_base + ".csv"

        # save thumbnail for OpenBiometrics
        image = opencv_bridge.imgmsg_to_cv2(data.thumb)
        cv2.imwrite(image_path,image)

        # run OpenBiometrics commandline for age estimation
        os.system("br -algorithm AgeEstimation -enroll " + image_path + " " + csv_path);

        # interpret csv result
        age = 0.0
        age_confidence = 0.0
        with open(csv_path,"rb") as file:
            reader = csv.reader(file)
            line = reader.next()
            line = reader.next()
            age = float(line[1])
            age_confidence = 1.0  # for now...

        # run OpenBiometrics commandline for gender estimation
        os.system("br -algorithm GenderEstimation -enroll " + image_path + " " + csv_path);

        # interpret csv result
        gender = 1
        gender_confidence = 0.0
        with open(csv_path,"rb") as file:
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
        msg.cface_id = data.cface_id
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

    rospy.init_node("face_analysis")
    node = FaceAnalysisOpenBiometrics()
    rospy.spin()
