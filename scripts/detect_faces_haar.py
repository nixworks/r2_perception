#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

# FACE DETECTION: HAAR

# FACE DETECTION: analyze camera frame and output all found faces at a configurable rate
#     the camera is a ROS USB camera node under 'camera' in the local namespace
#     the raw faces are published to 'raw_face' in the local namespace
#     parameter updates are gathered from the 'vision_pipeline' parameter server

# HAAR: faces are detected using OpenCV Haar cascades

# the node should be called 'detect_faces'

from __future__ import with_statement
import os
import rospy
import numpy
import time
import cv2
import math
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from r2_perception.msg import Face,Float32XYZ
from cv_bridge import CvBridge
from threading import Lock


# create OpenCV-ROS bridge object
opencv_bridge = CvBridge()


# Generate unique serial number for the raw faces 
serial_number = 0

def GenerateFaceID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectFacesHaar(object):


    # constructor
    def __init__(self):

        # create lock
        self.lock = Lock()

        # start possible debug window
        cv2.startWindowThread()

        # initialize current frame and timestamp
        self.cur_frame = Image()
        self.cur_ts = 0.0

        # get pipeline name
        self.name = rospy.get_namespace().split('/')[-2]

        # get fixed parameters
        self.thumb_width = rospy.get_param("/thumb_width")
        self.thumb_height = rospy.get_param("/thumb_height")

        self.haar_cascade_filename = rospy.get_param("/haar_cascade_filename")
        self.face_cascade = cv2.CascadeClassifier(self.haar_cascade_filename)

        # get dynamic parameters
        self.debug_face_detect_flag = rospy.get_param("debug_face_detect_flag")
        if self.debug_face_detect_flag:
            cv2.namedWindow(self.name + " faces")

        self.face_height = rospy.get_param("face_height")

        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")

        self.face_detect_rate = rospy.get_param("face_detect_rate")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.face_detect_rate),self.HandleTimer)

        self.face_detect_work_width = rospy.get_param("face_detect_work_width")
        self.face_detect_work_height = rospy.get_param("face_detect_work_height")

        self.haar_scale_factor = rospy.get_param("haar_scale_factor")
        self.haar_min_width = rospy.get_param("haar_min_width")
        self.haar_min_height = rospy.get_param("haar_min_height")

        # start dynamic reconfigure client from vision_pipeline
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.face_pub = rospy.Publisher("raw_face",Face,queue_size=5)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):

        new_debug_face_detect_flag = data.debug_face_detect_flag
        if new_debug_face_detect_flag != self.debug_face_detect_flag:
            self.debug_face_detect_flag = new_debug_face_detect_flag
            if self.debug_face_detect_flag:
                cv2.namedWindow(self.name + " faces")
            else:
                cv2.destroyWindow(self.name + " faces")

        self.face_height = data.face_height

        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate

        new_face_detect_rate = data.face_detect_rate
        if new_face_detect_rate != self.face_detect_rate:
            self.face_detect_rate = new_face_detect_rate
            self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.face_detect_rate),self.HandleTimer)            

        self.face_detect_work_width = data.face_detect_work_width
        self.face_detect_work_height = data.face_detect_work_height

        self.haar_scale_factor = data.haar_scale_factor
        self.haar_min_width = data.haar_min_width
        self.haar_min_height = data.haar_min_height


    # when a new camera image arrives
    def HandleImage(self,data):

        with self.lock:

            # copy the image and update the timestamp
            self.cur_image = data
            self.cur_ts = rospy.get_rostime()


    # at face detection rate
    def HandleTimer(self,data):

        with self.lock:

            # if no image is available, exit
            if self.cur_ts == 0.0:
                return

            # calculate distance to camera plane
            cpd = 1.0 / math.tan(self.fovy)

            # cache working size (change according to rotation)
            width = self.face_detect_work_width
            height = self.face_detect_work_height

            # convert image from ROS to OpenCV, unrotate and rescale
            image = cv2.resize(opencv_bridge.imgmsg_to_cv2(self.cur_image,"bgr8"),(width,height),interpolation=cv2.INTER_LINEAR)
            if self.rotate == 90:
                width = self.face_detect_work_height
                height = self.face_detect_work_width
                cpd /= self.aspect
                image = cv2.transpose(image)
            elif self.rotate == -90:
                width = self.face_detect_work_height
                height = self.face_detect_work_width
                cpd /= self.aspect
                image = cv2.transpose(image)
                image = cv2.flip(image,1)
            elif self.rotate == 180:
                image = cv2.flip(image,1)
                image = cv2.flip(image,-1)

            # detect all faces in the image
            faces = self.face_cascade.detectMultiScale(image,scaleFactor=self.haar_scale_factor,minSize=(self.haar_min_width,self.haar_min_height),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

            # if there are no faces detected, exit
            if len(faces) == 0:
                return

            # iterate over all found faces
            for (x,y,w,h) in faces:

                # if the face actually doesn't exist, continue with next face
                if (w <= 0) or (h <= 0):
                    continue

                # calculate distance of the face to the camera
                cx = float(self.face_height) * cpd * float(height) / float(h)

                # convert camera coordinates to normalized coordinates on the camera plane
                fy = 1.0 - 2.0 * float(x + w / 2) / float(width)
                fz = 1.0 - 2.0 * float(y + h / 2) / float(height)

                # project to face distance
                cy = cx * fy / cpd
                cz = cx * fz / cpd

                # prepare raw face message
                msg = Face()
                msg.face_id = GenerateFaceID()
                msg.ts = self.cur_ts
                msg.rect.origin.x = -fy
                msg.rect.origin.y = -fz
                msg.rect.size.x = 2.0 * float(w) / float(width)
                msg.rect.size.y = 2.0 * float(h) / float(height)
                msg.position.x = cx
                msg.position.y = cy
                msg.position.z = cz
                msg.confidence = 1.0
                msg.smile = 0.0
                msg.frown = 0.0
                msg.expressions = []
                msg.landmarks = []

                # cut out face thumbnail and resize
                cvthumb = cv2.resize(image[y:y+h,x:x+w],(self.thumb_width,self.thumb_height))

                # convert thumbnail from OpenCV to ROS
                msg.thumb = opencv_bridge.cv2_to_imgmsg(cvthumb,encoding="8UC3")

                # and publish the raw face
                self.face_pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('detect_faces')
    node = DetectFacesHaar()
    rospy.spin()
