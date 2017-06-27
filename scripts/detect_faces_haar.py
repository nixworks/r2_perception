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
from r2_perception.msg import Face
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


class DetectFaces(object):


    # constructor
    def __init__(self):

        # create lock
        self.lock = Lock()

        # initialize current frame and timestamp
        self.cur_frame = Image()
        self.cur_ts = 0.0

        # get global parameters
        self.haar_cascade = rospy.get_param("/haar_cascade")
        self.face_height = rospy.get_param("/face_height")

        # get local parameters
        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")
        self.face_detect_rate = rospy.get_param("face_detect_rate")

        # start Haar cascade
        self.face_cascade = cv2.CascadeClassifier(self.haar_cascade)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.face_detect_rate),self.HandleTimer)

        # start dynamic reconfigure client
        #self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.face_pub = rospy.Publisher("raw_face",Face,queue_size=5)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):

        # copy parameters from server
        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate
        self.face_detect_rate = data.face_detect_rate

        # reset timer
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.face_detect_rate),self.HandleTimer)


    # when a new camera image arrives
    def HandleImage(self,data):

        with self.lock:

            # copy the image and update the timestamp
            self.cur_image = data
            self.cur_ts = rospy.get_rostime()


    # at face detection rate
    def HandleTimer(self,data):

        rosts = rospy.get_rostime()

        with self.lock:

            # if no image is available, exit
            if self.cur_ts == 0.0:
                return

            # convert image from ROS to OpenCV
            subx = 160
            suby = 120
            cpd = 1.0 / math.tan(self.fovy)
            color_image = cv2.resize(opencv_bridge.imgmsg_to_cv2(self.cur_image,"bgr8"),(subx,suby),interpolation=cv2.INTER_LINEAR)

            if self.rotate == 90:
                subx = 120
                suby = 160
                cpd /= self.aspect
                color_image = cv2.transpose(color_image)

            elif self.rotate == -90:
                subx = 120
                suby = 160
                cpd /= self.aspect
                color_image = cv2.transpose(color_image)
                color_image = cv2.flip(color_image,1)

            elif self.rotate == 180:
                color_image = cv2.flip(color_image,-1)

            # detect all faces in the image
            faces = self.face_cascade.detectMultiScale(color_image,scaleFactor=1.1,minSize=(20,20),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

            # faces is now a list of (x,y,w,h) tuples, where (x,y) is the center of the face rectangle and (w,h) is the size

            # if there are no faces detected, exit
            if len(faces) == 0:
                return

            # iterate over all faces
            for (x,y,w,h) in faces:

                # if the face actually doesn't exist, continue with next face
                if (w <= 0) or (h <= 0):
                    continue

                # calculate distance of the face to the camera (X-coordinate)
                cx = float(self.face_height) * cpd * float(suby) / float(h)

                # convert camera coordinates to normalized coordinates on the camera plane
                fy = 1.0 - 2.0 * float(x + w / 2) / float(subx)
                fz = 1.0 - 2.0 * float(y + h / 2) / float(suby)

                # project to face distance (Y- and Z-coordinates)
                cy = cx * fy / cpd
                cz = cx * fz / cpd

                # prepare raw face message
                msg = Face()

                msg.face_id = GenerateFaceID()

                msg.ts = self.cur_ts

                msg.rect.origin.x = -fy
                msg.rect.origin.y = -fz
                msg.rect.size.x = 2.0 * float(w) / float(subx)
                msg.rect.size.y = 2.0 * float(h) / float(suby)

                msg.position.x = cx
                msg.position.y = cy
                msg.position.z = cz

                msg.confidence = 1.0
                msg.smile = 0.0
                msg.frown = 0.0

                msg.expressions = []

                msg.landmarks = []

                # cut out face thumbnail and resize to 64x64
                cvthumb = cv2.resize(color_image[y:y+h,x:x+w],(64,64))

                # convert thumbnail from OpenCV to ROS
                msg.thumb = opencv_bridge.cv2_to_imgmsg(cvthumb,encoding="8UC3")

                # and publish the raw face
                self.face_pub.publish(msg)

            print "after detect_faces {}".format(rospy.get_rostime().to_sec())


if __name__ == '__main__':

    rospy.init_node('detect_faces')
    node = DetectFaces()
    rospy.spin()
