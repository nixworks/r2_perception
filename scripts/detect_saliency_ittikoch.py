#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

# SALIENT POINT DETECTION: ITTI & KOCH

# SALIENT POINT DETECTION: analyze camera frame and output all found salient points at a configurable rate
#     the camera is a ROS USB camera node under 'camera' in the local namespace
#     the raw salient points are published to 'raw_saliency' in the local namespace
#     parameter updates are gathered from the 'vision_pipeline' parameter server

# ITTI & KOCH: the algorithm is roughly based to the work of Itti & Koch in the early 2000s

#     there are major artistic reductions for OpenCV2+Python+Hanson+Sophia context

#     the solution can be viewed as a saliency-detecting CNN, but with handcrafted weights via traditional vision operations

# the node should be called 'detect_saliency'

from __future__ import with_statement
import os
import rospy
import numpy
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from r2_perception.msg import Saliency,Float32XYZ
from cv_bridge import CvBridge
from threading import Lock


# create OpenCV-ROS bridge object
opencv_bridge = CvBridge()

# Generate unique serial number for the raw salient points
serial_number = 0
def GenerateSaliencyID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


def cv2normalize(image):
    # I probably don't understand OpenCV too well...
    dest = image.copy()
    cv2.normalize(cv2.absdiff(image,0.0),dest,0.0,1.0,cv2.NORM_MINMAX)
    return dest


class DetectSaliency(object):


    # constructor
    def __init__(self):

        # create lock
        self.lock = Lock()

        # initialize current and last frame and timestamps
        self.cur_image = Image()
        self.last_image = Image()
        self.cur_ts = 0.0
        self.last_ts = 0.0

        # get global parameters
        self.debug = rospy.get_param("/debug")

        # get local parameters
        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")
        self.saliency_detect_rate = rospy.get_param("saliency_detect_rate")

        # start timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.saliency_detect_rate),self.HandleTimer)

        # start dynamic reconfigure client
        #self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.saliency_pub = rospy.Publisher("raw_saliency",Saliency,queue_size=5)

        # debugging:
        if self.debug:
            cv2.startWindowThread()
            cv2.namedWindow("faux saliency")


    def HandleConfig(self,data):

        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate
        self.saliency_detect_rate = data.saliency_detect_rate

        # reset timer
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.saliency_detect_rate),self.HandleTimer)


    def HandleImage(self,data):

        with self.lock:

            self.last_image = self.cur_image
            self.last_ts = self.cur_ts
            self.cur_image = data
            self.cur_ts = rospy.get_rostime()


    def HandleTimer(self,data):

        with self.lock:

            if (self.cur_ts == 0.0) or (self.last_ts == 0):
                return

            # resolution for feature maps
            subx = 320
            suby = 240

            covx = 128
            covy = 96

            # convert ROS images to OpenCV and rescale to subx,suby, the working resolution
            bgr_cur_image = opencv_bridge.imgmsg_to_cv2(self.cur_image)
            bgr_last_image = opencv_bridge.imgmsg_to_cv2(self.last_image)

            if self.rotate == 90:
                subx = 240
                suby = 320
                covx = 96
                covy = 128
                bgr_cur_image = cv2.transpose(bgr_cur_image)
                bgr_last_image = cv2.transpose(bgr_last_image)

            elif self.rotate == -90:
                subx = 240
                suby = 320
                covx = 96
                covy = 128
                bgr_cur_image = cv2.transpose(bgr_cur_image)
                bgr_cur_image = cv2.flip(bgr_cur_image,1)
                bgr_last_image = cv2.transpose(bgr_last_image)
                bgr_last_image = cv2.flip(bgr_last_image,1)

            elif self.rotate == 180:
                bgr_cur_image = cv2.flip(bgr_cur_image,-1)
                bgr_last_image = cv2.flip(bgr_last_image,-1)

            # also, convert from BGR to YUV (which is more natural for vision tasks)
            yuv_u8 = cv2.cvtColor(cv2.resize(bgr_cur_image,(subx,suby),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2YUV)
            yuv = yuv_u8.astype("float32")

            yuv_last_u8 = cv2.cvtColor(cv2.resize(bgr_last_image,(subx,suby),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2YUV)
            yuv_last = yuv_last_u8.astype("float32")

            # generate motion map from difference between last frame and current frame
            dyuv = cv2.subtract(yuv,yuv_last)

            # split intensity and color maps
            yc,uc,vc = cv2.split(yuv) 
            dyc,duc,dvc = cv2.split(dyuv)

            # use Gaussian blur to create low-pass filtered versions of each map
            yc4 = cv2.GaussianBlur(yc,(13,13),0) # intensity
            uc4 = cv2.GaussianBlur(uc,(13,13),0) # red-green
            vc4 = cv2.GaussianBlur(vc,(13,13),0) # blue-yellow
            dyc4 = cv2.GaussianBlur(dyc,(13,13),0) # motion

            # emulate high-pass filtering (high-frequency content is the 'most interesting') by
            # subtracting low-pass images from unfiltered images

            # furthermore, for the non-motion maps, use Laplacian as optical derivative

            # start with the weights
            motion_factor = 0.5
            color_factor = 0.2
            contrast_factor = 0.1

            # generally, motion is the 'most interesting',
            motion = cv2normalize(dyc - dyc4)

            # color responses are 'a bit less interesting',
            vmap = cv2normalize(cv2.Laplacian(vc - vc4,-1))
            umap = cv2normalize(cv2.Laplacian(uc - uc4,-1))

            # intensity contrast is 'also somewhat interesting'
            contrast = cv2normalize(cv2.Laplacian(yc - yc4,-1))

            # add all together
            total = motion * motion_factor + vmap * color_factor + umap * color_factor + contrast * contrast_factor

            # find successively brightest points in a much lower resolution result
            resized = cv2.resize(total,(covx,covy),interpolation=cv2.INTER_LINEAR)
            scratch = resized.copy()
            points = []
            for i in range(0,4):
                brightest_v = 0.0
                point = Float32XYZ()
                for y in range(0,covy):
                    for x in range(0,covx):
                        if scratch[y,x] > point.z:
                            point.x = x
                            point.y = y
                            point.z = scratch[y,x]
                points.append(point)
                cv2.circle(scratch,(point.x,point.y),20,0,-1)

            # convert to messages and send off
            for point in points:
                msg = Saliency()
                msg.saliency_id = GenerateSaliencyID()
                msg.ts = self.cur_ts
                msg.position.x = -1.0 + 2.0 * (point.x / float(covx))
                msg.position.y = -1.0 + 2.0 * (point.y / float(covy))
                msg.confidence = point.z

                self.saliency_pub.publish(msg)

            if self.debug:
                cv2.imshow("faux saliency",cv2.resize(resized,(subx,suby),interpolation=cv2.INTER_LINEAR))


if __name__ == '__main__':

    rospy.init_node('detect_saliency')
    node = DetectSaliency()
    rospy.spin()
