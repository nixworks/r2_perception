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
import math
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
    
    dest = image.copy()
    cv2.normalize(cv2.absdiff(image,0.0),dest,0.0,1.0,cv2.NORM_MINMAX)
    return dest

  
class DetectSaliencyIttiKoch(object):


    # constructor
    def __init__(self):

        # create lock
        self.lock = Lock()

        # start possible debug window
        cv2.startWindowThread()

        # initialize current and last frame and timestamps
        self.cur_image = Image()
        self.last_image = Image()
        self.cur_ts = 0.0
        self.last_ts = 0.0

        # get pipeline name
        self.name = rospy.get_namespace().split('/')[-2]

        # get parameters
        self.debug_saliency_detect_flag = rospy.get_param("debug_saliency_detect_flag")
        if self.debug_saliency_detect_flag:
            cv2.namedWindow(self.name + " saliency")

        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")

        self.saliency_detect_rate = rospy.get_param("saliency_detect_rate")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.saliency_detect_rate),self.HandleTimer)

        self.saliency_detect_work_width = rospy.get_param("saliency_detect_work_width")
        self.saliency_detect_work_height = rospy.get_param("saliency_detect_work_height")

        self.ittikoch_reduced_width = rospy.get_param("ittikoch_reduced_width")
        self.ittikoch_reduced_height = rospy.get_param("ittikoch_reduced_height")
        self.ittikoch_gaussian_size = rospy.get_param("ittikoch_gaussian_size")
        self.ittikoch_motion_factor = rospy.get_param("ittikoch_motion_factor") # 0.5
        self.ittikoch_color_factor = rospy.get_param("ittikoch_color_factor") # 0.2
        self.ittikoch_contrast_factor = rospy.get_param("ittikoch_contrast_factor") # 0.1
        self.ittikoch_num_points = rospy.get_param("ittikoch_num_points")
        self.ittikoch_eraser_radius = rospy.get_param("ittikoch_eraser_radius")

        # start dynamic reconfigure client from vision_pipeline
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.saliency_pub = rospy.Publisher("raw_saliency",Saliency,queue_size=5)


    def HandleConfig(self,data):

        new_debug_saliency_detect_flag = data.debug_saliency_detect_flag
        if new_debug_saliency_detect_flag != self.debug_saliency_detect_flag:
            self.debug_saliency_detect_flag = new_debug_saliency_detect_flag
            if self.debug_saliency_detect_flag:
                cv2.namedWindow(self.name + " saliency")
            else:
                cv2.destroyWindow(self.name + " saliency")

        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate

        new_saliency_detect_rate = data.saliency_detect_rate
        if new_saliency_detect_rate != self.saliency_detect_rate:
            self.saliency_detect_rate = new_saliency_detect_rate
            self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.saliency_detect_rate),self.HandleTimer)

        self.saliency_detect_work_width = data.saliency_detect_work_width
        self.saliency_detect_work_height = data.saliency_detect_work_height

        self.ittikoch_reduced_width = data.ittikoch_reduced_width
        self.ittikoch_reduced_height = data.ittikoch_reduced_height
        self.ittikoch_gaussian_size = data.ittikoch_gaussian_size
        self.ittikoch_motion_factor = data.ittikoch_motion_factor
        self.ittikoch_color_factor = data.ittikoch_color_factor
        self.ittikoch_contrast_factor = data.ittikoch_contrast_factor
        self.ittikoch_num_points = data.ittikoch_num_points
        self.ittikoch_eraser_radius = data.ittikoch_eraser_radius


    # when an new camera image arrives
    def HandleImage(self,data):

        with self.lock:

            # copy the images and update the timestamps
            self.last_image = self.cur_image
            self.last_ts = self.cur_ts

            self.cur_image = data
            self.cur_ts = rospy.get_rostime()


    # at saliency detection rate
    def HandleTimer(self,data):

        with self.lock:

            # if no images available, exit
            if (self.cur_ts == 0.0) or (self.last_ts == 0):
                return

            # calculate distance to camera plane
            cpd = 1.0 / math.tan(self.fovy)

            # cache feature map size (change according to rotation)
            wwidth = self.saliency_detect_work_width
            wheight = self.saliency_detect_work_height
            rwidth = self.ittikoch_reduced_width
            rheight = self.ittikoch_reduced_height

            # convert images from ROS to OpenCV, unrotate and rescale
            bgr_cur_image = cv2.resize(opencv_bridge.imgmsg_to_cv2(self.cur_image),(wwidth,wheight),interpolation=cv2.INTER_LINEAR)
            bgr_last_image = cv2.resize(opencv_bridge.imgmsg_to_cv2(self.last_image),(wwidth,wheight),interpolation=cv2.INTER_LINEAR)
            if self.rotate == 90:
                wwidth = self.saliency_detect_work_height
                wheight = self.saliency_detect_work_width
                rwidth = self.ittikoch_reduced_height
                rheight = self.ittikoch_reduced_width
                cpd /= self.aspect
                bgr_cur_image = cv2.transpose(bgr_cur_image)
                bgr_last_image = cv2.transpose(bgr_last_image)
            elif self.rotate == -90:
                wwidth = self.saliency_detect_work_height
                wheight = self.saliency_detect_work_width
                rwidth = self.ittikoch_reduced_height
                rheight = self.ittikoch_reduced_width
                cpd /= self.aspect
                bgr_cur_image = cv2.transpose(bgr_cur_image)
                bgr_cur_image = cv2.flip(bgr_cur_image,1)
                bgr_last_image = cv2.transpose(bgr_last_image)
                bgr_last_image = cv2.flip(bgr_last_image,1)
            elif self.rotate == 180:
                bgr_cur_image = cv2.flip(bgr_cur_image,1)
                bgr_cur_image = cv2.flip(bgr_cur_image,-1)
                bgr_last_image = cv2.flip(bgr_last_image,1)
                bgr_last_image = cv2.flip(bgr_last_image,-1)

            # also, convert from BGR to YUV (which is more natural for vision tasks)
            yuv_cur_image = cv2.cvtColor(bgr_cur_image,cv2.COLOR_BGR2YUV)
            yuv_cur_image = yuv_cur_image.astype("float32")
            yuv_last_image = cv2.cvtColor(bgr_last_image,cv2.COLOR_BGR2YUV)
            yuv_last_image = yuv_last_image.astype("float32")

            # generate motion map from difference between last frame and current frame
            dyuv_image = cv2.subtract(yuv_cur_image,yuv_last_image)

            # split intensity and color maps
            y_image,u_image,v_image = cv2.split(yuv_cur_image) 
            dy_image,du_image,dv_image = cv2.split(dyuv_image)

            # use Gaussian blur to create low-pass filtered versions of each map
            y_lowpass_image = cv2.GaussianBlur(y_image,(self.ittikoch_gaussian_size,self.ittikoch_gaussian_size),0) # intensity
            u_lowpass_image = cv2.GaussianBlur(u_image,(self.ittikoch_gaussian_size,self.ittikoch_gaussian_size),0) # red-green
            v_lowpass_image = cv2.GaussianBlur(v_image,(self.ittikoch_gaussian_size,self.ittikoch_gaussian_size),0) # blue-yellow
            dy_lowpass_image = cv2.GaussianBlur(dy_image,(self.ittikoch_gaussian_size,self.ittikoch_gaussian_size),0) # motion

            # emulate high-pass filtering (high-frequency content is the 'most interesting') by
            # subtracting low-pass images from unfiltered images

            # furthermore, for the non-motion maps, use Laplacian as optical derivative

            # motion image
            motion_image = cv2normalize(dy_image - dy_lowpass_image)

            # color responses
            vmap_image = cv2normalize(cv2.Laplacian(v_image - v_lowpass_image,-1))
            umap_image = cv2normalize(cv2.Laplacian(u_image - u_lowpass_image,-1))

            # intensity contrast
            contrast_image = cv2normalize(cv2.Laplacian(y_image - y_lowpass_image,-1))

            # add all together
            total = motion_image * self.ittikoch_motion_factor + vmap_image * self.ittikoch_color_factor + umap_image * self.ittikoch_color_factor + contrast_image * self.ittikoch_contrast_factor

            cpd = 1.0 / math.tan(self.fovy)

            # find successively brightest points in a much lower resolution result
            reduced = cv2.resize(total,(rwidth,rheight),interpolation=cv2.INTER_LINEAR)
            scratch = reduced.copy()
            points = []
            for i in range(0,self.ittikoch_num_points):
                brightest_v = 0.0
                point = Float32XYZ()
                for y in range(0,rheight):
                    for x in range(0,rwidth):
                        if scratch[y,x] > point.z:
                            point.x = float(x) / float(rwidth)
                            point.y = float(y) / float(rheight)
                            point.z = scratch[y,x]
                points.append(point)
                cv2.circle(scratch,(int(point.x * float(rwidth)),int(point.y * float(rheight))),self.ittikoch_eraser_radius,0,-1)

            # convert to messages and send off
            for point in points:

                # convert camera coordinates to normalized coordinates on the camera plane
                fy = (1.0 - 2.0 * point.x) / cpd
                fz = (1.0 - 2.0 * point.y) / cpd
                fx = 1.0

                # normalize
                flen = math.sqrt(fx * fx + fy * fy + fz * fz)
                if flen != 0.0:
                    fx /= flen
                    fy /= flen
                    fz /= flen

                # get coordinates on feature maps to grab actual features
                x = int(point.x * float(wwidth))
                y = int(point.y * float(wheight))

                # prepare saliency message
                msg = Saliency()
                msg.saliency_id = GenerateSaliencyID()
                msg.ts = self.cur_ts
                msg.direction.x = fx
                msg.direction.y = fy
                msg.direction.z = fz
                msg.motion = motion_image[y,x] * self.ittikoch_motion_factor
                msg.umap = umap_image[y,x] * self.ittikoch_color_factor
                msg.vmap = vmap_image[y,x] * self.ittikoch_color_factor
                msg.contrast = contrast_image[y,x] * self.ittikoch_contrast_factor
                msg.confidence = 1.0
                self.saliency_pub.publish(msg)

            if self.debug_saliency_detect_flag:
                cv2.imshow(self.name + " saliency",cv2.resize(reduced,(wwidth,wheight),interpolation=cv2.INTER_LINEAR))


if __name__ == '__main__':

    rospy.init_node('detect_saliency')
    node = DetectSaliencyIttiKoch()
    rospy.spin()
