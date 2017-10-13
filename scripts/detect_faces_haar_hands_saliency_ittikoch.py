#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

# combined Haar face detect, hand detect and saliency detect node

from __future__ import with_statement
import os
import rospy
import numpy
import time
import cv2
import math
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from r2_perception.msg import Face,Hand,Saliency,Float32XYZ
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


def GenerateHandID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


def GenerateSaliencyID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


def cv2normalize(image):
    
    dest = image.copy()
    cv2.normalize(cv2.absdiff(image,0.0),dest,0.0,1.0,cv2.NORM_MINMAX)
    return dest

  
class DetectFacesHaarHandsSaliencyIttiKoch(object):


    # constructor
    def __init__(self):

        # start possible debug window
        cv2.startWindowThread()

        # initialize current and last frame and timestamp
        self.cur_image = Image()
        self.last_image = Image()
        self.cur_ts = 0.0
        self.last_ts = 0.0

        # get pipeline name
        self.name = rospy.get_namespace().split('/')[-2]

        # get fixed parameters
        self.thumb_width = rospy.get_param("/thumb_width")
        self.thumb_height = rospy.get_param("/thumb_height")

        self.haar_cascade_filename = rospy.get_param("/haar_cascade_filename")
        self.face_cascade = cv2.CascadeClassifier(self.haar_cascade_filename)

        # get dynamic parameters
        self.debug_face_detect_flag = rospy.get_param("debug_face_detect_flag")
        self.debug_hand_detect_flag = rospy.get_param("debug_hand_detect_flag")
        self.debug_saliency_detect_flag = rospy.get_param("debug_saliency_detect_flag")
        if self.debug_face_detect_flag or self.debug_hand_detect_flag or self.debug_saliency_detect_flag:
            cv2.namedWindow(self.name + " debug")
            if self.debug_saliency_detect_flag:
                cv2.namedWindow(self.name + " saliency")

        self.face_height = rospy.get_param("face_height")

        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rotate = rospy.get_param("rotate")

        self.detect_rate = rospy.get_param("detect_rate")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.detect_rate),self.HandleTimer)

        self.face_rate_divider = rospy.get_param("face_rate_divider")
        self.hand_rate_divider = rospy.get_param("hand_rate_divider")
        self.saliency_rate_divider = rospy.get_param("saliency_rate_divider")
        self.face_rate_count = 0
        self.hand_rate_count = 0
        self.saliency_rate_count = 0

        self.haar_scale_factor = rospy.get_param("haar_scale_factor")
        self.haar_min_width = rospy.get_param("haar_min_width")
        self.haar_min_height = rospy.get_param("haar_min_height")

        self.ittikoch_reduced_width = rospy.get_param("ittikoch_reduced_width")
        self.ittikoch_reduced_height = rospy.get_param("ittikoch_reduced_height")
        self.ittikoch_gaussian_size = rospy.get_param("ittikoch_gaussian_size")
        self.ittikoch_motion_factor = rospy.get_param("ittikoch_motion_factor") # 0.5
        self.ittikoch_color_factor = rospy.get_param("ittikoch_color_factor") # 0.2
        self.ittikoch_contrast_factor = rospy.get_param("ittikoch_contrast_factor") # 0.1
        self.ittikoch_num_points = rospy.get_param("ittikoch_num_points")
        self.ittikoch_min_level = rospy.get_param("ittikoch_min_level")
        self.ittikoch_eraser_radius = rospy.get_param("ittikoch_eraser_radius")

        # start dynamic reconfigure client from vision_pipeline
        #self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.face_pub = rospy.Publisher("raw_face",Face,queue_size=5)
        self.hand_pub = rospy.Publisher("raw_hand",Hand,queue_size=5)
        self.saliency_pub = rospy.Publisher("raw_saliency",Saliency,queue_size=5)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):

        new_debug_face_detect_flag = data.debug_face_detect_flag
        new_debug_hand_detect_flah = data.debug_hand_detect_flag
        new_debug_saliency_detect_flag = data.debug_saliency_detect_flag
        if new_debug_face_detect_flag != self.debug_face_detect_flag or new_debug_hand_detect_flag != self.debug_hand_detect_flag or new_debug_saliency_detect_flag != self.debug_saliency_detect_flag:
            self.debug_face_detect_flag = new_debug_face_detect_flag
            self.debug_hand_detect_flag = new_debug_hand_detect_flag
            self.debug_saliency_detect_flag = new_debug_saliency_detect_flag
            if self.debug_face_detect_flag or self.debug_hand_detect_flag or self.debug_saliency_detect_flag:
                cv2.namedWindow(self.name + " debug")
                if self.debug_saliency_detect_flag:
                    cv2.namedWindow(self.name + " saliency")
            else:
                cv2.destroyWindow(self.name + " debug")
                cv2.destroyWindow(self.name + " saliency")

        self.face_height = data.face_height

        self.fovy = data.fovy
        self.aspect = data.aspect
        self.rotate = data.rotate

        new_detect_rate = data.detect_rate
        if new_detect_rate != self.detect_rate:
            self.detect_rate = new_detect_rate
            self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.detect_rate),self.HandleTimer)            

        self.face_rate_divider = data.face_rate_divider
        self.hand_rate_divider = data.hand_rate_divider
        self.saliency_rate_divider = data.saliency_rate_divider
        self.face_rate_count = 0
        self.hand_rate_count = 0
        self.saliency_rate_count = 0

        self.haar_scale_factor = data.haar_scale_factor
        self.haar_min_width = data.haar_min_width
        self.haar_min_height = data.haar_min_height

        self.ittikoch_reduced_width = data.ittikoch_reduced_width
        self.ittikoch_reduced_height = data.ittikoch_reduced_height
        self.ittikoch_gaussian_size = data.ittikoch_gaussian_size
        self.ittikoch_motion_factor = data.ittikoch_motion_factor
        self.ittikoch_color_factor = data.ittikoch_color_factor
        self.ittikoch_contrast_factor = data.ittikoch_contrast_factor
        self.ittikoch_num_points = data.ittikoch_num_points
        self.ittikoch_min_level = dat.ittikoch_min_level
        self.ittikoch_eraser_radius = data.ittikoch_eraser_radius


    # when a new camera image arrives
    def HandleImage(self,data):

        # copy the image and update the timestamp
        self.prev_image = self.cur_image
        self.prev_ts = self.cur_ts

        self.cur_image = data
        self.cur_ts = rospy.get_rostime()


    # at detection rate
    def HandleTimer(self,data):

        # if no image is available, exit
        if self.cur_ts == 0.0:
            return
        if self.prev_ts == 0.0:
            return

        # convert image from ROS to OpenCV, unrotate and rescale
        image = opencv_bridge.imgmsg_to_cv2(self.cur_image,"bgr8")
        previmage = opencv_bridge.imgmsg_to_cv2(self.prev_image,"bgr8")
        width = image.shape[1]
        height = image.shape[0]

        # arbitrary rotation around center by self.rotate
        rotmat = cv2.getRotationMatrix2D((width / 2,height / 2),self.rotate,1.0)
        rotimage = cv2.warpAffine(image,rotmat,(width,height))
        rotprevimage = cv2.warpAffine(previmage,rotmat,(width,height))

        # calculate distance to camera plane (after rotation, so it's a bit off... TODO)
        cpd = 1.0 / math.tan(self.fovy)

        self.face_rate_count += 1
        if self.face_rate_count >= self.face_rate_divider:

            self.face_rate_count = 0

            # detect all faces in the image
            faces = self.face_cascade.detectMultiScale(rotimage,scaleFactor=self.haar_scale_factor,minSize=(self.haar_min_width,self.haar_min_height),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

            # iterate over all found faces
            for (x,y,w,h) in faces:

                # if the face actually doesn't exist, continue with next face
                if (w <= 0) or (h <= 0):
                    continue

                # calculate distance of the face to the camera
                cx = float(self.face_height) * cpd * float(sizey) / float(h)

                # convert camera coordinates to normalized coordinates on the camera plane
                fy = 1.0 - 2.0 * float(x + w / 2) / float(sizex)
                fz = 1.0 - 2.0 * float(y + h / 2) / float(sizey)

                # project to face distance
                cy = cx * fy / cpd
                cz = cx * fz / cpd

                # prepare raw face message
                msg = Face()
                msg.face_id = GenerateFaceID()
                msg.ts = self.cur_ts
                msg.rect.origin.x = -fy
                msg.rect.origin.y = -fz
                msg.rect.size.x = 2.0 * float(w) / float(sizex)
                msg.rect.size.y = 2.0 * float(h) / float(sizey)
                msg.position.x = cx
                msg.position.y = cy
                msg.position.z = cz
                msg.confidence = 1.0
                msg.smile = 0.0
                msg.frown = 0.0
                msg.expressions = []
                msg.landmarks = []

                # cut out face thumbnail and resize
                cvthumb = cv2.resize(rotimage[y:y+h,x:x+w],(self.thumb_width,self.thumb_height))

                # convert thumbnail from OpenCV to ROS
                msg.thumb = opencv_bridge.cv2_to_imgmsg(cvthumb,encoding="8UC3")

                # and publish the raw face
                self.face_pub.publish(msg)


        self.hand_rate_count += 1
        if self.hand_rate_count >= self.hand_rate_divider:

            self.hand_rate_count = 0

            # detect all hands in the image
            ()


        self.saliency_rate_count += 1
        if self.saliency_rate_count >= self.saliency_rate_divider:

            self.saliency_rate_count = 0

            # convert from BGR to YUV (which is more natural for vision tasks)
            yuv_cur_image = cv2.cvtColor(rotimage,cv2.COLOR_BGR2YUV)
            yuv_cur_image = yuv_cur_image.astype("float32")
            yuv_last_image = cv2.cvtColor(rotprevimage,cv2.COLOR_BGR2YUV)
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

            # find successively brightest points in a much lower resolution result
            reduced = cv2.resize(total,(self.ittikoch_reduced_width,self.ittikoch_reduced_height),interpolation=cv2.INTER_LINEAR)
            scratch = reduced.copy()
            points = []
            for i in range(0,self.ittikoch_num_points):
                point = Float32XYZ()
                point.z = self.ittikoch_min_level
                found = False
                for y in range(0,self.ittikoch_reduced_height):
                    for x in range(0,self.ittikoch_reduced_width):
                        if scratch[y,x] > point.z:
                            point.x = float(x) / float(self.ittikoch_reduced_width)
                            point.y = float(y) / float(self.ittikoch_reduced_height)
                            point.z = scratch[y,x]
                            found = True
                if found:
                    points.append(point)
                    #cv2.circle(scratch,(int(point.x * float(rwidth)),int(point.y * float(rheight))),self.ittikoch_eraser_radius,0,-1)

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
                x = int(point.x * float(self.ittikoch_reduced_width))
                y = int(point.y * float(self.ittikoch_reduced_height))

                # prepare saliency message
                msg = Saliency()
                msg.saliency_id = GenerateSaliencyID()
                msg.ts = self.cur_ts
                msg.direction.x = fx
                msg.direction.y = fy
                msg.direction.z = fz
                msg.screen.x = point.x
                msg.screen.y = point.y
                msg.motion = motion_image[y,x] * self.ittikoch_motion_factor
                msg.umap = umap_image[y,x] * self.ittikoch_color_factor
                msg.vmap = vmap_image[y,x] * self.ittikoch_color_factor
                msg.contrast = contrast_image[y,x] * self.ittikoch_contrast_factor
                msg.confidence = 1.0
                self.saliency_pub.publish(msg)

                if self.debug_saliency_detect_flag:
                    cv2.imshow(self.name + " saliency",cv2.resize(reduced,(sizex,sizey),interpolation=cv2.INTER_LINEAR))


        if self.debug_face_detect_flag or self.debug_hand_detect_flag or self.debug_saliency_detect_flag:
            cv2.imshow(self.name + " debug",rotimage)
            # TODO: show found faces and hands


if __name__ == '__main__':

    rospy.init_node('detect_faces_hands_saliency')
    node = DetectFacesHaarHandsSaliencyIttiKoch()
    rospy.spin()
