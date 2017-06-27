#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

from __future__ import with_statement
import os
import rospy
import numpy
import time
import cv2
import tf
import geometry_msgs
from dynamic_reconfigure.server import Server
from r2_perception.cfg import VisionConfig
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import sqrt
from r2_perception.msg import Face,Hand,Saliency,FaceRequest,FaceResponse,CandidateUser,CandidateHand,CandidateSaliency
from visualization_msgs.msg import Marker
from threading import Lock
from geometry_msgs.msg import PointStamped


opencv_bridge = CvBridge()

thumb_ext = ".png"

# maximum fuse distance between faces (meters)
face_continuity_threshold_m = 0.5

# maximum fuse distance between hands (meters)
hand_continuity_threshold_m = 0.5

# maximum fuse distance between salient points (meters)
saliency_continuity_threshold_m = 0.3

# minimum confidence needed for an observation to be valid (0..1)
minimum_confidence = 0.4

# maximum extrapolation time (seconds)
time_difference = rospy.Time(1,0)

# number of points needed for full confidence
full_points = 5


serial_number = 0

def GenerateCandidateUserID():
    global serial_number
    result = serial_number
    serial_number += 10
    return result

def GenerateCandidateHandID():
    global serial_number
    result = serial_number
    serial_number += 10
    return result

def GenerateCandidateSaliencyID():
    global serial_number
    result = serial_number
    serial_number += 10
    return result


class CandidateUserPredictor(object):


    def __init__(self):

        self.faces = [] # list of detected Faces
        self.age = 0.0 # estimated age
        self.age_confidence = 0.0 # confidence in age
        self.gender = 0 # estimated gender
        self.gender_confidence = 0.0 # confidence in gender
        self.identity = 0 # recognized user identity
        self.identity_confidence = 0.0 # confidence in identity


    def Extrapolate(self,ts):

        n = len(self.faces)
        if n < 2:
            return self.faces[0]

        # prepare for linear regression
        sumf = Face()
        sumt = 0.0
        sumtt = 0.0
        sumft = Face()

        # iterate over last max_n faces only
        for face in self.faces:

            # time delta
            t = (ts - face.ts).to_sec()

            # face
            sumf.rect.origin.x += face.rect.origin.x
            sumf.rect.origin.y += face.rect.origin.y
            sumf.rect.size.x += face.rect.size.x
            sumf.rect.size.y += face.rect.size.y
            sumf.position.x += face.position.x
            sumf.position.y += face.position.y
            sumf.position.z += face.position.z
            sumf.confidence += face.confidence
            sumf.smile += face.smile
            sumf.frown += face.frown
            # expression strings cannot be extrapolated
            # landmarks might be useful here
            # thumb needs no extrapolation

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # face * time
            sumft.rect.origin.x += face.rect.origin.x * t
            sumft.rect.origin.y += face.rect.origin.y * t
            sumft.rect.size.x += face.rect.size.x * t
            sumft.rect.size.y += face.rect.size.y * t
            sumft.position.x += face.position.x * t
            sumft.position.y += face.position.y * t
            sumft.position.z += face.position.z * t
            sumft.confidence += face.confidence * t
            sumft.smile += face.smile * t
            sumft.frown += face.frown * t

        # face slope
        slpf = Face()
        den = float(n) * sumtt - sumt * sumt
        if den == 0.0:
            return self.faces[0]

        slpf.rect.origin.x = (n * sumft.rect.origin.x - sumt * sumf.rect.origin.x) / den
        slpf.rect.origin.y = (n * sumft.rect.origin.y - sumt * sumf.rect.origin.y) / den
        slpf.rect.size.x = (n * sumft.rect.size.x - sumt * sumf.rect.size.x) / den
        slpf.rect.size.y = (n * sumft.rect.size.y - sumt * sumf.rect.size.y) / den
        slpf.position.x = (n * sumft.position.x - sumt * sumf.position.x) / den
        slpf.position.y = (n * sumft.position.y - sumt * sumf.position.y) / den
        slpf.position.z = (n * sumft.position.z - sumt * sumf.position.z) / den
        slpf.confidence = (n * sumft.confidence - sumt * sumf.confidence) / den
        slpf.smile = (n * sumft.smile - sumt * sumf.smile) / den
        slpf.frown = (n * sumft.frown - sumt * sumf.frown) / den

        # result
        result = Face()
        result.ts = ts
        result.face_id = 0
        result.rect.origin.x = (sumf.rect.origin.x - slpf.rect.origin.x * sumt) / float(n)
        result.rect.origin.y = (sumf.rect.origin.y - slpf.rect.origin.y * sumt) / float(n)
        result.rect.size.x = (sumf.rect.size.x - slpf.rect.size.x * sumt) / float(n)
        result.rect.size.y = (sumf.rect.size.y - slpf.rect.size.y * sumt) / float(n)
        result.position.x = (sumf.position.x - slpf.position.x * sumt) / float(n)
        result.position.y = (sumf.position.y - slpf.position.y * sumt) / float(n)
        result.position.z = (sumf.position.z - slpf.position.z * sumt) / float(n)
        result.confidence = (sumf.confidence - slpf.confidence * sumt) / float(n)
        result.smile = (sumf.smile - slpf.smile * sumt) / float(n)
        result.frown = (sumf.frown - slpf.frown * sumt) / float(n)

        return result


    def PruneBefore(self,ts):

        new_faces = [face for face in self.faces if face.ts.to_sec() >= ts.to_sec()]
        self.faces = new_faces


    def Append(self,face):

        self.faces.append(face)


    def CalculateConfidence(self):

        global full_points

        # calculate confidence
        n = len(self.faces)
        if n > full_points:
            n = full_points
        total = 0.0
        for face in self.faces[-n:]:
            total += face.confidence
        total /= float(full_points)

        return total


class CandidateHandPredictor(object):


    def __init__(self):

        self.hands = [] # list of detected Hands


    def Extrapolate(self,ts):

        n = len(self.hands)
        if n < 2:
            return self.hands[0]

        # prepare for linear regression
        sumh = Hand()
        sumt = 0.0
        sumtt = 0.0
        sumht = Hand()
        gestures = []

        # iterate over last max_n hands only
        for hand in self.hands:

            # time delta
            t = (ts - hand.ts).to_sec()

            # hand
            sumh.position.x += hand.position.x
            sumh.position.y += hand.position.y
            sumh.position.z += hand.position.z
            sumh.confidence += hand.confidence

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # hand * time
            sumht.position.x += hand.position.x * t
            sumht.position.y += hand.position.y * t
            sumht.position.z += hand.position.z * t
            sumht.confidence += hand.confidence * t

            # gestures
            for gesture in hand.gestures:
                if gesture not in gestures:
                    gestures.append(gesture)

        # hand slope
        slph = Hand()
        den = float(n) * sumtt - sumt * sumt
        if den == 0.0:
            return self.hands[0]

        slph.position.x = (n * sumht.position.x - sumt * sumh.position.x) / den
        slph.position.y = (n * sumht.position.y - sumt * sumh.position.y) / den
        slph.position.z = (n * sumht.position.z - sumt * sumh.position.z) / den
        slph.confidence = (n * sumht.confidence - sumt * sumh.confidence) / den

        # result
        result = Hand()
        result.ts = ts
        result.hand_id = 0
        result.position.x = (sumh.position.x - slph.position.x * sumt) / float(n)
        result.position.y = (sumh.position.y - slph.position.y * sumt) / float(n)
        result.position.z = (sumh.position.z - slph.position.z * sumt) / float(n)
        result.confidence = (sumh.confidence - slph.confidence * sumt) / float(n)
        result.gestures = gestures

        return result


    def PruneBefore(self,ts):

        new_hands = [hand for hand in self.hands if hand.ts.to_sec() >= ts.to_sec()]
        self.hands = new_hands



    def Append(self,hand):

        self.hands.append(hand)


    def CalculateConfidence(self):

        global full_points

        # calculate confidence
        n = len(self.hands)
        if n > full_points:
            n = full_points
        total = 0.0
        for hand in self.hands[-n:]:
            total += hand.confidence
        total /= float(full_points)

        return total


class CandidateSaliencyPredictor(object):


    def __init__(self):

        self.saliencies = [] # list of detected Saliencies


    def Extrapolate(self,ts):

        n = len(self.saliencies)
        if n < 2:
            return self.saliencies[0]

        # prepare for linear regression
        sums = Saliency()
        sumt = 0.0
        sumtt = 0.0
        sumst = Saliency()

        # iterate over last max_n saliencies only
        for saliency in self.saliencies:

            # time delta
            t = (ts - saliency.ts).to_sec()

            # saliency
            sums.position.x += saliency.position.x
            sums.position.y += saliency.position.y
            sums.confidence += saliency.confidence

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # saliency * time
            sumst.position.x += saliency.position.x * t
            sumst.position.y += saliency.position.y * t
            sumst.confidence += saliency.confidence * t

        # saliency slope
        slps = Saliency()
        den = float(n) * sumtt - sumt * sumt
        if den == 0.0:
            return self.saliencies[0]

        slps.position.x = (n * sumst.position.x - sumt * sums.position.x) / den
        slps.position.y = (n * sumst.position.y - sumt * sums.position.y) / den
        slps.confidence = (n * sumst.confidence - sumt * sums.confidence) / den

        # result
        result = Saliency()
        result.ts = ts
        result.saliency_id = 0
        result.position.x = (sums.position.x - slps.position.x * sumt) / float(n)
        result.position.y = (sums.position.y - slps.position.y * sumt) / float(n)
        result.confidence = (sums.confidence - slps.confidence * sumt) / float(n)

        return result


    def PruneBefore(self,ts):

        new_saliencies = [saliency for saliency in self.saliencies if saliency.ts.to_sec() >= ts.to_sec()]
        self.saliencies = new_saliencies


    def Append(self,saliency):

        self.saliencies.append(saliency)


    def CalculateConfidence(self):

        global full_points

        # calculate confidence
        n = len(self.saliencies)
        if n > full_points:
            n = full_points
        total = 0.0
        for saliency in self.saliencies[-n:]:
            total += saliency.confidence
        total /= float(full_points)

        return total


class VisionPipeline(object):


    def __init__(self):

        self.lock = Lock()

        self.thumbs_base_dir = rospy.get_param("/thumbs_dir")

        self.debug = rospy.get_param("/debug")
        self.store_thumbs = rospy.get_param("/store_thumbs")
        self.visualization = rospy.get_param("/visualization")
        self.visualize_pipeline = rospy.get_param("/visualize_pipeline")

        self.rotate = rospy.get_param("rotate")
        self.vision_rate = rospy.get_param("vision_rate")
        self.face_regression = rospy.get_param("face_regression")
        self.hand_regression = rospy.get_param("hand_regression")
        self.saliency_regression = rospy.get_param("saliency_regression")

        self.name = rospy.get_namespace().split('/')[-2]

        self.session_tag = str(rospy.get_param("/session_tag"))
        self.session_id = hash(self.session_tag) & 0xFFFFFFFF

        self.camera_id = hash(self.name) & 0xFFFFFFFF

        if self.store_thumbs:
            today_tag = time.strftime("%Y%m%d")
            camera_tag = self.name + "_%08X" % (self.camera_id & 0xFFFFFFFF)
            self.thumb_dir = self.thumbs_base_dir + "/" + today_tag + "/" + self.session_tag + "_%08X/" % (self.session_id & 0xFFFFFFFF) + camera_tag + "/"
            if not os.path.exists(self.thumb_dir):
                os.makedirs(self.thumb_dir)

        self.cusers = {}
        self.chands = {}
        self.csaliencies = {}

        # start listening to transforms
        self.listener = tf.TransformListener()

        # start timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.vision_rate),self.HandleTimer)

        # request/response to/from face analysis node
        self.face_response_sub = rospy.Subscriber("face_response",FaceResponse,self.HandleFaceResponse)
        self.face_request_pub = rospy.Publisher("face_request",FaceRequest,queue_size=5)

        # subscribers for raw face, hand and saliency nodes
        self.face_sub = rospy.Subscriber("raw_face",Face,self.HandleFace)
        self.hand_sub = rospy.Subscriber("raw_hand",Hand,self.HandleHand)
        self.saliency_sub = rospy.Subscriber("raw_saliency",Saliency,self.HandleSaliency)

        # publishers for candidate user, hand and saliency
        self.cuser_pub = rospy.Publisher("cuser",CandidateUser,queue_size=5)
        self.chand_pub = rospy.Publisher("chand",CandidateHand,queue_size=5)
        self.csaliency_pub = rospy.Publisher("csaliency",CandidateSaliency,queue_size=5)

        # rviz markers
        if self.visualization and self.visualize_pipeline:
            self.face_rviz_pub = rospy.Publisher("rviz_face",Marker,queue_size=5)
            self.hand_rviz_pub = rospy.Publisher("rviz_hand",Marker,queue_size=5)
            self.saliency_rviz_pub = rospy.Publisher("rviz_saliency",Marker,queue_size=5)

        #self.config_srv = Server(VisionConfig,self.HandleConfig)

        # for debugging
        if self.debug:
            cv2.startWindowThread()
            cv2.namedWindow(self.name)
            self.frame_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleFrame)


    def HandleConfig(self,data,level):

        return data


    def HandleFace(self,data):

        global face_continuity_threshold_m

        with self.lock:

            if data.ts.secs == 0:
                data.ts = rospy.get_rostime()

            closest_cuser_id = 0
            closest_dist = face_continuity_threshold_m

            for cuser_id in self.cusers:
                if self.face_regression:
                    face = self.cusers[cuser_id].Extrapolate(data.ts)
                else:
                    face = self.cusers[cuser_id].faces[-1]
                dx = data.position.x - face.position.x
                dy = data.position.y - face.position.y
                dz = data.position.z - face.position.z
                d = sqrt(dx * dx + dy * dy + dz * dz)
                if (closest_cuser_id == 0) or (d < closest_dist):
                    closest_cuser_id = cuser_id
                    closest_dist = d

            if closest_dist < face_continuity_threshold_m:

                self.cusers[closest_cuser_id].Append(data)

            else:

                closest_cuser_id = GenerateCandidateUserID()
                cuser = CandidateUserPredictor()
                cuser.Append(data)

                self.cusers[closest_cuser_id] = cuser

                # send face analysis request to face_analysis
                msg = FaceRequest()
                msg.session_id = self.session_id
                msg.camera_id = self.camera_id
                msg.cuser_id = closest_cuser_id
                msg.face_id = data.face_id
                msg.ts = data.ts
                msg.thumb = data.thumb
                self.face_request_pub.publish(msg)

            if self.store_thumbs:
                cuser_tag = "cuser_%08X" % (closest_cuser_id & 0xFFFFFFFF)
                dir = self.thumb_dir + cuser_tag + "/"
                if not os.path.exists(dir):
                    os.makedirs(dir)
                thumb = opencv_bridge.imgmsg_to_cv2(data.thumb)
                face_tag = "face_%08X" % (data.face_id & 0xFFFFFFFF)
                thumb_file = dir + face_tag + thumb_ext
                cv2.imwrite(thumb_file,thumb)


    def HandleHand(self,data):

        global hand_continuity_threshold_m

        with self.lock:

            if data.ts.secs == 0:
                data.ts = rospy.get_rostime()

            closest_chand_id = 0
            closest_dist = hand_continuity_threshold_m

            for chand_id in self.chands:
                if self.hand_regression:
                    hand = self.chands[chand_id].Extrapolate(data.ts)
                else:
                    hand = self.chands[chand_id].hands[-1]
                dx = data.position.x - hand.position.x
                dy = data.position.y - hand.position.y
                dz = data.position.z - hand.position.z
                d = sqrt(dx * dx + dy * dy + dz * dz)
                if (closest_chand_id == 0) or (d < closest_dist):
                    closest_chand_id = chand_id
                    closest_dist = d

            if closest_dist < hand_continuity_threshold_m:

                self.chands[closest_chand_id].Append(data)

            else:

                closest_chand_id = GenerateCandidateHandID()
                chand = CandidateHandPredictor()
                chand.Append(data)

                self.chands[closest_chand_id] = chand


    def HandleSaliency(self,data):

        global saliency_continuity_threshold_m

        with self.lock:

            if data.ts.secs == 0:
                data.ts = rospy.get_rostime()

            closest_csaliency_id = 0
            closest_dist = saliency_continuity_threshold_m

            for csaliency_id in self.csaliencies:
                if self.saliency_regression:
                    saliency = self.csaliencies[csaliency_id].Extrapolate(data.ts)
                else:
                    saliency = self.csaliencies[csaliency_id].saliencies[-1]
                dx = data.position.x - saliency.position.x
                dy = data.position.y - saliency.position.y
                d = sqrt(dx * dx + dy * dy)
                if (closest_csaliency_id == 0) or (d < closest_dist):
                    closest_csaliency_id = csaliency_id
                    closest_dist = d

            if closest_dist < saliency_continuity_threshold_m:

                self.csaliencies[closest_csaliency_id].Append(data)

            else:

                closest_csaliency_id = GenerateCandidateSaliencyID()
                csaliency = CandidateSaliencyPredictor()
                csaliency.Append(data)

                self.csaliencies[closest_csaliency_id] = csaliency


    def HandleFaceResponse(self,data):

        if data.cuser_id not in self.cusers:
            return

        with self.lock:

            self.cusers[data.cuser_id].age = data.age
            self.cusers[data.cuser_id].age_confidence = data.age_confidence
            self.cusers[data.cuser_id].gender = data.gender
            self.cusers[data.cuser_id].gender_confidence = data.gender_confidence
            self.cusers[data.cuser_id].identity = data.identity
            self.cusers[data.cuser_id].identity_confidence = data.identity_confidence


    # for debugging
    def HandleFrame(self,data):

        with self.lock:

            # get image
            subx = 320
            suby = 240
            cvimage = cv2.resize(opencv_bridge.imgmsg_to_cv2(data,"bgr8"),(subx,suby),interpolation=cv2.INTER_LINEAR)

            if self.rotate == 90:
                subx = 240
                suby = 320
                cvimage = cv2.transpose(cvimage)

            elif self.rotate == -90:
                subx = 240
                suby = 320
                cvimage = cv2.transpose(cvimage)
                cvimage = cv2.flip(cvimage,1)

            elif self.rotate == 180:
                cvimage = cv2.flip(cvimage,-1)

            # get time
            ts = rospy.get_rostime()

            # display users
            for cuser_id in self.cusers:

                # the face
                if self.face_regression:
                    face = self.cusers[cuser_id].Extrapolate(ts)
                else:
                    face = self.cusers[cuser_id].faces[-1]
                x = int((0.5 + 0.5 * face.rect.origin.x) * float(subx))
                y = int((0.5 + 0.5 * face.rect.origin.y) * float(suby))
                cv2.circle(cvimage,(x,y),10,(0,0,255),2)

                # annotate with info if available
                label = ""
                if self.cusers[cuser_id].identity_confidence > 0.0:
                    label = self.cusers[cuser_id].identity
                if self.cusers[cuser_id].age_confidence > 0.0:
                    if label != "":
                        label += ", "
                    label += str(int(self.cusers[cuser_id].age)) + " y/o"
                if self.cusers[cuser_id].gender_confidence > 0.0:
                    if label != "":
                        label += ", "
                    if self.cusers[cuser_id].gender == 2:
                        label += "female"
                    else:
                        label += "male"
                cv2.putText(cvimage,label,(x - 20,y + 20),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,255,255))

            # TODO: display hands

            # display salient points
            for csaliency_id in self.csaliencies:

                if self.saliency_regression:
                    point = self.csaliencies[csaliency_id].Extrapolate(ts)
                else:
                    point = self.csaliencies[csaliency_id].saliencies[-1]
                x = int((0.5 + 0.5 * point.position.x) * float(subx))
                y = int((0.5 + 0.5 * point.position.y) * float(suby))
                cv2.circle(cvimage,(x,y),10,(255,0,0),2)

            cv2.imshow(self.name,cvimage)


    def SendUserMarkers(self,frame_id,ts,cuser_id,ns,position):

        # the face
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = ts
        marker.id = cuser_id
        marker.ns = ns
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(2,0)
        marker.frame_locked = False
        self.face_rviz_pub.publish(marker)

        # label under the face
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = ts
        marker.id = cuser_id + 1
        marker.ns = ns
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.MODIFY
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z - 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.7
        marker.color.g = 0.7
        marker.color.b = 0.7
        marker.color.a = 0.5
        if self.cusers[cuser_id].gender == 2:
            gender = "female"
        else:
            gender = "male"
        marker.text = self.name + " candidate {} ({} {})".format(cuser_id,gender,int(self.cusers[cuser_id].age))
        marker.lifetime = rospy.Duration(2,0)
        marker.frame_locked = False
        self.face_rviz_pub.publish(marker)


    def SendHandMarkers(self,frame_id,ts,chand_id,ns,position,gestures):

        # the face
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = ts
        marker.id = chand_id
        marker.ns = ns
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(2,0)
        marker.frame_locked = False
        self.hand_rviz_pub.publish(marker)

        # label under the hand
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = ts
        marker.id = chand_id + 1
        marker.ns = ns
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.MODIFY
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z - 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.7
        marker.color.g = 0.7
        marker.color.b = 0.7
        marker.color.a = 0.5
        marker.text = self.name + " hand: "
        for gesture in gestures:
            marker.text += " " + gesture
        marker.lifetime = rospy.Duration(2,0)
        marker.frame_locked = False
        self.hand_rviz_pub.publish(marker)


    def HandleTimer(self,data):

        global minimum_confidence
        global time_difference

        ts = data.current_expected

        #transform = self.buffer.lookup_transform(self.name,"world",ts)

        with self.lock:

            # mine the current candidate users for confident ones
            for cuser_id in self.cusers:
                conf = self.cusers[cuser_id].CalculateConfidence()
                if conf >= minimum_confidence:

                    # get best candidate user
                    if self.face_regression:
                        cuser = self.cusers[cuser_id].Extrapolate(ts)
                    else:
                        cuser = self.cusers[cuser_id].faces[-1]

                    if self.listener.canTransform("world",self.name,ts):

                        # make a PointStamped structure to satisfy TF
                        ps = PointStamped()
                        ps.header.seq = 0
                        ps.header.stamp = ts
                        ps.header.frame_id = self.name
                        ps.point.x = cuser.position.x
                        ps.point.y = cuser.position.y
                        ps.point.z = cuser.position.z

                        # transform to world coordinates
                        pst = self.listener.transformPoint("world",ps)

                        # setup candidate user message
                        msg = CandidateUser()
                        msg.session_id = self.session_id
                        msg.camera_id = self.camera_id
                        msg.cuser_id = cuser_id
                        msg.ts = ts
                        msg.position.x = pst.point.x
                        msg.position.y = pst.point.y
                        msg.position.z = pst.point.z
                        #msg.position.x = cuser.position.x
                        #msg.position.y = cuser.position.y
                        #msg.position.z = cuser.position.z
                        msg.confidence = cuser.confidence
                        msg.smile = cuser.smile
                        msg.frown = cuser.frown
                        msg.expressions = cuser.expressions
                        #msg.landmarks = cuser.landmarks
                        msg.age = self.cusers[cuser_id].age
                        msg.age_confidence = self.cusers[cuser_id].age_confidence
                        msg.gender = self.cusers[cuser_id].gender
                        msg.gender_confidence = self.cusers[cuser_id].gender_confidence
                        msg.identity = self.cusers[cuser_id].identity
                        msg.identity_confidence = self.cusers[cuser_id].identity_confidence
                        self.cuser_pub.publish(msg)

                    # output markers to rviz
                    if self.visualization and self.visualize_pipeline:
                        self.SendUserMarkers(self.name,ts,cuser_id,"/robot/perception/{}".format(self.name),cuser.position)

            # prune the candidate users and remove them if they disappeared
            to_be_removed = []
            prune_before_time = ts - time_difference
            for cuser_id in self.cusers:
                self.cusers[cuser_id].PruneBefore(prune_before_time)
                if len(self.cusers[cuser_id].faces) == 0:
                    to_be_removed.append(cuser_id)
            for key in to_be_removed:
                del self.cusers[key]


            # mine the current candidate hands for confident ones
            for chand_id in self.chands:
                conf = self.chands[chand_id].CalculateConfidence()
                if conf >= minimum_confidence:

                    # get best candidate hand
                    if self.hand_regression:
                        chand = self.chands[chand_id].Extrapolate(ts)
                    else:
                        chand = self.chands[chand_id].hands[-1]

                    if self.listener.canTransform("world",self.name,ts):

                        # make a PointStamped structure to satisfy TF
                        ps = PointStamped()
                        ps.header.seq = 0
                        ps.header.stamp = ts
                        ps.header.frame_id = self.name
                        ps.point.x = chand.position.x
                        ps.point.y = chand.position.y
                        ps.point.z = chand.position.z

                        # transform to world coordinates
                        pst = self.listener.transformPoint("world",ps)

                        # setup candidate hand message
                        msg = CandidateHand()
                        msg.session_id = self.session_id
                        msg.camera_id = self.camera_id
                        msg.chand_id = chand_id
                        msg.ts = ts
                        #msg.position.x = pst.point.x
                        #msg.position.y = pst.point.y
                        #msg.position.z = pst.point.z
                        msg.position.x = chand.position.x
                        msg.position.y = chand.position.y
                        msg.position.z = chand.position.z
                        msg.confidence = chand.confidence
                        self.chand_pub.publish(msg)

                    # output markers to rviz
                    if self.visualization and self.visualize_pipeline:
                        self.SendHandMarkers(self.name,ts,chand_id,"/robot/perception/{}".format(self.name),chand.position,chand.gestures)

            # prune the candidate hands and remove them if they disappeared
            to_be_removed = []
            prune_before_time = ts - time_difference
            for chand_id in self.chands:
                self.chands[chand_id].PruneBefore(prune_before_time)
                if len(self.chands[chand_id].hands) == 0:
                    to_be_removed.append(chand_id)
            for key in to_be_removed:
                del self.chands[key]


            # mine the current candidate saliencies for confident ones
            for csaliency_id in self.csaliencies:
                conf = self.csaliencies[csaliency_id].CalculateConfidence()
                if conf >= minimum_confidence:
                    msg = CandidateSaliency()
                    msg.session_id = self.session_id
                    msg.camera_id = self.camera_id
                    msg.csaliency_id = csaliency_id
                    msg.ts = ts
                    if self.saliency_regression:
                        csaliency = self.csaliencies[csaliency_id].Extrapolate(ts)
                    else:
                        csaliency = self.csaliencies[csaliency_id].saliencies[-1]
                    msg.position = csaliency.position
                    msg.confidence = csaliency.confidence
                    self.csaliency_pub.publish(msg)

                    # output markers to rviz
                    if self.visualization and self.visualize_pipeline:
                        marker = Marker()
                        marker.header.frame_id = self.name
                        marker.header.stamp = ts
                        marker.id = csaliency_id
                        marker.ns = "/robot/perception/" + self.name
                        marker.type = Marker.SPHERE
                        marker.action = Marker.MODIFY
                        marker.pose.position.x = csaliency.position.x
                        marker.pose.position.y = csaliency.position.y
                        marker.pose.position.z = 0.0
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.1
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                        marker.color.a = 0.5
                        marker.lifetime = rospy.Duration(2,0)
                        marker.frame_locked = False
                        self.saliency_rviz_pub.publish(marker)

            # prune the candidate saliencies and remove them if they disappeared
            to_be_removed = []
            prune_before_time = ts - time_difference
            for csaliency_id in self.csaliencies:
                self.csaliencies[csaliency_id].PruneBefore(prune_before_time)
                if len(self.csaliencies[csaliency_id].saliencies) == 0:
                    to_be_removed.append(csaliency_id)
            for key in to_be_removed:
                del self.csaliencies[key]


if __name__ == '__main__':

    rospy.init_node('vision_pipeline')
    node = VisionPipeline()
    rospy.spin()
