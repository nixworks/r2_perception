#!/usr/bin/env python2.7

import os
import rospy
import numpy
import time
import cv2
import tf
import math
import geometry_msgs
from dynamic_reconfigure.server import Server
from r2_perception.cfg import vision_pipelineConfig as VisionConfig
from math import sqrt
from r2_perception.msg import Face,Hand,Saliency


class UserPredictor(object):


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


    def CalculateConfidence(self,full_points):

        # calculate confidence
        n = len(self.faces)
        if n > full_points:
            n = full_points
        total = 0.0
        for face in self.faces[-n:]:
            total += face.confidence
        total /= float(full_points)

        return total
