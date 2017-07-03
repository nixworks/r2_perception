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


class SaliencyPredictor(object):


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


    def CalculateConfidence(self,full_points):

        # calculate confidence
        n = len(self.saliencies)
        if n > full_points:
            n = full_points
        total = 0.0
        for saliency in self.saliencies[-n:]:
            total += saliency.confidence
        total /= float(full_points)

        return total
