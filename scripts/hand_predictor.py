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


class HandPredictor(object):


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


    def CalculateConfidence(self,full_points):

        # calculate confidence
        n = len(self.hands)
        if n > full_points:
            n = full_points
        total = 0.0
        for hand in self.hands[-n:]:
            total += hand.confidence
        total /= float(full_points)

        return total
