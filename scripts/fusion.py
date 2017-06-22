#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

from __future__ import with_statement
import os
import rospy
import time
from r2_perception.msg import CandidateUser,CandidateHand,CandidateSaliency,EstablishedUser,EstablishedHand,EstablishedSaliency
from threading import Lock


class Fusion(object):


    def __init__(self):

        # create lock
        self.lock = Lock()

        self.debug = rospy.get_param("/debug")
        self.store_thumbs = rospy.get_param("/store_thumbs")
        self.visualization = rospy.get_param("/visualization")
        self.visualize_pipeline = rospy.get_param("/visualize_pipeline")

        self.rate = 30.0

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),self.HandleTimer)

        self.cuser_subs = []
        self.cuser_subs.append(rospy.Subscriber("lefteye/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("righteye/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("realsense/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("wideangle/cuser",CandidateUser,self.HandleCandidateUser))
        self.chand_subs = []
        self.chand_subs.append(rospy.Subscriber("lefteye/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("righteye/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("realsense/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("wideangle/chand",CandidateHand,self.HandleCandidateHand))
        self.csaliency_subs = []
        self.csaliency_subs.append(rospy.Subscriber("lefteye/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("righteye/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("realsense/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("wideangle/csaliency",CandidateSaliency,self.HandleCandidateSaliency))

        self.euser_pub = rospy.Publisher("user",EstablishedUser,queue_size=5)
        self.ehand_pub = rospy.Publisher("hand",EstablishedHand,queue_size=5)
        self.esaliency_pub = rospy.Publisher("saliency",EstablishedSaliency,queue_size=5)


    def HandleCandidateUser(self,data):

        with self.lock:
            ()
            # add or replace candidate user to list


    def HandleCandidateHand(self,data):
        with self.lock:
            ()
            # add or replace candidate hand to list


    def HandleCandidateSaliency(self,data):

        with self.lock:
            ()
            # add or replace candidate saliency to list


    def HandleTimer(self,data):

        with self.lock:

            ts = data.current_expected

            # fuse candidate faces between pipelines

            # fuse candidate hands between pipelines

            # fuse candidate saliencies between pipelines by calculating shortest vector distance

            # fuse faces and saliencies to improve saliency confidence

            # fuse hands and saliencies to improve saliency confidence

            # fuse sounds and faces

            # fuse sounds and hands

            # fuse sounds and saliency

            # fuse speech and faces

            # fuse speech and saliency

            # output all established stuff


if __name__ == '__main__':


    rospy.init_node('fusion')
    node = Fusion()
    rospy.spin()
