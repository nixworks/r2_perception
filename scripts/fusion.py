#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

from __future__ import with_statement
import os
import rospy
import time
from r2_perception.msg import CandidateUser,CandidateHand,CandidateSaliency,EstablishedUser,EstablishedHand,EstablishedSaliency
from threading import Lock


# maximum keep time (seconds)
time_difference = rospy.Time(1,0)


class Fusion(object):


    def __init__(self):

        # prepare observations
        self.cusers = {}
        self.chands = {}
        self.csaliencies = {}

        # create lock
        self.lock = Lock()

        # get parameters
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

            # for now, just take the most recent candidate user
            if data.camera_id not in self.cusers:
                self.cusers[data.camera_id] = {}
            self.cusers[data.camera_id][data.cuser_id] = data


    def HandleCandidateHand(self,data):

        with self.lock:

            # for now, just take the most recent candidate hand
            if data.camera_id not in self.chands:
                self.chands[data.camera_id] = {}
            self.chands[data.camera_id][data.chand_id] = data


    def HandleCandidateSaliency(self,data):

        with self.lock:

            # for now, just take the most recent candidate salient point
            if data.camera_id not in self.csaliencies:
                self.csaliencies[data.camera_id] = {}
            self.csaliencies[data.camera_id][data.csaliency_id] = data


    def HandleTimer(self,data):

        with self.lock:

            ts = data.current_expected

            # prepare established observations
            users = []
            hands = []
            saliencies = []

            # put candidate users in user list and fuse if close enough
            for camera_id in self.cusers:
                for cuser_id in self.cusers[camera_id]:
                    ()
                    # check existing users and either fuse or add

            # fuse candidate hands between pipelines
            for camera_id in self.chands:
                for chand_id in self.chands[camera_id]:
                    ()
                    # check existing hands and either fuse or add

            # fuse candidate salient points between pipelines by calculating shortest vector distance
            for camera_id in self.csaliencies:
                for csaliency_id in self.csaliencies[camera_id]:
                    ()
                    # check existing salient points and combine; here just match the vectors, only keep fused salient points for now

            # fuse faces and saliencies to improve saliency confidence
            # TODO: compare salient points and faces to improve faces

            # fuse hands and saliencies to improve saliency confidence
            # TODO: compare salient points and hands to improve hands

            # fuse sounds and faces

            # fuse sounds and hands

            # fuse sounds and saliency

            # output all established stuff

            # TODO: send markers to RViz
            if self.visualization:
                ()

            prune_before_time = ts - time_difference

            # clean out old users from self.cusers
            for camera_id in self.cusers:
                for cuser_id in self.cusers[camera_id]:
                    ()
                    #if self.cusers[camera_id][cuser_id].ts.to_sec() < prune_before_time:

            # clean out old hands from self.chands
            for camera_id in self.chands:
                for chand_id in self.chands[camera_id]:
                    ()
                    #if self.chands[camera_id][chand_id].ts.to_sec() < prune_before_time:

            # clean out old saliencies from self.csaliencies
            for camera_id in self.csaliencies:
                for csaliency_id in self.csaliencies[camera_id]:
                    ()
                    #if self.csaliencies[camera_id][csaliency_id].ts.to_sec() < prune_before_time:


if __name__ == '__main__':


    rospy.init_node('fusion')
    node = Fusion()
    rospy.spin()
