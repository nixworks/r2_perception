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


class UserLink(object):

    def __init__(self):

        self.camera_id = 0
        self.cuser_id = 0


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

        self.session_tag = str(rospy.get_param("/session_tag"))
        self.session_id = hash(self.session_tag) & 0xFFFFFFFF

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
            eusers = []
            ehands = []
            esaliencies = []

            # build user fuse groups
            usergroups = []

            # iterate over all pipeline pairs without duplications
            for camera_id1 in self.cusers:
                for camera_id2 in self.cusers:
                    if camera_id1 < camera_id2:  # the IDs are numeric hashes based on the unique pipeline name

                        print "camera_id1 {} and camera_id2 {}:".format(camera_id1,camera_id2)

                        # iterate over all combinations of users
                        for cuser_id1 in self.cusers[camera_id1]:
                            for cuser_id2 in self.cusers[camera_id2]:

                                print "    cuser_id1 {} and cuser_id2 {}:".format(cuser_id1,cuser_id2)

                                # calculate distance
                                dx = self.cusers[camera_id1][cuser_id1].position.x - self.cusers[camera_id2][cuser_id2].position.x
                                dy = self.cusers[camera_id1][cuser_id1].position.y - self.cusers[camera_id2][cuser_id2].position.y
                                dz = self.cusers[camera_id1][cuser_id1].position.z - self.cusers[camera_id2][cuser_id2].position.z
                                distance = sqrt(dx * dx + dy * dy + dz * dz)

                                print "        distance {} m".format(distance)

                                # if close enough,
                                if distance < user_fuse_distance:

                                    # find existing user group that has camera_id1:cuser_id1
                                    found = -1
                                    for i in range(0,len(usergroups)):
                                        for k in range(0,len(usergroups[i])):
                                            if (usergroups[i][k].camera_id == camera_id1) and (usergroups[i][k].cuser_id == cuser_id1):
                                                found = i
                                                break
                                        if found:
                                            break

                                    # prepare link
                                    link2 = UserLink()
                                    link2.camera_id = camera_id2
                                    link2.cuser_id = cuser_id2

                                    if found != -1:
                                        print "            close enough, add link to existing group"
                                        # add link to existing group
                                        usergroups[found].append(link2)
                                    else:
                                        print "            close enough, create new group for link"
                                        # create new group with these links
                                        group = []
                                        link1 = UserLink()
                                        link1.camera_id = camera_id1
                                        link1.cuser_id = cuser_id1
                                        group.append(link1)
                                        group.append(link2)
                                        usergroups.append(group)

            # create established user from each user group
            for group in usergroups:

                euser = EstablishedUser()
                euser.session_id = self.session_id
                euser.ts = ts
                euser.position.x = 0.0
                euser.position.y = 0.0
                euser.position.z = 0.0
                euser.confidence = 0.0
                euser.smile = 0.0
                euser.frown = 0.0
                euser.expressions = ""
                euser.age = 0.0
                euser.age_confidence = 0.0
                euser.gender = 0
                euser.gender_confidence = 0.0
                euser.identity = 0
                euser.identity_confidence = 0

                print "converting group to established user:"

                n = len(group)
                for link in group:
                    print "    camera_id {}, cuser_id {}, position {},{},{}".format(camera_id,cuser_id,self.cusers[link.camera_id][link.cuser_id].position.x,self.cusers[link.camera_id][link.cuser_id].position.y,self.cusers[link.camera_id][link.cuser_id].position.z)
                    euser.position.x += self.cusers[link.camera_id][link.cuser_id].position.x
                    euser.position.y += self.cusers[link.camera_id][link.cuser_id].position.y
                    euser.position.z += self.cusers[link.camera_id][link.cuser_id].position.z
                    euser.confidence += self.cusers[link.camera_id][link.cuser_id].confidence
                    euser.smile += self.cusers[link.camera_id][link.cuser_id].smile
                    euser.frown += self.cusers[link.camera_id][link.cuser_id].frown
                    euser.expressions += self.cusers[link.camera_id][link.cuser_id].expressions
                    euser.age += self.cusers[link.camera_id][link.cuser_id].age
                    euser.age_confidence += self.cusers[link.camera_id][link.cuser_id].age_confidence

                euser.position.x /= n
                euser.position.y /= n
                euser.position.z /= n
                euser.confidence /= n
                euser.smile /= n
                euser.frown /= n
                euser.age /= n
                euser.age_confidence /= n
                # TODO: gender is the most likely of any of the group
                # TODO: identity is the most likely of any of the group

                eusers.append(euser)

            # create established user for all users not referenced in any group
            for camera_id in self.cusers:
                for cuser_id in self.cusers[camera_id]:

                    # find user in any link of any group
                    found = False
                    for group in usergroups:
                        for link in group:
                            if (link.camera_id == camera_id) and (link.cuser_id == cuser_id):
                                found = True
                                break
                        if found:
                            break

                    # and convert to established user if not found
                    if not found:

                        print "converting single candidate user to established user:"
                        print "    camera_id {}, cuser_id {}, position {},{},{}".format(camera_id,cuser_id,self.cusers[camera_id][cuser_id].position.x,self.cusers[camera_id][cuser_id].position.y,self.cusers[camera_id][cuser_id].position.z)

                        euser = EstablishedUser()
                        euser.session_id = self.session_id
                        euser.ts = ts
                        euser.position.x = self.cusers[camera_id][cuser_id].position.x
                        euser.position.y = self.cusers[camera_id][cuser_id].position.y
                        euser.position.z = self.cusers[camera_id][cuser_id].position.z
                        euser.confidence = self.cusers[camera_id][cuser_id].confidence
                        euser.smile = self.cusers[camera_id][cuser_id].smile
                        euser.frown = self.cusers[camera_id][cuser_id].frown
                        euser.expressions = self.cusers[camera_id][cuser_id].expressions
                        euser.age = self.cusers[camera_id][cuser_id].age
                        euser.age_confidence = self.cusers[camera_id][cuser_id].age_confidence
                        euser.gender = self.cusers[camera_id][cuser_id].gender
                        euser.gender_confidence = self.cusers[camera_id][cuser_id].gender_confidence
                        euser.identity = self.cusers[camera_id][cuser_id].identity
                        euser.identity_confidence = self.cusers[camera_id][cuser_id].identity_confidence
                        eusers.append(euser)


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
            for euser in eusers:
                self.euser_pub.publish(euser)

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
