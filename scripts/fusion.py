#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

from __future__ import with_statement
import os
import rospy
import time
from r2_perception.msg import CandidateFace,CandidateHand,CandidateSaliency,EstablishedFace,EstablishedHand,EstablishedSaliency
from threading import Lock


class FaceLink(object):

    def __init__(self):

        self.camera_id = 0
        self.cface_id = 0


class Fusion(object):


    def __init__(self):

        # prepare observations
        self.cfaces = {}
        self.chands = {}
        self.csaliencies = {}

        # create lock
        self.lock = Lock()

        # get fixed parameters
        self.store_thumbs_flag = rospy.get_param("/store_thumbs_flag")

        self.session_tag = str(rospy.get_param("/session_tag"))
        self.session_id = hash(self.session_tag) & 0xFFFFFFFF

        #self.visualize = rospy.get_param("/visualize")

        # get dynamic parameters (TODO: dynamic reconfigure for fusion too)
        self.fusion_rate = rospy.get_param("fusion_rate")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fusion_rate),self.HandleTimer)

        self.cface_subs = []
        self.cface_subs.append(rospy.Subscriber("lefteye/cface",CandidateFace,self.HandleCandidateFace))
        self.cface_subs.append(rospy.Subscriber("righteye/cface",CandidateFace,self.HandleCandidateFace))
        self.cface_subs.append(rospy.Subscriber("realsense/cface",CandidateFace,self.HandleCandidateFace))
        self.cface_subs.append(rospy.Subscriber("wideangle/cface",CandidateFace,self.HandleCandidateFace))

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

        self.eface_pub = rospy.Publisher("face",EstablishedFace,queue_size=5)
        self.ehand_pub = rospy.Publisher("hand",EstablishedHand,queue_size=5)
        self.esaliency_pub = rospy.Publisher("saliency",EstablishedSaliency,queue_size=5)


    def HandleCandidateFace(self,data):

        with self.lock:

            # for now, just take the most recent candidate face
            if data.camera_id not in self.cfaces:
                self.cfaces[data.camera_id] = {}
            self.cfaces[data.camera_id][data.cface_id] = data


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
            efaces = []
            ehands = []
            esaliencies = []

            # build face fuse groups
            facegroups = []

            # iterate over all pipeline pairs without duplications
            for camera_id1 in self.cfaces:
                for camera_id2 in self.cfaces:
                    if camera_id1 < camera_id2:  # the IDs are numeric hashes based on the unique pipeline name

                        print "camera_id1 {} and camera_id2 {}:".format(camera_id1,camera_id2)

                        # iterate over all combinations of faces
                        for cface_id1 in self.cfaces[camera_id1]:
                            for cface_id2 in self.cfaces[camera_id2]:

                                print "    cface_id1 {} and cface_id2 {}:".format(cface_id1,cface_id2)

                                # calculate distance
                                dx = self.cfaces[camera_id1][cface_id1].position.x - self.cfaces[camera_id2][cface_id2].position.x
                                dy = self.cfaces[camera_id1][cface_id1].position.y - self.cfaces[camera_id2][cface_id2].position.y
                                dz = self.cfaces[camera_id1][cface_id1].position.z - self.cfaces[camera_id2][cface_id2].position.z
                                distance = sqrt(dx * dx + dy * dy + dz * dz)

                                print "        distance {} m".format(distance)

                                # if close enough,
                                if distance < face_fuse_distance:

                                    # find existing face group that has camera_id1:face_id1
                                    found = -1
                                    for i in range(0,len(facegroups)):
                                        for k in range(0,len(facegroups[i])):
                                            if (facegroups[i][k].camera_id == camera_id1) and (facegroups[i][k].cface_id == cface_id1):
                                                found = i
                                                break
                                        if found:
                                            break

                                    # prepare link
                                    link2 = FaceLink()
                                    link2.camera_id = camera_id2
                                    link2.cface_id = cface_id2

                                    if found != -1:
                                        print "            close enough, add link to existing group"
                                        # add link to existing group
                                        facegroups[found].append(link2)
                                    else:
                                        print "            close enough, create new group for link"
                                        # create new group with these links
                                        group = []
                                        link1 = FaceLink()
                                        link1.camera_id = camera_id1
                                        link1.cface_id = cface_id1
                                        group.append(link1)
                                        group.append(link2)
                                        facegroups.append(group)

            # create established face from each face group
            for group in facegroups:

                eface = EstablishedFace()
                eface.session_id = self.session_id
                eface.ts = ts
                eface.position.x = 0.0
                eface.position.y = 0.0
                eface.position.z = 0.0
                eface.confidence = 0.0
                eface.smile = 0.0
                eface.frown = 0.0
                eface.expressions = ""
                eface.age = 0.0
                eface.age_confidence = 0.0
                eface.gender = 0
                eface.gender_confidence = 0.0
                eface.identity = 0
                eface.identity_confidence = 0

                print "converting group to established face:"

                n = len(group)
                for link in group:
                    print "    camera_id {}, cface_id {}, position {},{},{}".format(camera_id,cface_id,self.cfaces[link.camera_id][link.cface_id].position.x,self.cfaces[link.camera_id][link.cface_id].position.y,self.cfaces[link.camera_id][link.cface_id].position.z)
                    eface.position.x += self.cfaces[link.camera_id][link.cface_id].position.x
                    eface.position.y += self.cfaces[link.camera_id][link.cface_id].position.y
                    eface.position.z += self.cfaces[link.camera_id][link.cface_id].position.z
                    eface.confidence += self.cfaces[link.camera_id][link.cface_id].confidence
                    eface.smile += self.cfaces[link.camera_id][link.cface_id].smile
                    eface.frown += self.cfaces[link.camera_id][link.cface_id].frown
                    eface.expressions += self.cfaces[link.camera_id][link.cface_id].expressions
                    eface.age += self.cfaces[link.camera_id][link.cface_id].age
                    eface.age_confidence += self.cfaces[link.camera_id][link.cface_id].age_confidence

                eface.position.x /= n
                eface.position.y /= n
                eface.position.z /= n
                eface.confidence /= n
                eface.smile /= n
                eface.frown /= n
                eface.age /= n
                eface.age_confidence /= n
                # TODO: gender is the most likely of any of the group
                # TODO: identity is the most likely of any of the group

                efaces.append(eface)

            # create established face for all faces not referenced in any group
            for camera_id in self.cfaces:
                for cface_id in self.cfaces[camera_id]:

                    # find face in any link of any group
                    found = False
                    for group in facegroups:
                        for link in group:
                            if (link.camera_id == camera_id) and (link.cface_id == cface_id):
                                found = True
                                break
                        if found:
                            break

                    # and convert to established face if not found
                    if not found:

                        print "converting single candidate face to established face:"
                        print "    camera_id {}, cface_id {}, position {},{},{}".format(camera_id,cface_id,self.cfaces[camera_id][cface_id].position.x,self.cfaces[camera_id][cface_id].position.y,self.cfaces[camera_id][cface_id].position.z)

                        eface = EstablishedFace()
                        eface.session_id = self.session_id
                        eface.ts = ts
                        eface.position.x = self.cfaces[camera_id][cface_id].position.x
                        eface.position.y = self.cfaces[camera_id][cface_id].position.y
                        eface.position.z = self.cfaces[camera_id][cface_id].position.z
                        eface.confidence = self.cfaces[camera_id][cface_id].confidence
                        eface.smile = self.cfaces[camera_id][cface_id].smile
                        eface.frown = self.cfaces[camera_id][cface_id].frown
                        eface.expressions = self.cfaces[camera_id][cface_id].expressions
                        eface.age = self.cfaces[camera_id][cface_id].age
                        eface.age_confidence = self.cfaces[camera_id][cface_id].age_confidence
                        eface.gender = self.cfaces[camera_id][cface_id].gender
                        eface.gender_confidence = self.cfaces[camera_id][cface_id].gender_confidence
                        eface.identity = self.cfaces[camera_id][cface_id].identity
                        eface.identity_confidence = self.cfaces[camera_id][cface_id].identity_confidence
                        efaces.append(eface)


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
            for eface in efaces:
                self.eface_pub.publish(eface)

            # TODO: send markers to RViz
            #if self.visualize:
            #    ()

            #prune_before_time = ts - self.face_keep_time

            # clean out old faces from self.cfaces
            for camera_id in self.cfaces:
                for cface_id in self.cfaces[camera_id]:
                    ()
                    #if self.cfaces[camera_id][cface_id].ts.to_sec() < prune_before_time:

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
