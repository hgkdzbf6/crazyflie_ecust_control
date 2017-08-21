#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class Follower():
    def __init__(self,leaderFrame):
        rospy.init_node('follower', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        # rospy.Subscriber("/"+leaderName+'/leaderPosition',PoseStamped,self.followerSubCB)
        self.listener = TransformListener()
        self.goal=PoseStamped()
        self.leaderFrame=leaderFrame
        self.takeoffFlag=0
        self.goalIndex = 0
        rospy.loginfo("demo start!!!!!!!")

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        self.goal = PoseStamped()
        self.goal.header.seq = 0
        self.goal.header.frame_id = self.worldFrame
        self.goal.pose.orientation.w=1
        while not rospy.is_shutdown():
            self.pubGoal.publish(self.goal)
            # rospy.loginfo(self.goal)
            # rospy.loginfo(self.worldFrame)
            # rospy.loginfo(self.leaderFrame)
            # rospy.loginfo(self.frame)
            t = self.listener.getLatestCommonTime(self.worldFrame, self.leaderFrame)


            if self.listener.canTransform(self.worldFrame, self.leaderFrame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.leaderFrame, t)
                # rospy.loginfo(position)
                # rospy.loginfo(quaternion)
                self.followerGoalGenerate(position,quaternion)
                rospy.sleep(0.02)

    def followerGoalGenerate(self,position,quaternion):
        # rospy.loginfo("info received!")
        self.goal.header.seq += 1
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x=position[0]+0.4
        self.goal.pose.position.y=position[1]
        # self.goal.pose.position.z=position.z
        self.goal.pose.position.z=0.8
        self.goal.pose.orientation.w=quaternion[3]
        self.goal.pose.orientation.x=quaternion[0]
        self.goal.pose.orientation.y=quaternion[1]
        self.goal.pose.orientation.z=quaternion[2]


