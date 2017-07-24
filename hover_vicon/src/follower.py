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
    def __init__(self,leaderName):
        rospy.init_node('follower', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        rospy.Subscriber("/"+leaderName+'/leaderPosition',PoseStamped,self.followerSubCB)
        self.listener = TransformListener()
        self.goal=PoseStamped()
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
            rospy.sleep(0.02)

    def followerSubCB(self,goal):
        # rospy.loginfo("info received!")
        self.goal.header.seq += 1
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x=goal.pose.position.x+0.5
        self.goal.pose.position.y=goal.pose.position.y
        self.goal.pose.position.z=goal.pose.position.z
        self.goal.pose.orientation.w=goal.pose.orientation.w
        self.goal.pose.orientation.x=goal.pose.orientation.x
        self.goal.pose.orientation.y=goal.pose.orientation.y
        self.goal.pose.orientation.z=goal.pose.orientation.z
