#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class Demo():
    def __init__(self, goals):
        rospy.init_node('demo', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        self.goals = goals
        self.takeoffFlag=0
        self.goalIndex = 0
        rospy.loginfo("demo start!!!!!!!")
    
    def cmdVelCallback(self,data):
        if data.linear.z != 0.0 and self.takeoffFlag==0:
            self.takeoffFlag=1
            rospy.sleep(10)
            self.takeoffFlag=2

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            self.calc_goal(goal,self.goalIndex)
            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if self.takeoffFlag == 1:
                    self.goalIndex=0

                elif self.takeoffFlag == 2 and self.goalIndex < len(self.goals) - 1:
                    rospy.sleep(self.goals[self.goalIndex][4]*2)
                    rospy.loginfo(self.goalIndex)
                    self.goalIndex += 1

    def calc_goal(self, goal,index):
        goal.header.seq += 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goals[index][0]
        goal.pose.position.y = self.goals[index][1]
        goal.pose.position.z = self.goals[index][2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[index][3])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
