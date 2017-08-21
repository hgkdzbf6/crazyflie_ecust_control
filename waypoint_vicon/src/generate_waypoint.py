#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist


class Generate_Point():
    def __init__(self,point_num, delta,radius=1,deltaX=0,deltaY=0):
        rospy.init_node('demo', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        self.delta=delta
        self.point_num=point_num
        self.takeoffFlag = 0
        self.radius=radius
        self.deltaX=deltaX
        self.deltaY=deltaY
        self.goalIndex = 0
        rospy.loginfo("demo start!!!!!!!")

    def cmdVelCallback(self, data):
        if data.linear.z != 0.0 and self.takeoffFlag == 0:
            self.takeoffFlag = 1
            rospy.sleep(10)
            self.takeoffFlag = 2

    def run(self):
        self.listener.waitForTransform(
            self.worldFrame, self.frame, rospy.Time(), rospy.Duration(10.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            self.circle_generate(goal, self.goalIndex)
            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(
                    self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if self.takeoffFlag == 1:
                    self.goalIndex = 0

                elif self.takeoffFlag == 2 and self.goalIndex < self.point_num - 1:
                    rospy.sleep(0.02)
                    rospy.loginfo(self.goalIndex)
                    self.goalIndex += 1

    def circle_generate(self,goal,index):
        circle = 5
        point_num = self.point_num
        delta = self.delta
        radius = self.radius
        offset_x = self.deltaX
        offset_y = self.deltaY
        angle = circle *index* 2 * math.pi / point_num + delta
        x = radius * math.cos(angle) + offset_x
        y = radius * math.sin(angle) + offset_y

        goal.header.seq += 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.8
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, 0)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]