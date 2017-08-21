#!/usr/bin/env python
# -*- coding:utf-8 -*-  

import rospy
import math 
import tf
import numpy as np
import time
from tf import TransformListener
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class LeaderManager():
    def __init__(self):
        rospy.init_node('leaderManager', anonymous=True)
        self.idle=0
        self.leader_takeoff=1
        self.follower1_takeoff=2
        self.follower2_takeoff=3
        self.joy_controll=4
        self.auto_formation=5
        self.stop=6

        self.joy_enabled=False

        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.leaderFrame=rospy.get_param("~leaderFrame","/leader")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.x=rospy.get_param("~x")
        self.y=rospy.get_param("~y")
        self.z=rospy.get_param("~z")
        # self.leaderAdvertise=rospy.Publisher('leaderPosition',PoseStamped,queue_size=1)
        self.listener = TransformListener()
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("/manager_state",Int32,self.stateCallback)
        # 订阅手柄节点
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        self.takeoffFlag = 0
        self.goalIndex = 0
        rospy.loginfo("demo start!!!!!!!")

    def stateCallback(self,data):
        if data.data==self.joy_controll:
            self.joy_enabled= True
        else:
            self.joy_enabled= False
        pass

    def cmdVelCallback(self,data):
        if data.linear.z != 0.0 and self.takeoffFlag==0:
            self.takeoffFlag=1
            rospy.sleep(10)
            self.takeoffFlag=2

    def joy_callback(self,data):
        global lastData
        lastData = data
        pass

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            # 计算目标位置
            if self.joy_enabled==True :
                goal=self.calc_goal(goal)
            else :
                goal=self.static_goal(goal)
            
            self.pubGoal.publish(goal)
            # self.leaderAdvertise.publish(goal)
            # 自己的坐标系和世界坐标系的转换
            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)

    def static_goal(self,goal):
        goal.header.seq += 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x=self.x
        goal.pose.position.y=self.y
        goal.pose.position.z=self.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal

    def calc_goal(self,goal):
        global lastData
        r=0.02
        yaw=0
        goal.header.seq += 1
        goal.header.stamp = rospy.Time.now()
        if lastData != None:
            if math.fabs(lastData.axes[1]) > 0.1:
                goal.pose.position.z += lastData.axes[1] / r / 2
            if math.fabs(lastData.axes[4]) > 0.1:
                goal.pose.position.x += lastData.axes[4] / r * 1
            if math.fabs(lastData.axes[3]) > 0.1:
                goal.pose.position.y += lastData.axes[3] / r * 1
            if math.fabs(lastData.axes[0]) > 0.1:
                yaw += lastData.axes[0] / r * 2

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal
