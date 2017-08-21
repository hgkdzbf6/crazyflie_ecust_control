#!/usr/bin/env python
# -*- coding:utf-8 -*-  

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# 叫做Normal的原因：一个节点既能表示为leader，然后又能表示为follower
# 所有的节点都是follower，和leader，我们认为的leader节点也可以是跟随手柄移动的一个目标点。
# 理论上normal节点只需要做两件事情：
# 1 跟随一个目标，这个目标值是一个frame，所以现在的目标是通过小乌龟来学习tf
# 2 广播自身的数据,也就是自己的目标点坐标吧。
#   有必要吗？因为所有的vicon数据全部在tf里面，有没有必要再从飞机上面获取呢？
#   考虑可扩展性还是加上吧
# 当然实际上还要加入自己的控制，要侦听那个manager节点的消息呢
# 3 侦听manager节点的状态消息

class Normal():
    def __init__(self):
        rospy.init_node('follower', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        # rospy.Subscriber("/"+leaderName+'/leaderPosition',PoseStamped,self.followerSubCB)
        self.listener = TransformListener()
        self.goal=PoseStamped()
        # 第一件事情，跟随这个目标，这个目标的格式应该是一个frame
        self.leaderFrame=rospy.get_param("~leaderFrame","")
        self.offsetX=rospy.get_param("~offsetX","0")
        self.offsetY=rospy.get_param("~offsetY","0")
        # self.offsetZ=rospy.get_param("~offsetZ","0")
        # 第二件事情，广播自身的位置吧
        # self.pubAttitude=rospy.Publisher('pose',)
        # 第三件事情，侦听manager节点的状态信息
        self.takeoffFlag = 0
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
        self.goal.pose.position.x=position[0]+float(self.offsetX)
        self.goal.pose.position.y=position[1]+float(self.offsetY)
        # self.goal.pose.position.z=position.z
        self.goal.pose.position.z=0.8
        self.goal.pose.orientation.w=quaternion[3]
        self.goal.pose.orientation.x=quaternion[0]
        self.goal.pose.orientation.y=quaternion[1]
        self.goal.pose.orientation.z=quaternion[2]


if __name__=="__main__":
    f=Normal()
    f.run()