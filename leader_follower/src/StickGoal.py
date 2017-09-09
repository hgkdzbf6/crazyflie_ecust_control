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


lastData = None

class JoyGoal():
    def __init__(self):
        rospy.init_node('stick_goal', anonymous=True)
        # 世界坐标系
        self.stickFrame = rospy.get_param("~stickFrame", "/vicon/G1/G1")
        # 广播消息
        self.pubGoal = rospy.Publisher('/stick_goal', PoseStamped, queue_size=1)
        # tf侦听器
        self.x=1
        self.y=0
        self.z=0
        self.r=50
        self.yaw=0
        self.worldFrame="/world"
        self.msg=PoseStamped()
        self.br = tf.TransformBroadcaster()
        self.listener=tf.TransformListener()

        # 订阅命令topic，来看起飞了没有
    def joy_callback(self,data):
        global lastData
        lastData = data


    def run(self):
        # 要做的事情：
        # 坐标转换，获得角度
        self.listener.waitForTransform(self.worldFrame, self.stickFrame, rospy.Time(), rospy.Duration(5.0))
        
        while not rospy.is_shutdown():
            t = self.listener.getLatestCommonTime("/world", self.stickFrame)
            if self.listener.canTransform(self.worldFrame, self.stickFrame, t):
                msg=PoseStamped()
                self.msg.header.frame_id=self.stickFrame
                self.msg.header.stamp=rospy.Time(0)
                self.msg.pose.position.x=0#pos[0]
                self.msg.pose.position.y=-1.5#pos[1]
                self.msg.pose.position.z=0#pos[2]
                #quaternion=tf.transformations.quaternion_from_euler(0, 0, self.yaw)
                self.msg.pose.orientation.w=1
                self.msg.pose.orientation.x=0
                self.msg.pose.orientation.y=0
                self.msg.pose.orientation.z=0
                msg=self.listener.transformPose("/world", self.msg)

                # self.br.sendTransform(
                #     pos,
                #     quaternion,
                #     rospy.Time.now(),
                #     "stick_goal",
                #     "/world"
                # )


                self.pubGoal.publish(msg)
            rospy.sleep(0.02)



if __name__=="__main__":
    f=JoyGoal()
    f.run()