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
        rospy.init_node('joy_goal', anonymous=True)
        # 世界坐标系
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        # 广播消息
        self.pubGoal = rospy.Publisher('joy_goal', PoseStamped, queue_size=1)
        # tf侦听器
        self.x=-1
        self.y=1
        self.z=1
        self.r=50
        self.yaw=0
        self.msg=PoseStamped()
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("/joy",Joy,self.joy_callback)

        # 订阅命令topic，来看起飞了没有
    def joy_callback(self,data):
        global lastData
        lastData = data


    def run(self):
        while not rospy.is_shutdown():
            global lastData
            if lastData != None:
                if math.fabs(lastData.axes[1]) > 0.1:
                    self.z += lastData.axes[1] / self.r / 2
                if math.fabs(lastData.axes[4]) > 0.1:
                    self.x += lastData.axes[4] / self.r * 0.2
                if math.fabs(lastData.axes[3]) > 0.1:
                    self.y += lastData.axes[3] / self.r * 0.2
                if math.fabs(lastData.axes[0]) > 0.1:
                    self.yaw += lastData.axes[0] / self.r * 0.3
            self.br.sendTransform((self.x,self.y,self.z),
            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                rospy.Time.now(),
                "/joy_goal",
                self.worldFrame
                )                
            self.msg.header.frame_id="/joy_goal"
            self.msg.header.stamp=rospy.Time.now()
            self.msg.pose.position.x=self.x
            self.msg.pose.position.y=self.y
            self.msg.pose.position.z=self.z
            quaternion=tf.transformations.quaternion_from_euler(0, 0, self.yaw)
            self.msg.pose.orientation.w=quaternion[3]
            self.msg.pose.orientation.x=quaternion[0]
            self.msg.pose.orientation.y=quaternion[1]
            self.msg.pose.orientation.z=quaternion[2]
            self.pubGoal.publish(self.msg)
            rospy.sleep(0.02)



if __name__=="__main__":
    f=JoyGoal()
    f.run()