#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from math import fabs

lastData = None

def joyChanged(data):
    global lastData
    lastData = data
    # print(data)

def circle_generate(index):
    circle = 5
    point_num = 2000
    delta = 0
    radius = 0.8
    offset_x = 2.8
    offset_y = 2
    angle = circle *index* 2 * math.pi / point_num + delta
    x = radius * math.cos(angle) + offset_x
    y = radius * math.sin(angle) + offset_y
    msg=PoseStamped()
    msg.header.seq += 1
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 1.2
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    return msg

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    joy_topic = rospy.get_param("~joy_topic", "joy")
    index=0
    rate = rospy.Rate(r)

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)
    rospy.Subscriber(joy_topic, Joy, joyChanged)

    while not rospy.is_shutdown():
        if index<2000:
            msg=circle_generate(index)
            index=index+1
        pub.publish(msg)
        rospy.sleep(0.02)
