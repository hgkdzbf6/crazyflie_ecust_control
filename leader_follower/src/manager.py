#!/usr/bin/env python
# -*- coding:utf-8 -*-  
import rospy
import time
import threading
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_srvs.srv import Empty
# 怎样的控制器的设计才是最好的设计？（算了，能实现的设计吧）
# 减轻控制节点的任务，把解析命令的任务交给下层被控制的节点。
# 这样会带来耦合，但是没办法呢。
# 控制者相应也需要得到反馈信息吧，把反馈信息提供给控制器节点就好吧。
class Manager():
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        self.init_state()
        self.init_formation()
        # 状态变量
        self.state=0
        # 编队形式
        self.formation=1
        # 剧本是这样的，leader先起飞，等飞到指定位置之后，
        # follower1起飞，等飞到指定位置之后，
        # follower2起飞，等飞到指定位置之后，
        # 然后切到手柄控制

        # 暂停标志
        self.pause=0
        # leader的起飞信号，通过给出手柄给出吧，所以要订阅手柄这个主题

        self.leader_node=rospy.get_param("~leader_node")        
        self.follower1_node=rospy.get_param("~follower1_node")
        self.follower2_node=rospy.get_param("~follower2_node")
        # 判断是否到达目标点，也就是判断是否进入下一个状态，有开环控制和闭环控制两种方法：
        # 1、开环控制：
        #     定时一段时间，断言在这个时间的话，目标一定到达了这个位置
        #     不需要其他额外信息，定时10秒后肯定到了目标位置了
        # 2、闭环控制：
        #     当飞机距离目标点有一定时间的话，设定容许范围吧，当飞机在这个范围内一段时间后，认为到了这个位置
        #     比开环控制额外需要知道离目标点的距离。
        #     或者两个都要知道，目标点的位置以及飞机现在的实际位置。
        # 默认是开环控制吧

        # timer的测试吧，timer和sched的区别。
        # timer是不循环的，sched是不循环的。
        # timer的生成：～好吧timer也是不循环呢，那就没必要写了

        # 这个要同时控制
        self.is_open_loop=1

        rospy.loginfo("waiting for leader emergency service %s" % (self.leader_node+"/emergency"))
        rospy.wait_for_service(self.leader_node+'/emergency')
        rospy.loginfo("found leader emergency service")
        self._leader_emergency = rospy.ServiceProxy(self.leader_node+'/emergency', Empty)

        rospy.loginfo("waiting for follower1 emergency service")
        rospy.wait_for_service(self.follower1_node+'/emergency')
        rospy.loginfo("found follower1 emergency service")
        self._follower1_emergency = rospy.ServiceProxy(self.follower1_node+'/emergency', Empty)

        rospy.loginfo("waiting for follower2 emergency service")
        rospy.wait_for_service(self.follower2_node+'/emergency')
        rospy.loginfo("found follower2 emergency service")
        self._follower2_emergency = rospy.ServiceProxy(self.follower2_node+'/emergency', Empty)

        rospy.loginfo("waiting for leader land service")
        rospy.wait_for_service(self.leader_node+'/land')
        rospy.loginfo("found leader land service")
        self._leader_land = rospy.ServiceProxy(self.leader_node+'/land', Empty)

        rospy.loginfo("waiting for follower1 land service")
        rospy.wait_for_service(self.follower1_node+'/land')
        rospy.loginfo("found follower1 land service")
        self._follower1_land = rospy.ServiceProxy(self.follower1_node+'/land', Empty)

        rospy.loginfo("waiting for follower2 land service")
        rospy.wait_for_service(self.follower2_node+'/land')
        rospy.loginfo("found follower2 land service")
        self._follower2_land = rospy.ServiceProxy(self.follower2_node+'/land', Empty)

        rospy.loginfo("waiting for leader takeoff service")
        rospy.wait_for_service(self.leader_node+'/takeoff')
        rospy.loginfo("found leader takeoff service")
        self._leader_takeoff = rospy.ServiceProxy(self.leader_node+'/takeoff', Empty)

        rospy.loginfo("waiting for follower1 takeoff service")
        rospy.wait_for_service(self.follower1_node+'/takeoff')
        rospy.loginfo("found follower1 takeoff service")
        self._follower1_takeoff = rospy.ServiceProxy(self.follower1_node+'/takeoff', Empty)

        rospy.loginfo("waiting for follower2 takeoff service")
        rospy.wait_for_service(self.follower2_node+'/takeoff')
        rospy.loginfo("found follower2 takeoff service")
        self._follower2_takeoff = rospy.ServiceProxy(self.follower2_node+'/takeoff', Empty)

        self.state_pub=rospy.Publisher("/manager_state",Int32,queue_size=1)
        self.pub_data=Int32(0)

        rospy.Subscriber("/joy",Joy,self.joy_callback)
        self._buttons = None
    
    def timer_callback(self,data):
        self.state=data
        rospy.loginfo("now the state is :%d" % self.state)
        pass

    # 把回调函数和控制分开，控制时机放在控制当中吧
    def joy_callback(self,data):        
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self.state != self.emergency:
                    # 降落
                    self.state=self.idle
                if i == 1 and data.buttons[i] == 1 and self.state != self.emergency:
                    # 急停
                    self.state=self.emergency
                if i == 2 and data.buttons[i] == 1 and self.state == self.idle:
                    # 起飞
                    self.state=self.leader_takeoff
                if i == 3 and data.buttons[i] == 1 and self.state != self.emergency:
                    # 变换队形吧
                    self.formation=(self.formation+1)%(self.formation_length-1)+1
                    if self.formation==self.formation_line:
                        rospy.loginfo("formation is line now")
                    elif self.formation==self.formation_circle:
                        rospy.loginfo("formation is circle now")
                    elif self.formation==self.formation_triangle:
                        rospy.loginfo("formation is triangle now")
                    pass
                if i == 4 and data.buttons[i] == 1 and self.state != self.emergency:
                    pass
                if i == 5 and data.buttons[i] == 1 and self.state != self.emergency:
                    pass
        self._buttons = data.buttons


    def init_formation(self):
        self.formation_line=1
        self.formation_triangle=2
        self.formation_circle=3
        self.formation_length=4

    def init_state(self):
        self.idle=0
        self.leader_takeoff=1
        self.follower1_takeoff=2
        self.follower2_takeoff=3
        self.joy_controll=4
        self.auto_formation=5
        self.emergency=6

    # 控制在这里
    def run(self):
        while not rospy.is_shutdown():        
            if self.state==self.emergency:
                # 紧急模式：马上停止吧
                self._leader_emergency()
                self._follower1_emergency()
                self._follower2_emergency()
                return
            elif self.state==self.idle:
                # 空闲模式，一般不会用到
                pass
            elif self.state==self.leader_takeoff:
                self._leader_takeoff()
                self.state=self.idle
                # 10秒后切换为follower1 takeoff模式
                timer=threading.Timer(10.0,self.timer_callback,[self.follower1_takeoff])
                timer.start()
                pass
            elif self.state==self.follower1_takeoff:
                # 10秒后切换为follower2 takeoff模式
                self._follower1_takeoff()
                self.state=self.idle
                timer=threading.Timer(10.0,self.timer_callback,[self.follower2_takeoff])
                timer.start()
                pass
            elif self.state==self.follower2_takeoff:
                # 10秒后切换为joy control模式
                self._follower2_takeoff()
                self.state=self.idle
                timer=threading.Timer(10.0,self.timer_callback,[self.joy_controll])
                timer.start()
                pass
            elif self.state==self.joy_controll:
                # 手柄控制还没写好，放假回来再写！
                pass
            elif self.state==self.idle:
                # 空闲的话什么都不做
                pass
            # 广播出去，休眠0.02秒
            self.pub_data=self.state
            self.state_pub.publish(self.pub_data)
            rospy.sleep(0.02)

if __name__=="__main__":
    m=Manager()
    m.run()