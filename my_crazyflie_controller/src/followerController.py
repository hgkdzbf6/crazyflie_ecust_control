#!/usr/bin/env python
import rospy
from my_crazyflie_controller.msg import LeaderController
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty


class FollowerController():
    def __init__(self, leader_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        rospy.loginfo("waiting for land service")
        rospy.wait_for_service('land')
        rospy.loginfo("found land service")
        self._land = rospy.ServiceProxy('land', Empty)

        rospy.loginfo("waiting for takeoff service")
        rospy.wait_for_service('takeoff')
        rospy.loginfo("found takeoff service")
        self._takeoff = rospy.ServiceProxy('takeoff', Empty)

        rospy.loginfo("waiting for takeoff2 service")
        rospy.wait_for_service('takeoff2')
        rospy.loginfo("found takeoff2 service")
        self._takeoff2 = rospy.ServiceProxy('takeoff2', Empty)

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self.command_enum()
        # rospy.Subscriber(leader_controller, Joy, self._joyChanged)
        rospy.Subscriber(leader_topic,LeaderController, self._leader_controller)

    def command_enum(self):
        self.idle=0
        self.takeoff=1
        self.land=2
        self.stop=3
        self.takeoff2=4

    def _leader_controller(self,data):
        if data.command==self.idle:
            pass
        elif data.command==self.takeoff:
            self._takeoff()
        elif data.command==self.land:
            self._land()
        elif data.command==self.stop:
            self._emergency()
        elif data.command==self.takeoff2:
            self._takeoff2()

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    # use_controller = rospy.get_param("~use_crazyflie_controller", False)
    leader_topic = rospy.get_param("~leader_topic", "leader_topic")
    controller = FollowerController(leader_topic)
    rospy.spin()
