#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from my_crazyflie_controller.msg import leader_controller
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class leaderController():
    def __init__(self, joy_topic):
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

        self.command_enum()
        self._controller=leader_controller()

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        self._pub=rospy.Publisher("leaderCommand",leader_controller,queue_size=1)
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def command_enum(self):
        self.idle=0
        self.takeoff=1
        self.land=2
        self.stop=3
        self.takeoff2=4

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                    self._controller.command=self.land
                    self._pub.publish(self._controller)
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()                    
                    self._controller.command=self.stop
                    self._pub.publish(self._controller)
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    self._takeoff()
                    self._controller.command=self.takeoff
                    self._pub.publish(self._controller)
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    print not value

                if i == 5 and data.buttons[i] == 1:
                    value = int(rospy.get_param("posCtlPid/thrustBase"))
                    print value
                    self._update_params(["posCtlPid/thrustBase"])

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('leader', anonymous=True)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = leaderController(joy_topic)
    rospy.spin()
