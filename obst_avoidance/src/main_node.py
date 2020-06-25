#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MainLoop:
    def __init__(self):
        rospy.init_node('Main', anonymous=False)
        self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_rpi = rospy.Publisher('cmd_rpi', String, queue_size=1)

        self.avoid_status = True
        self.flight_status = False

    def send_drone_act(self, action):
        msg = String()
        msg.data = action
        self.pub_act.publish(msg)

    def send_rpi_cmd(self, action):
        msg = String()
        msg.data = action
        self.pub_rpi.publish(msg)

    def send_drone_cmd(self, data):
        msg = Twist
        msg.linear.x = data[0]
        msg.linear.y = data[1]
        msg.linear.z = data[2]
        msg.angular.z = data[3]
        self.pub_vel.publish(data)

if __name__ == '__main__':

    while not rospy.is_shutdown():
        main = MainLoop()

        rospy.sleep(2)

        main.send_drone_act("connect")

        rospy.sleep(5.0)

        main.send_drone_act("takeoff")

        rospy.sleep(15.0)

        main.send_drone_act("land")

        rospy.sleep(5.0)

        main.send_drone_act("disconnect")

        rospy.spin()
