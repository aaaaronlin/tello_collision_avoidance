#!/usr/bin/env python
from std_msgs.msg import String
import rospy

class MainLoop():
	def __init__(self):
		rospy.init_node('Main', anonymous=True)
		self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)

	def send_cmd(self, action):
		msg = String()
		msg.data = action
		self.pub_act.publish(msg)


if __name__ == '__main__':

    main = MainLoop()

    main.send_cmd("connect")

    main.send_cmd("takeoff")

    rospy.sleep(5.0)

    main.send_cmd("land")

    rospy.spin()