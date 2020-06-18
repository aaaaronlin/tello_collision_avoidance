#!/usr/bin/env python
from std_msgs.msg import String
import rospy

class MainLoop():
	def __init__(self):
		self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)

	def beginFlight(self):
		self.__publishAction("connect")
		self.__publishAction("takeoff")

	def __publishAction(self, action)
		msg = String()
		msg.data = action
		self.pub_act.publish(msg)


if __name__ == '__main__':

    main = MainLoop()

    mainLoop.beginFlight()

    rospy.spin()