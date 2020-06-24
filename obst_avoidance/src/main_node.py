#!/usr/bin/env python
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy

class MainLoop():
	def __init__(self):
		rospy.init_node('Main', anonymous=False)
		self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def send_act(self, action):
		msg = String()
		msg.data = action
		self.pub_act.publish(msg)

	def send_cmd(self, data):
		msg = Twist
		msg.linear.x = data[0]
		msg.linear.y = data[1]
		msg.linear.z = data[2]
		msg.angular.z = data[3]
		self.pub_vel.publish(data)


if __name__ == '__main__':
	main = MainLoop()

	rospy.sleep(2)

	main.send_act("connect")

	rospy.sleep(5.0)

	main.send_act("takeoff")

	rospy.sleep(10.0)

	main.send_act("land")

	rospy.sleep(10.0)

	main.send_act("disconnect")

	rospy.spin()

