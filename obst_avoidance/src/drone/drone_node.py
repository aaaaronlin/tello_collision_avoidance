#!/usr/bin/env python
import rospy
from tello import Tello
from obst_avoidance.msg import telemetry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# A general ROS Node for communicating with a WiFi drone
class Drone_Node:
	def __init__(self):
		# Setup ROS node and drone topics (commands and telemetry)
		rospy.init_node('Drone', anonymous=False)
		rospy.Subscriber('cmd_vel', Twist, self.__send_cmd)
		rospy.Subscriber('cmd_action', String, self.__action)
		self.pub_tel = rospy.Publisher('telemetry', telemetry, queue_size=1)

		# set drone ip and port from launch file
		self.drone_ip = rospy.get_param('drone_ip', "192.168.0.1")
		self.drone_port = rospy.get_param('drone_port', 8889)

		# Initialize low-level drone driver located in tello.py
		# Tellopy library is used for simplicity
		self.d = Tello(self.drone_ip, self.drone_port)

	# string commands for actions that may be provided by drone API
	# should correspond to a series of functions in drone driver file e.g. tello.py
	# callback for cmd_action topic (std_msgs/String)
	def __action(self, msg):
		if msg.data == "connect":
			self.d.drone_connect()
		elif msg.data == "disconnect":
			self.d.drone_disconnect()
		elif msg.data == "land":
			self.d.drone_land()
		elif msg.data == "takeoff":
			self.d.drone_takeoff()
		else:
			print("Invalid Action")

	# send command to drone
	# callback from cmd_vel topic (geometry_msgs/Twist message)
	def __send_cmd(self, msg):
		self.d.cmd_vel([msg.linear.y, msg.linear.x, msg.angular.z, msg.linear.z])

	# publish telemetry.msg located in /msg
	def pub_telemetry(self):
		msg = telemetry()
		# corrections located in driver
		msg.stamp = rospy.Time.now()
		msg.acc = self.d.acc
		msg.gyro = self.d.gyro
		msg.q = self.d.q
		msg.vel = self.d.vel
		self.pub_tel.publish(msg)

if __name__ == '__main__':
	d = Drone_Node()

	# rate to publish telemetry

	r = rospy.Rate(10)  # Hz

	while not rospy.is_shutdown():

		d.pub_telemetry()

		rospy.spin()

