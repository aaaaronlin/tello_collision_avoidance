#!/usr/bin/env python
import rospy
from tello import Tello
from obst_avoidance.msg import telemetry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Drone_Node:
	# Initialization for drone
	def __init__(self):
		rospy.init_node('Tello', anonymous=False)
		rospy.Subscriber('cmd_vel', Twist, self.__send_cmd)
		rospy.Subscriber('cmd_action', String, self.__action)
		self.pub_tel = rospy.Publisher('telemetry', telemetry, queue_size=1)

		if rospy.has_param('drone_ip'):
			self.drone_ip = rospy.get_param('drone_ip')
		else:
			self.drone_ip = "192.168.0.1"

		if rospy.has_param('drone_port'):
			self.drone_port = rospy.get_param('drone_port')
		else:
			self.drone_port = 8889
			
		self.drone = Tello()

	def __action(self, msg):
		if msg.data == "connect":
			self.drone.connect()
		elif msg.data == "disconnect":
			self.drone.disconnect()
		elif msg.data == "land":
			self.drone.land()
		elif msg.data == "takeoff":
			self.drone.takeoff()
		else:
			print("Invalid Action")

	# send command to drone
	def __send_cmd(self, msg):
		self.drone.cmd_vel([msg.linear.y, msg.linear.x, msg.angular.z, msg.linear.z])

	# publish telemetry
	def pub_telemetry(self):
		msg = telemetry()
		# match telemetry.msg with corrections from Tello output
		msg.stamp = rospy.Time.now()
		msg.acc = self.drone.acc
		msg.gyro = self.drone.gyro
		msg.q = self.drone.q
		msg.vel = self.drone.vel
		self.pub_tel.publish(msg)

if __name__ == '__main__':

    d = Drone_Node()

    d.pub_telemetry()

    rospy.spin()