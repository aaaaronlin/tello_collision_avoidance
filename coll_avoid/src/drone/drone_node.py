#!/usr/bin/env python
import rospy
from tello import Tello
from coll_avoid.msg import telemetry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

# A general ROS Node for communicating with a WiFi drone
class Drone_Node:
	def __init__(self):
		# Setup drone topics (commands and telemetry)
		rospy.Subscriber('cmd_vel', Twist, self.__send_cmd)
		rospy.Subscriber('cmd_action', String, self.__action)
		self.pub_tel = rospy.Publisher('telemetry', telemetry, queue_size=1)

		# set drone ip and port from launch file
		self.drone_ip = rospy.get_param('drone_ip', "192.168.0.1")
		self.drone_port = rospy.get_param('drone_port', 8889)
		self.drone_timeout = rospy.get_param('drone_timeout', 5.0)

		# Initialize low-level drone driver located in tello.py
		# Tellopy library is used for simplicity
		self.d = Tello(self.drone_ip, self.drone_port, self.drone_timeout)

		self.connected = False
		self.in_flight = False

	# string commands for actions that may be provided by drone API
	# should correspond to a series of functions in drone driver file e.g. tello.py
	# callback for cmd_action topic (std_msgs/String)
	def __action(self, msg):
		if msg.data == "land":
			self.d.drone_land()
			self.in_flight = False

		elif msg.data == "emergency_stop":
			self.emergency_land()

		elif msg.data == "takeoff":
			self.d.drone_takeoff()
			self.in_flight = True

		elif msg.data == "connect":
			self.connected = self.d.drone_connect()
			# if connection fails, shutdown socket and node
			if not self.connected:
				self.d.drone_disconnect()
				rospy.signal_shutdown("No drone.")

		elif msg.data == "disconnect":
			# make sure drone lands before disconnecting
			# cannot reconnect until new flight
			if self.in_flight:
				self.d.drone_land()
				self.in_flight = False

			self.d.drone_disconnect()
			self.connected = False

		else:
			print("Invalid Action.")

	# send command to drone
	# callback from cmd_vel topic (geometry_msgs/Twist message)
	def __send_cmd(self, msg):
		self.d.cmd_vel([msg.linear.y, msg.linear.x, msg.angular.z, msg.linear.z])

	# publish telemetry.msg located in /msg
	def pub_telemetry(self):
		msg = telemetry()
		# corrections located in driver
		msg.header.stamp = rospy.Time.now()
		msg.acc = self.d.acc
		msg.gyro = self.d.gyro
		msg.q = self.d.q
		msg.vel = self.d.vel
		msg.batt = self.d.batt
		try:
			self.pub_tel.publish(msg)
		except Exception as e:
			print("Telemetry Error:" + str(e))
			return

	def emergency_land(self):
		self.d.drone_land()
		self.in_flight = False

		rospy.sleep(0.5)

		self.d.drone_disconnect()
		self.connected = False

if __name__ == '__main__':
	rospy.init_node('Drone', anonymous=False)
	dn = Drone_Node()

	# maximum rate to publish telemetry
	# Tello ~10-15 Hz

	r = rospy.Rate(50)  # Hz

	while not rospy.is_shutdown():
		# make sure drone is connected and new data has arrived before publishing anything
		if dn.connected and dn.d.new_data:
			# publish telemetry
			dn.pub_telemetry()
			# await next data update
			dn.d.toggle_new_data()

		r.sleep()

	# emergency land in case drone is still in flight
	# allows for ctrl+c land
	if dn.in_flight and dn.connected:
		dn.emergency_land()

