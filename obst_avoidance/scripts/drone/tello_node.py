#!/usr/bin/env python
import tellopy
import rospy
from obst_avoidance.msg import telemetry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Tello:
	# Initialization for drone
	def __init__(self):
		rospy.init_node('Tello', anonymous=True)
		rospy.Subscriber('cmd_vel', Twist, self.__send_cmd)
		rospy.Subscriber('cmd_action', String, self.__action)
		self.pub_tel = rospy.Publisher('telemetry', telemetry, queue_size=1)
		# define connection
		self.ip = rospy.get_param('drone_ip')
		self.port = rospy.get_param('drone_port')
		# connect to various ports
		self.tello = tellopy.Tello()
		#self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.pub_telemetry)
		self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.pub_telemetry)

	def __action(self, msg):
		if msg.data == "connect":
			self.__connect()
		elif msg.data == "disconnect":
			self.__disconnect()
		elif msg.data == "land":
			self.__land()
		elif msg.data == "takeoff":
			self.__takeoff()
		else:
			print("Invalid Action")

	# connect to drone socket
	def __connect(self):
		self.tello.connect()
		self.tello.wait_for_connection(60.0)

	# disconnect from socket
	def __disconnect(self):
		self.tello.quit()

	# land tello
	def __land(self):
		self.tello.land()

	def __takeoff(self):
		self.tello.takeoff()
		
	# send command to drone
	def __send_cmd(self, msg):
		tello.set_roll(msg.linear.y)
		tello.set_pitch(msg.linear.x)
		tello.set_yaw(msg.angular.z)
		tello.set_throttle(msg.linear.z)
		
	# publish telemetry
	def pub_telemetry(self, event, sender, data, **args):
		if event is self.tello.EVENT_LOG_DATA:
			msg = telemetry()
			# matches telemetry.msg with corrections from Tello output
			with data.imu as d:
				msg.stamp = rospy.Time.now()
				msg.acc = 9.81*[d.acc_x -d.acc_y -d.acc_z]
				msg.gyro = [d.gyro_x d.gyro_y d.gyro_z]
				msg.q = [d.q0 d.q1 d.q2 d.q3]
				msg.vel = [d.vg_x d.vg_y -d.vg_z]
			self.pub_tel.publish(msg)

if __name__ == '__main__':

    drone = Tello()

    rospy.spin()