#!/usr/bin/env python
import tellopy
import rospy
from obst_avoidance.msg import telemetry
from geometry_msgs.msg import Twist

class Tello:
	# Initialization for drone
	def __init__(self):
		rospy.init_node('Tello', anonymous=True)
		rospy.Subscriber('cmd_vel', Twist, self.__send_cmd)
		self.tm_pub = rospy.Publisher('telemetry', telemetry, queue_size=1)
		# define connection
		self.ip = rospy.get_param('drone_ip')
		self.port = rospy.get_param('drone_port')
		# connect to various ports
		self.tello = tellopy.Tello()
		#self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.pub_telemetry)
		self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.pub_telemetry)

	# connect to drone socket
	def connect(self):
		self.tello.connect()
		self.tello.wait_for_connection(60.0)

	# disconnect from socket
	def disconnect(self):
		self.tello.quit()

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
			with data.imu as d:
				msg.acc = [d.acc_x d.acc_y d.acc_z]
				msg.gyro = [d.gyro_x d.gyro_y d.gyro_z]
				msg.q = [d.q0 d.q1 d.q2 d.q3]
				msg.vel = [d.vg_x d.vg_y d.vg_z]
			self.tm_pub.publish(msg)

if __name__ == '__main__':

    drone = Tello()

    rospy.spin()