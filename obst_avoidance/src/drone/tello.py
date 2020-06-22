#!/usr/bin/env python
import tellopy
import numpy as np
import time

class Tello():

	def __init__(self):
		self.tello = tellopy.Tello()
		self.acc = np.zeros(3)
		self.gyro = np.zeros(3)
		self.q = np.zeros(4)
		self.vel = np.zeros(3)
		self.t = 0.0 # time since epoch in sec

	# connect to drone socket
	def connect(self):
		self.tello.connect()
		self.tello.wait_for_connection(60.0)
		print("Waiting...")
		#self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.__update_state)
		self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.__update_state)
		
	# disconnect from socket
	def disconnect(self):
		self.tello.quit()

	# land prompt
	def land(self):
		self.tello.land()

	# takeoff prompt
	def takeoff(self):
		self.tello.takeoff()

	# send command
	def cmd_vel(self, cmd):
		self.tello.set_roll(cmd[0])
		self.tello.set_pitch(cmd[1])
		self.tello.set_yaw(cmd[2])
		self.tello.set_throttle(cmd[3])

	# update_state
	def __update_state(self, event, sender, data, **args):
		if event is self.tello.EVENT_LOG_DATA:
			with data.imu as d:
				self.t = time.time()
				self.acc = 9.81*[d.acc_x, -d.acc_y, -d.acc_z]
				self.gyro = [d.gyro_x, d.gyro_y, d.gyro_z]
				self.q = [d.q0, d.q1, d.q2, d.q3]
				self.vel = [d.vg_x, d.vg_y, -d.vg_z]