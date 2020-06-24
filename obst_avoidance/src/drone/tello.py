#!/usr/bin/env python
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger
import numpy as np
import time

# Inherits tellopy class and exposes what is needed for state feedback

class Tello(tello.Tello):

	def __init__(self, ip, port):
		# not necessary with Tellopy
		self.ip = ip
		self.port = port

		self.tello = tello.Tello()
		# state variables of interest
		self.acc = np.zeros(3)
		self.gyro = np.zeros(3)
		self.q = np.zeros(4)
		self.vel = np.zeros(3)
		self.recv_t = 0.0

	# update_state
	def update_state(self, event, sender, data, **args):
		drone = sender
		if event is drone.EVENT_LOG_DATA:
			d = [float(i) for i in data.format_cvs().split(',')]
			self.recv_t = time.time()  # time since epoch in sec

			# corrections to meters and seconds
			self.acc = [9.81 * d[6], -9.81 * d[7], -9.81 * d[8]]
			self.gyro = d[9:11]
			self.q = d[10:13]
			self.vel = d[14:16]

	# connect to drone socket
	def connect(self):
		#self.subscribe(self.EVENT_FLIGHT_DATA, self.update_state)
		self.subscribe(self.EVENT_LOG_DATA, self.update_state)

		self.connect()
		try:
			self.wait_for_connection(60.0)
		except Exception as e:
			print (e)
			self.quit()


	# disconnect from socket
	def disconnect(self):
		self.quit()

	# land prompt
	def land(self):
		self.land()

	# takeoff prompt
	def takeoff(self):
		self.takeoff()

	# send command
	def cmd_vel(self, cmd):
		self.set_roll(cmd[0])
		self.set_pitch(cmd[1])
		self.set_yaw(cmd[2])
		self.set_throttle(cmd[3])

