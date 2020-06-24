#!/usr/bin/env python
import tellopy
#from sensor.sensor_system import Sensor_System as ss
import sys
import time, datetime
import numpy as np
import pickle

file = "/home/aaron/Documents/Repos/wifi_drone_avoidance/src/obst_avoidance/scripts/tello_"+datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')+".pickle"

drone = tellopy.Tello()

# quadcopter states
acc = np.zeros(3)
gyro = np.zeros(3)
q = np.zeros(4)
vel = np.zeros(3)
t = 0.0  # time since epoch in sec
cmd_vel = np.zeros(4)

def update_state(event, sender, data, **args):
	drone = sender
	if event is drone.EVENT_LOG_DATA:
		t = time.time()
		d = [float(i) for i in data.format_cvs().split(',')]
		# corrections to meters and seconds
		acc = [9.81*d[6], -9.81*d[7], -9.81*d[8]]
		gyro = d[9:11]
		q = d[10:13]
		vel = d[14:16]

def fix_range(data, min=-1.0, max=1.0):
	for val in data:
		if val < min:
			val = min
		elif val > max:
			val = max
	return data
	
def cmd_vel(data):
	cmd_vel = fix_range(data)
	drone.set_roll(cmd_vel[0])
	drone.set_pitch(cmd_vel[1])
	drone.set_yaw(cmd_vel[2])
	drone.set_throttle(cmd_vel[3])

def setup():
	# connect to drone

	drone.subscribe(drone.EVENT_LOG_DATA, update_state)

	drone.connect()
	drone.wait_for_connection(60.0)

	# connect to sensors

def main():

	setup()

	try:
		while True:
			with open(file, "wb") as f:
				data = {'t':t, 'vel':vel, 'acc':acc, 'gyro':gyro, 'q':q, 'cmd_vel':cmd_vel}
				pickle.dump(data, f)
			# trajectory here
			
	except KeyboardInterrupt:
		f.close()
		drone.land()
	finally:

		drone.quit()

if __name__ == '__main__':

	main()