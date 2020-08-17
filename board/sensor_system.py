# Initiliazes four sensors and connects to a bluetooth address
# If there is a connection, polls and sends measurements from all sensors ~15 Hz

import bluetooth
import time
import VL53L0X
import RPi.GPIO as GPIO

server_address = # add your PC bluetooth address here
port = # set port number, should match launch file

# GPIO for each sensor shutdown
S1_SHDN = 14  # forward
S2_SHDN = 22  # left
S3_SHDN = 21  # right
S4_SHDN = 20  # up

GPIO.setwarning(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(S1_SHDN, GPIO.OUT)
GPIO.setup(S2_SHDN, GPIO.OUT)
GPIO.setup(S3_SHDN, GPIO.OUT)
GPIO.setup(S4_SHDN, GPIO.OUT)

GPIO.setup(S1_SHDN, GPIO.LOW)
GPIO.setup(S2_SHDN, GPIO.LOW)
GPIO.setup(S3_SHDN, GPIO.LOW)
GPIO.setup(S4_SHDN, GPIO.LOW)

GPIO.output(S1_SHDN, False)
GPIO.output(S2_SHDN, False)
GPIO.output(S3_SHDN, False)
GPIO.output(S4_SHDN, False)

# The library used cannot assign each device to an arbitrary i2c address, only the default 0x29
# Thus we initialize each one at a time and change address before initiliazing the next

GPIO.output(S1_SHDN, True)
time.sleep(0.5)
s1 = VL53L0X.VL53L0X(i2c_address=0x29)
s1.open()
s1.close()
s1.change_address(0x2A)
s1.open()

GPIO.output(S2_SHDN, True)
time.sleep(0.5)
s2 = VL53L0X.VL53L0X(i2c_address=0x29)
s2.open()
s2.close()
s2.change_address(0x2B)
s2.open()

GPIO.output(S1_SHDN, True)
time.sleep(0.5)
s3 = VL53L0X.VL53L0X(i2c_address=0x29)
s3.open()
s3.close()
s3.change_address(0x2C)
s3.open()

GPIO.output(S1_SHDN, True)
time.sleep(0.5)
s4 = VL53L0X.VL53L0X(i2c_address=0x29)
s4.open()
s4.close()
s4.change_address(0x2D)
s4.open()

connected = False
ranging = False
# message count sent to PC
i = 0

while True:
	if not connected:
		s = None
		s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		try:
			print("Connecting...")
			s.connect((server_address, port))
			print("Connected")
			i = 0
			connected = True
		except bluetooth.btcommon.BluetoothError as btErr:
			s.close()
			connected = False
			time.sleep(1)
			continue
	if connected:
		if not ranging:
			print("start ranging")
			time.sleep(0.5)
			s1.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)
			time.sleep(0.5)
			s2.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)
			time.sleep(0.5)
			s3.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)
			time.sleep(0.5)
			s4.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)
			ranging = True

		d1 = s1.get_distance()
		d2 = s2.get_distance()
		d3 = s3.get_distance()
		d4 = s4.get_distance()

		msg = str(i) + "," + str(d1) + "," + str(d2) + "," + str(d3) + "," + str(d4)
		print(msg)

		try:
			s.send(msg)
		except bluetooth.btcommon.BluetoothError as btErr:
			print(btErr)
			s.close()
			connected = False

			s1.stop_ranging()
			s2.stop_ranging()
			s3.stop_ranging()
			s4.stop_ranging()

			ranging = False

			continue
		i += 1
		time.sleep(0.033)

s.close()
s1.close()
s2.close()
s3.close()
s4.close()