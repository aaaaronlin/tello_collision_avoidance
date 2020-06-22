#!/usr/bin/python

# MIT License
# 
# Copyright (c) 2017 John Bryan Moore
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X
import RPi.GPIO as GPIO

class Sensor_System():

    def __init__(self):
        self.s1 = None

        self.s2 = None
        # GPIO for Sensor 1 shutdown pin
        self.s1_SHDN = 20
        # GPIO for Sensor 2 shutdown pin
        self.s2_SHDN = 16

        self.rate = 0.0

        GPIO.setwarnings(False)

        GPIO.setmode(GPIO.BCM)

    def reset(self):
        # Setup GPIO for shutdown pins on each VL53L0X

        GPIO.setup(s1_SHDN, GPIO.OUT)
        GPIO.setup(s2_SHDN, GPIO.OUT)

        # Set all shutdown pins low to turn off each VL53L0X
        GPIO.output(s1_SHDN, GPIO.LOW)
        GPIO.output(s2_SHDN, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

    def start(self):

        # Create one object per VL53L0X passing the address to give to
        # each.
        self.s1 = VL53L0X.VL53L0X(address=0x2B)
        self.s2 = VL53L0X.VL53L0X(address=0x2D)

        # Set shutdown pin high for the first VL53L0X then 
        # call to start ranging 
        GPIO.output(sensor1_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.s1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

        # Set shutdown pin high for the second VL53L0X then 
        # call to start ranging 
        GPIO.output(sensor2_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.s2.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

        self.rate = s1.get_timing()/1000000.00

    def poll_meas(self):
        return [self.s1.get_distance(), self.s2.get_distance()] # in mm

    def end(self):
        self.s1.stop_ranging()
        GPIO.output(sensor2_shutdown, GPIO.LOW)
        self.s2.stop_ranging()
        GPIO.output(sensor1_shutdown, GPIO.LOW)