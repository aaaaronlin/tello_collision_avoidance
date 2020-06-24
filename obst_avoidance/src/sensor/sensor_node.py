#!/usr/bin/env python
import rospy
from obst_avoidance.msg import sensor_meas
import bluetooth

# B8:27:EB:7D:75:63 raspberry pi
# 94:E9:79:E6:E1:90 pc

class Sensor_Node():
    def __init__(self):
        rospy.init_node("Sensor_Node", anonymous=False)
        self.pub_meas = rospy.Publisher('sensor_meas', sensor_meas, queue_size=1)

if __name__ == '__main__':
    s = Sensor_Node()

    r = rospy.Rate(10)  # Hz

    while not rospy.is_shutdown():

        rospy.spin()