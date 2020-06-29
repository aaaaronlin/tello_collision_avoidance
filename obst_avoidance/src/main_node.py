#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from obst_avoidance.msg import estimate, sensor_meas, telemetry
from estimator.kalman import KalmanFilter

class MainLoop:
    def __init__(self):
        rospy.init_node('Main', anonymous=False)
        self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_rpi = rospy.Publisher('cmd_rpi', String, queue_size=1)
        self.pub_est = rospy.Publisher('est', estimate, queue_size=1)

        rospy.Subscriber('telemetry', telemetry, self.predict)
        rospy.Subscriber('sensor_meas', sensor_meas, self.update)

        self.avoid_status = True
        self.flight_status = False

        self.KF = KalmanFilter()

    def send_drone_act(self, action):
        msg = String()
        msg.data = action
        self.pub_act.publish(msg)

    def send_rpi_cmd(self, action):
        msg = String()
        msg.data = action
        self.pub_rpi.publish(msg)

    def send_drone_cmd(self, data):
        msg = Twist
        msg.linear.x = data[0]
        msg.linear.y = data[1]
        msg.linear.z = data[2]
        msg.angular.z = data[3]
        self.pub_vel.publish(data)

    def predict(self, msg):
        self.KF.predict(0.1, msg.vel[1], msg.vel[2])

    def update(self, msg):
        self.KF.update_with_measurement(0.1, msg.meas[0])


if __name__ == '__main__':

    main = MainLoop()

    rospy.sleep(2)

    main.send_drone_act("connect")

    rospy.sleep(5.0)

    main.send_drone_act("takeoff")

    while not rospy.is_shutdown():

        rospy.spin()

    main.send_drone_act("land")

    rospy.sleep(5.0)

    main.send_drone_act("disconnect")

