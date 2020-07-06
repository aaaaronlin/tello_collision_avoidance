#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from obst_avoidance.msg import estimate, sensor_meas, telemetry
from estimator.kalman import SimpleKalmanFilter, KalmanFilter

class Main_Node:
    def __init__(self):
        rospy.init_node('Main', anonymous=False)
        self.pub_act = rospy.Publisher('cmd_action', String, queue_size=0)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        self.pub_board = rospy.Publisher('cmd_board', String, queue_size=0)
        self.pub_est = rospy.Publisher('est', estimate, queue_size=0)

        rospy.Subscriber('telemetry', telemetry, self.predict)
        rospy.Subscriber('sensor_meas', sensor_meas, self.update)

        self.avoid_status = True
        self.flight_status = True

        self.SKF = SimpleKalmanFilter()
        self.KF = KalmanFilter()

    def send_drone_act(self, action):
        msg = String()
        msg.data = action
        self.pub_act.publish(msg)

    def send_board_cmd(self, action):
        msg = String()
        msg.data = action
        self.pub_board.publish(msg)

    def send_drone_cmd(self, data):
        msg = Twist
        msg.linear.x = data[0]
        msg.linear.y = data[1]
        msg.linear.z = data[2]
        msg.angular.z = data[3]
        self.pub_vel.publish(data)

    def predict(self, msg):
        # don't need feedback yet
        x, p = self.SKF.predict()

        est_msg = estimate()
        est_msg.dist[0] = x
        est_msg.covariance[0] = p
        self.pub_est.publish(est_msg)

    def update(self, msg):
        x, p, k = self.SKF.update_with_measurement(msg.meas[0])

        est_msg = estimate()
        est_msg.dist[0] = x
        est_msg.covariance[0] = p
        est_msg.k = k
        self.pub_est.publish(est_msg)

if __name__ == '__main__':

        main = Main_Node()

        rospy.sleep(2)

        main.send_drone_act("connect")

        # wait for drone connection timeout
        rospy.sleep(5)

        main.send_board_cmd("connect")

        main.send_board_cmd("start")

        #main.send_drone_act("takeoff")

        rospy.sleep(30.0)

        #main.send_drone_act("land")

        rospy.sleep(5.0)

        main.send_board_cmd("stop")

        main.send_board_cmd("disconnect")

        main.send_drone_act("disconnect")

