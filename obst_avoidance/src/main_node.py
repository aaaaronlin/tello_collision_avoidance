#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from obst_avoidance.msg import estimate, sensor_meas, telemetry
from estimator.kalman import SimpleKalmanFilter, KalmanFilter
from controller.PID import PID
import drone.drone_node

class Main_Node:
    def __init__(self):
        self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_sensor = rospy.Publisher('cmd_sensor', String, queue_size=1)
        self.pub_est = rospy.Publisher('est', estimate, queue_size=1)

        rospy.Subscriber('telemetry', telemetry, predict)
        rospy.Subscriber('sensor_meas', sensor_meas, update)

        self.avoid_status = False
        self.flight_status = False

    def send_drone_act(self, action):
        msg = String()
        msg.data = action
        self.pub_act.publish(msg)

    def send_sensor_cmd(self, action):
        msg = String()
        msg.data = action
        self.pub_sensor.publish(msg)

    def send_drone_cmd(self, data):
        msg = Twist
        msg.linear.x = data[0]
        msg.linear.y = data[1]
        msg.linear.z = data[2]
        msg.angular.z = data[3]
        self.pub_vel.publish(data)


# don't need state yet TODO
def predict(msg):
    global main, skf, kf
    return
    #
    # x, p = self.SKF.predict()
    #
    # est_msg = estimate()
    # est_msg.dist[0] = x
    # est_msg.covariance[0] = p
    # self.pub_est.publish(est_msg)


def update(msg):
    global main, skf, kf, dist
    # remove after prediction added TODO
    skf.predict()
    # only take first value (1D) for now TODO
    dist[0] = msg.meas[0]*0.001

    x, p, k = skf.update_with_measurement(dist[0])

    est_msg = estimate()
    est_msg.dist[0] = x
    est_msg.covariance[0] = p
    est_msg.k = k
    main.pub_est.publish(est_msg)


# connect and start communication with drone and board
def initialize():
    main = Main_Node()

    rospy.sleep(2)

    main.send_drone_act("connect")
    # wait for drone connection timeout or connection
    rospy.sleep(7)
    main.send_sensor_cmd("connect")
    rospy.sleep(1)
    main.send_sensor_cmd("start")


def loop():
    global main, skf, kf, dist
    # control parameters
    dist_ref = [0.8, 0.8, 0.8, 0.8]
    dist = [2.0, 2.0, 2.0, 2.0]

    # init estimators and controllers TODO add other axis
    skf = SimpleKalmanFilter()
    kf = KalmanFilter()
    ctrl_1 = PID(kp=0.05, ki=0.02, kd=0.001)

    skf.reset()
    ctrl_1.reset()

    rate = rospy.Rate(10)  # Hz

    while not rospy.is_shutdown():

        k_1 = -ctrl_1.run(dist[0], dist_ref[0])
        #main.send_drone_cmd([k_1, 0, 0, 0])

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Main', anonymous=False)
    initialize()
    loop()

    # main.send_drone_act("takeoff")

    # rospy.sleep(30.0)

    # main.send_drone_act("land")

    # rospy.sleep(5.0)

    # main.send_sensor_cmd("stop")

    # main.send_sensor_cmd("disconnect")

    # main.send_drone_act("disconnect")
