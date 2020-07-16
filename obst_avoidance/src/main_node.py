#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from obst_avoidance.msg import estimate, sensor_meas, telemetry
from estimator.kalman import SimpleKalmanFilter, KalmanFilter
from controller.PID import PID
import numpy as np
import drone.drone_node

class Main_Node:
    def __init__(self):
        self.pub_act = rospy.Publisher('cmd_action', String, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_sensor = rospy.Publisher('cmd_sensor', String, queue_size=1)
        self.pub_est = rospy.Publisher('est', estimate, queue_size=1)

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
        linear = Vector3(data[0], data[1], data[2])
        angular = Vector3(0.0, 0.0, data[3])
        self.pub_vel.publish(linear, angular)

# don't need state yet TODO
def predict(msg):
    global main, skf1, kf, dist, t
    return

    # dt = rospy.Time.now().secs() - t
    # x, p = kf.predict(dt, msg.vel[0], msg.vel[1])

    # est_msg = estimate()
    # est_msg.dist = x
    # est_msg.covariance = np.ndarray.flatten(p)
    # main.pub_est.publish(est_msg)

    # t = rospy.Time.now().secs()


def update(msg):
    global main, skf1, kf, dist
    # only take first value (1D) for now TODO
    dist[0] = msg.meas[0]*0.001  # raw data comes in mm

    # remove after prediction added TODO
    skf1.predict()
    x, p, k = skf1.update_with_measurement(dist[0])

    est_msg = estimate()
    est_msg.dist[0] = x
    est_msg.covariance[0] = p
    est_msg.k = k
    main.pub_est.publish(est_msg)


# connect and start communication with drone and board
def initialize():
    global main
    main = Main_Node()

    rospy.sleep(2)

    main.send_drone_act("connect")
    # wait for drone connection timeout or connection
    rospy.sleep(7)
    main.send_sensor_cmd("connect")
    rospy.sleep(1)
    main.send_sensor_cmd("start")
    # wait for sensor connection timeout or connection
    rospy.sleep(7)


def loop():
    global main, skf1, kf, dist, t
    # control parameters in meters
    dist_ref = np.array([0.8, 0.8, 0.8, 0.8])  # desired position relative to any obstacles
    dist = np.array([2.0, 2.0, 2.0, 2.0])  # initial condition
    free_dist = np.array([1.5, 1.5, 1.5, 1.5])  # free area
    danger_dist = np.array([0.3, 0.3, 0.3, 0.3])
    # init estimators TODO add other axis
    skf1 = SimpleKalmanFilter(dist[0])
    kf = KalmanFilter(dist)
    # init controllers TODO add other axis
    ctrl_1 = PID(kp=1.6, ki=0.0, kd=0.0)
    ctrl_2 = PID(kp=1.6, ki=0.0, kd=0.0)

    skf1.reset()
    kf.reset()
    ctrl_1.reset()
    ctrl_2.reset()

    rate = rospy.Rate(10)  # Hz

    main.send_drone_act("takeoff")

    # start kalman if data available
    rospy.Subscriber('telemetry', telemetry, predict)
    rospy.Subscriber('sensor_meas', sensor_meas, update)
    t = rospy.Time.now().secs()

    while not rospy.is_shutdown():
        if dist[0] > free_dist[0]:  # further than 1.0 m away from a target distance
            k_1 = 0.0  # just hover
        elif dist[0] > danger_dist:
            k_1 = -ctrl_1.run(dist[0], dist_ref[0])
        else:
            k_1 = -1.0  # max reverse if in danger


        main.send_drone_cmd([k_1, 0, 0, 0])

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Main', anonymous=False)
    initialize()
    loop()

