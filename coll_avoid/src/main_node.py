#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from coll_avoid.msg import estimate, sensor_meas, telemetry
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

    # actions are string commands
    def send_drone_act(self, action):
        msg = String()
        msg.data = action
        self.pub_act.publish(msg)

    def send_sensor_act(self, action):
        msg = String()
        msg.data = action
        self.pub_sensor.publish(msg)

    # command desired linear and angular velocities
    def send_drone_cmd(self, data):
        linear = Vector3(data[0], data[1], data[2])
        angular = Vector3(0.0, 0.0, data[3])
        self.pub_vel.publish(linear, angular)

# don't need state yet TODO
def predict(msg):
    global main, kf, dist, t

    dt = rospy.Time.now().secs + (rospy.Time.now().nsecs*(10**-9)) - t
    vel = [msg.vel[0], msg.vel[1], msg.vel[2]]
    x, p = kf.predict(dt, vel)

    dist = x[0:4].tolist()

    est_msg = estimate()
    est_msg.dist = dist
    est_msg.vel = x[4:7].tolist()
    est_msg.covariance = np.ndarray.flatten(p).tolist()
    main.pub_est.publish(est_msg)

    t = rospy.Time.now().secs + (rospy.Time.now().nsecs*(10**-9))

def update(msg):
    global main, kf, dist, use_drone_state
    # TODO fix 4th axis (copies right meas to left)
    dist_raw = np.array([msg.meas[0]*0.001, msg.meas[1]*0.001, msg.meas[2]*0.001, msg.meas[3]*0.001]).T  # raw data comes in mm

    if not use_drone_state:
        kf.predict_no_drone()

    x, p, k = kf.update_with_measurement(dist_raw)

    dist = x[0:4].tolist()

    est_msg = estimate()
    est_msg.dist = dist
    est_msg.vel = x[4:7].tolist()
    est_msg.covariance = np.ndarray.flatten(p).tolist()
    est_msg.k = np.ndarray.flatten(k).tolist()
    main.pub_est.publish(est_msg)


# connect and start communication with drone and board
def start():
    global main
    main = Main_Node()

    rospy.sleep(2)

    main.send_drone_act("connect")
    # wait for drone connection timeout or connection
    rospy.sleep(7)
    main.send_sensor_act("connect")
    rospy.sleep(1)
    main.send_sensor_act("start")
    # wait for sensor connection timeout or connection
    rospy.sleep(7)


def loop():
    global main, kf, dist, t, use_drone_state
    # control parameters in meters
    dist_ref = np.array([0.8, 0.8, 0.8, 0.8])  # desired position relative to any obstacles
    dist = np.array([2.0, 2.0, 2.0, 2.0])  # initial condition
    free_dist = np.array([1.3, 1.3, 1.3, 1.0])  # free area
    danger_dist = np.array([0.2, 0.2, 0.2, 0.2])
    # init estimators
    kf = KalmanFilter(X=np.concatenate((dist, np.array([0.0, 0.0, 0.0]))).T)
    # init controllers TODO add other axis
    ctrl_1 = PID(kp=0.8, ki=0.0, kd=0.1)
    ctrl_2 = PID(kp=0.8, ki=0.0, kd=0.1)
    ctrl_3 = PID(kp=1.1, ki=0.0, kd=0.1)

    ctrl_1.reset()
    ctrl_2.reset()
    ctrl_3.reset()

    rate = rospy.Rate(10)  # Hz

    main.send_drone_act("takeoff")

    rospy.sleep(1.5)

    # start kalman if data available
    t = rospy.Time.now().secs + (rospy.Time.now().nsecs*(10**-9))
    use_drone_state = rospy.get_param('use_drone_state', False)
    if use_drone_state:
        rospy.Subscriber('telemetry', telemetry, predict)

    rospy.Subscriber('sensor_meas', sensor_meas, update)

    while not rospy.is_shutdown():

        # x
        if dist[0] > free_dist[0]:  # safe condition, free flight
            k_1 = 0.0  # just hover
            ctrl_1.reset()
        elif dist[0] > danger_dist[0]:
            k_1 = -ctrl_1.run(dist[0], dist_ref[0])
        else:
            main.send_drone_act('emergency_stop')
            break

        # y
        if dist[1] > free_dist[1] and dist[2] > free_dist[2]:  # safe condition, on both sides
            k_2 = 0.0  # just hover
            ctrl_2.reset()
        elif dist[1] > danger_dist[1] and dist[2] > free_dist[2]:  # close only on pos y side
            k_2 = ctrl_2.run(dist[1], dist_ref[1])
        elif dist[1] > free_dist[1] and dist[2] > danger_dist[2]:  # close only on neg y side
            k_2 = -ctrl_2.run(dist[2], dist_ref[2])
        elif dist[1] > danger_dist[1] and dist[2] > danger_dist[2]:  # close on both sides
            k_2 = ctrl_2.run(dist[1], dist[2])  # navigate to midpoint
        else:  # danger on either or both sides
            main.send_drone_act('emergency_stop')
            break

        # z
        if dist[3] > free_dist[3]:
            k_3 = 0.0
            ctrl_3.reset()
        elif dist[3] > danger_dist[3]:
            k_3 = -ctrl_3.run(dist[3], dist_ref[3])
        else:
            main.send_drone_act('emergency_stop')
            break

        main.send_drone_cmd([k_1, k_2, k_3, 0.0])  # linear x, y, z and angular z

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Main', anonymous=False)
    start()
    loop()

