#!/usr/bin/env python
import os, sys
import rosbag
import matplotlib.pyplot as plt
from kalman import SimpleKalmanFilter, KalmanFilter
import numpy as np

mainDir = os.path.split(os.path.split(sys.path[0])[0])[0]
bagDir = mainDir + '/bag/'
filename = sys.argv[1]

bag = rosbag.Bag(bagDir+filename)
tMin = np.inf

dist = np.array([2.0, 2.0, 2.0, 2.0])  # initial condition
kf_no_imu = KalmanFilter(X=np.concatenate((dist, np.array([0.0, 0.0, 0.0]))).T)
kf = KalmanFilter(X=np.concatenate((dist, np.array([0.0, 0.0, 0.0]))).T)
t_old = None
x_data = []
xi_data = []
p_data = []
pi_data = []
k_data = []
ki_data = []
t_data = []
vel_data = []
dist_data = []
vel_data = []
dt_data = []

for topic, msg, t_nano in bag.read_messages(topics=['/telemetry', '/sensor_meas']):
    # get current time in secs
    t = t_nano.to_sec()
    tMin = min(tMin, t)
    t -= tMin
    if topic == '/telemetry':
        # calculate dt
        if t_old is None:
            t_old = t
        dt = t - t_old
        t_old = t

        # predict
        vel = [msg.vel[0], msg.vel[1], msg.vel[2]]
        x, p = kf.predict(dt, vel)
        dt_data.append(dt)
        vel_data.append(vel)

    if topic == '/sensor_meas':
        kf_no_imu.predict_no_drone()
        dist_raw = np.array([msg.meas[0] * 0.001, msg.meas[1] * 0.001, msg.meas[2] * 0.001,
                             msg.meas[3] * 0.001]).T  # raw data comes in mm

        x, p, k = kf_no_imu.update_with_measurement(dist_raw)
        xi, pi, ki = kf.update_with_measurement(dist_raw)

        dist_data.append(dist_raw)
        x_data.append(x)
        xi_data.append(xi)
        p_data.append(np.ndarray.flatten(p).tolist())
        pi_data.append(np.ndarray.flatten(pi).tolist())
        k_data.append(np.ndarray.flatten(k).tolist())
        ki_data.append(np.ndarray.flatten(ki).tolist())
        t_data.append(t)

x_data = np.array(x_data)
xi_data = np.array(xi_data)
p_data = np.array(p_data)
pi_data = np.array(pi_data)
k_data = np.array(k_data)
ki_data = np.array(ki_data)
t_data = np.array(t_data)
dist_data = np.array(dist_data)

fig = plt.figure()

fig.suptitle(filename)
n = 4      # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i)

# x
fig.axes[0].plot(t_data, x_data[:, 0], label='est_dist_x_no_imu')
fig.axes[0].plot(t_data, xi_data[:, 0], label='est_dist_x_with_imu')
fig.axes[0].plot(t_data, dist_data[:, 0], label='raw_dist_x', alpha=0.3)
fig.axes[0].legend()
fig.axes[0].set_ylim([0, max(max(x_data[:, 0]), max(xi_data[:, 0]))])

# pos y
fig.axes[1].plot(t_data, x_data[:, 1], label='est_dist_y_no_imu')
fig.axes[1].plot(t_data, xi_data[:, 1], label='est_dist_y_with_imu')
fig.axes[1].plot(t_data, dist_data[:, 1], label='raw_dist_y', alpha=0.3)
fig.axes[1].set_ylim([0, max(max(x_data[:, 1]), max(xi_data[:, 1]))])
fig.axes[1].legend()

# neg y
fig.axes[2].plot(t_data, x_data[:, 2], label='est_dist_y_no_imu')
fig.axes[2].plot(t_data, xi_data[:, 2], label='est_dist_y_with_imu')
fig.axes[2].plot(t_data, dist_data[:, 2], label='raw_dist_y', alpha=0.3)
fig.axes[2].set_ylim([0, max(max(x_data[:, 2]), max(xi_data[:, 2]))])
fig.axes[2].legend()

# z
fig.axes[3].plot(t_data, x_data[:, 3], label='est_dist_z_no_imu')
fig.axes[3].plot(t_data, xi_data[:, 3], label='est_dist_z_with_imu')
fig.axes[3].plot(t_data, dist_data[:, 3], label='raw_dist_z', alpha=0.3)
fig.axes[3].set_ylim([0, max(max(x_data[:, 3]), max(xi_data[:, 3]))])
fig.axes[3].legend()

fig2 = plt.figure()
fig2.add_subplot(2, 1, 1)
fig2.axes[0].plot(vel_data)
fig2.add_subplot(2, 1, 2)
fig2.axes[1].plot(dt_data)

plt.show()