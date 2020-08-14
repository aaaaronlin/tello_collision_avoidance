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
kf = KalmanFilter(X=np.concatenate((dist, np.array([0.0, 0.0, 0.0]))).T)
t_old = None
x_data = []
p_data = []
k_data = []
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
        #x, p = kf.predict(dt, vel)
        dt_data.append(dt)
        vel_data.append(vel)

    if topic == '/sensor_meas':
        kf.predict_no_drone()
        dist_raw = np.array([msg.meas[0] * 0.001, msg.meas[1] * 0.001, msg.meas[1] * 0.001,
                             msg.meas[2] * 0.001]).T  # raw data comes in mm

        x, p, k = kf.update_with_measurement(dist_raw)

        dist_data.append(dist_raw)
        x_data.append(x)
        p_data.append(np.ndarray.flatten(p).tolist())
        k_data.append(np.ndarray.flatten(k).tolist())
        t_data.append(t)

x_data = np.array(x_data)
p_data = np.array(p_data)
k_data = np.array(k_data)
t_data = np.array(t_data)
dist_data = np.array(dist_data)

fig = plt.figure()

fig.suptitle(filename)
n = 6      # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i)

fig.axes[0].plot(t_data, x_data[:, 0], label='est_dist_x')
fig.axes[0].plot(t_data, dist_data[:, 0], label='raw_dist_x')
fig.axes[0].legend()
fig.axes[0].set_ylim([0, max(x_data[:, 0])])

fig.axes[1].plot(t_data, x_data[:, 1], label='est_dist_y')
fig.axes[1].plot(t_data, dist_data[:, 1], label='raw_dist_y')
fig.axes[1].set_ylim([0, max(x_data[:, 1])])
fig.axes[1].legend()

fig.axes[2].plot(t_data, x_data[:, 3], label='est_dist_z')
fig.axes[2].plot(t_data, dist_data[:, 3], label='raw_dist_z')
fig.axes[2].set_ylim([0, max(x_data[:, 3])])
fig.axes[2].legend()

fig.axes[3].plot(t_data, k_data, label='test_cov')
fig.axes[3].legend()

fig.axes[4].plot(t_data, p_data, label='test_k')
fig.axes[4].legend()

fig2 = plt.figure()
fig2.add_subplot(2, 1, 1)
fig2.axes[0].plot(vel_data)
fig2.add_subplot(2, 1, 2)
fig2.axes[1].plot(dt_data)

plt.show()