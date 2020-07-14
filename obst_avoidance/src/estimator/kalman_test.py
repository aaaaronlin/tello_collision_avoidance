#!/usr/bin/env python
import os, sys
import pickle
import matplotlib.pyplot as plt
from kalman import SimpleKalmanFilter
import numpy as np

mainDir = os.path.split(os.path.split(sys.path[0])[0])[0]
bagDir = mainDir + '/bag/'
filename = "1.4.pickle"

pickleFile = open(bagDir+filename, 'rb')
data, fname = pickle.load(pickleFile)
pickleFile.close()

meas = data['/sensor_meas']
est = data['/est']

fig = plt.figure()

fig.suptitle(fname)
n = 5      # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i, sharex=fig.axes[0])

fig.axes[0].plot(est['t'], est['dist'][:, 0], label='est_dist')
fig.axes[0].plot(meas['t'], meas['meas'][:, 0]*0.001, label='raw_dist')
fig.axes[0].legend()
fig.axes[0].set_ylim([0, 2])

SKF = SimpleKalmanFilter()

x_arr = np.zeros(np.size(meas['meas'], 0))
p_arr = np.zeros(np.size(meas['meas'], 0))
k_arr = np.zeros(np.size(meas['meas'], 0))
for i, m in np.ndenumerate(meas['meas'][:, 0]*0.001):
    SKF.predict()
    x, p, k = SKF.update_with_measurement(m)
    x_arr[i] = x
    p_arr[i] = p
    k_arr[i] = k

fig.axes[1].plot(meas['t'], x_arr, label='test_dist')
fig.axes[1].plot(meas['t'], meas['meas'][:, 0]*0.001, label='raw_dist')
fig.axes[1].set_ylim([0, 2])
fig.axes[1].legend()

fig.axes[2].plot(meas['t'], x_arr, label='test_dist')
fig.axes[2].set_ylim([0, 2])
fig.axes[2].legend()

fig.axes[3].plot(meas['t'], p_arr, label='test_cov')
fig.axes[3].legend()

fig.axes[4].plot(meas['t'], k_arr, label='test_k')
fig.axes[4].legend()

plt.show()