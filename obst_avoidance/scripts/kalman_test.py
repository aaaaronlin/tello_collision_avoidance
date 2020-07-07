#!/usr/bin/env python
import os, sys
import pickle
import matplotlib.pyplot as plt

bagDir = os.path.split(sys.path[0])[0] + '/bag/'
filename = "kalman1.pickle"

pickleFile = open(bagDir+filename, 'rb')
data, fname = pickle.load(pickleFile)
pickleFile.close()

meas = data['/sensor_meas']
est = data['/est']

fig = plt.figure()

fig.suptitle(fname)
n = 2      # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i, sharex=fig.axes[0])

fig.axes[0].plot(meas['t'], meas['meas'][:, 0]*0.001, label='raw_dist')
fig.axes[0].plot(est['t'], est['dist'][:, 0], label='est_dist')
fig.axes[0].legend()

plt.show()