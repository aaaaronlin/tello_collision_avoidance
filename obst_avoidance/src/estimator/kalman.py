#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv

# http://www.kostasalexis.com/the-kalman-filter.html
# https://www.researchgate.net/publication/330540064_Kalman_Filter_Algorithm_Design_for_HC-SR04_Ultrasonic_Sensor_Data_Acquisition_System

# General Class
class SensorFilter(object):
    def __init__(self):
        self.rejectMeasVal = 2000
        self.rejectMeasCount = 0

    @staticmethod
    def reject_meas(z):
        if z >= 2000:
            return True

# 1D Kalman for a single TOF sensor
class SimpleKalmanFilter(SensorFilter, object):
    def __init__(self):
        super(SimpleKalmanFilter, self).__init__()
        self.dt = 0
        self.X = 0
        self.A = 1
        self.P = 0
        self.Q = 0.001
        self.R = 0.005

    # reset state estimate and covariance
    def reset(self):
        self.X = 0
        self.P = 0

    def predict(self):
        # next state is just current state
        self.X = self.A * self.X
        self.P = self.P + self.Q

        return self.X, self.P

    def update_with_measurement(self, z):
        # check for bad measurements i.e. too large in case of VL53L0x
        if self.reject_meas(z):
            self.rejectMeasCount += 1
            # a series of bad measurements indicates either:
            # changing surfaces or not near any walls
            # if more than 1 sec of bad meas:
            if self.rejectMeasCount > 100:
                self.reset()
                self.rejectMeasCount = 0
            return
        else:
            self.rejectMeasCount = 0

        innErr = z - self.X
        innCov = self.P + self.R
        K = (1/innCov)*self.P
        self.X = self.X + K*innErr
        self.P = (1 - K)*self.P

        return self.X, self.P, K

# 4D Kalman incorporating state estimate from IMU
class KalmanFilter:
    def __init__(self):
        # state
        # [dist_from_wall_x, dist_from_wall_y, vel_x, vel_y]
        self.dt = 0
        self.X = np.array([0.0, 0.0, 0.0, 0.0]).T
        # create state-transition matrix
        self.A = np.array([[1.0, 0.0, 0.1, 0.0],  # temp dt
                           [0.0, 1.0, 0.0, 0.1],  # temp dt
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])
        # process noise cov guess
        self.P = np.array([[0.1, 0.0, 0.0, 0.0],
                           [0.0, 0.1, 0.0, 0.0],
                           [0.0, 0.0, 0.1, 0.0],
                           [0.0, 0.0, 0.0, 0.1]])

        # process noise noise TODO: tune Q
        self.Q = np.array([[0.05, 0.0, 0.0, 0.0],
                           [0.0, 0.05, 0.0, 0.0],
                           [0.0, 0.0, 0.05, 0.0],
                           [0.0, 0.0, 0.0, 0.05]])

        # meas noise variance TODO: tune R
        self.R = np.array([[0.001, 0.0, 0.0, 0.0],
                           [0.0, 0.001, 0.0, 0.0],
                           [0.0, 0.0, 0.001, 0.0],
                           [0.0, 0.0, 0.0, 0.001]])

    def predict(self, dt, velx, vely):
        # update state
        self.dt = dt
        self.A[0, 2] = dt
        self.A[1, 3] = dt
        self.X[2] = velx
        self.X[3] = vely
        # a priori estimate of state and covariance
        # use process noise cov TODO: tune Q
        self.X = np.dot(self.A, self.X)
        self.Q = np.array([[(dt^3)/3, 0.0, (dt^2)/2, 0.0],
                           [0.0, (dt^3)/3, 0.0, (dt^2)/2],
                           [(dt^2)/2, 0.0, dt, 0.0],
                           [0.0, (dt^2)/2, 0.0, dt]]) * 0.05
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        return self.X, self.P

    def update_with_measurement(self, z):
        # calculate H
        H = np.zeros((4, 1))
        # innovation residual and covariance
        innErr = z - np.dot(H, self.X)
        innCov = np.dot(H, np.dot(self.P, H.T)) + self.R
        # kalman gain
        K = np.dot(self.P, np.dot(H.T, inv(innCov)))

        # a posteriori estimate of state and covariance
        self.X = self.X + np.dot(K, innErr)
        self.P = np.dot(np.eye(4) - np.dot(K, H), self.P)

        # ensure symmetry of covariance
        # self.P = (self.P + self.P.T) * 0.5

        return self.X, self.P, K





