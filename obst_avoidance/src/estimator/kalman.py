#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv

# http://www.kostasalexis.com/the-kalman-filter.html
# https://www.researchgate.net/publication/330540064_Kalman_Filter_Algorithm_Design_for_HC-SR04_Ultrasonic_Sensor_Data_Acquisition_System

# General Class
class SensorFilter(object):
    def __init__(self):
        self.rejectMeasCountMax = 10  # ~1 sec of bad measurements
        self.dt = 0.0

    @staticmethod
    def reject_meas(z):
        if z > 2.000 or z < .020:  # 2000 mm and 20 mm bounds
            return True

# n-d Kalman for TOF sensors
class SimpleKalmanFilter(SensorFilter, object):
    def __init__(self, X=2.0):
        super(SimpleKalmanFilter, self).__init__()
        self.X = X
        self.X0 = X
        self.A = 1.0
        self.P = 0.0
        self.Q = 0.0002
        self.R = 0.002

        self.rejectMeasCount = 0

    # reset state estimate and covariance to initial condition
    def reset(self):
        self.X = self.X0
        self.P = 0.0

    def predict(self):
        # next state is just current state
        self.X = self.A * self.X
        self.P = self.P + self.Q

        return self.X, self.P

    def update_with_measurement(self, z):
        # check for bad measurements i.e. 8190 return value in case of VL53L0x
        if self.reject_meas(z):
            self.rejectMeasCount += 1
            # a series of bad measurements usually indicates either:
            # changing surfaces or not near any walls
            # if more than 1 sec of bad meas:
            if self.rejectMeasCount > self.rejectMeasCountMax:
                self.reset()
                self.rejectMeasCount = 0
            return self.X, self.P, 0
        else:
            self.rejectMeasCount = 0

        innErr = z - self.X
        innCov = self.P + self.R
        K = (1/innCov)*self.P
        self.X = self.X + K*innErr
        self.P = (1 - K)*self.P

        return self.X, self.P, K

# 4D Kalman incorporating state estimate from IMU
class KalmanFilter(SensorFilter, object):
    def __init__(self, X=np.array([2.0, 2.0, 2.0, 2.0]).T):
        super(KalmanFilter, self).__init__()
        # state
        # [dist_from_wall_x, dist_from_wall_y, dist_from_wall_-x, dist_from_wall_-y]
        self.X = X
        self.X0 = X
        # create state-transition matrix
        self.A = np.array([[1.0, 0.0, 0.1, 0.0],  # temp dt
                           [0.0, 1.0, 0.0, 0.1],  # temp dt
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])
        # process noise cov guess
        self.P = np.zeros((4, 4))

        # process noise noise TODO: tune Q
        self.Q = np.array([[0.0002, 0.0, 0.0, 0.0],
                           [0.0, 0.0002, 0.0, 0.0],
                           [0.0, 0.0, 0.0002, 0.0],
                           [0.0, 0.0, 0.0, 0.0002]])

        # meas noise variance TODO: tune R
        self.R = np.array([[0.001, 0.0, 0.0, 0.0],
                           [0.0, 0.001, 0.0, 0.0],
                           [0.0, 0.0, 0.001, 0.0],
                           [0.0, 0.0, 0.0, 0.001]])

    # reset state estimate and covariance
    def reset(self):
        self.X = self.X0
        self.P = np.zeros((4, 4))

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
        H = np.ones((4, 1))
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





