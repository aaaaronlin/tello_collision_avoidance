#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv

# http://www.kostasalexis.com/the-kalman-filter.html
# https://www.researchgate.net/publication/330540064_Kalman_Filter_Algorithm_Design_for_HC-SR04_Ultrasonic_Sensor_Data_Acquisition_System

# General Class
class SensorFilter(object):
    def __init__(self):
        self.rejectMeasCountMax = 5  # ~1 sec of bad measurements
        self.maxDist = 2.0  # maximum acceptable value from sensor in meters

    @staticmethod
    def reject_meas(z):
        if z > 2.000 or z < .020:  # 2000 mm and 20 mm bounds
            return True

# 1-d Kalman for TOF sensors
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
            # reflective/transparent surfaces or not near any walls
            # if more than 1 sec of bad meas, reset to 2 m:
            if self.rejectMeasCount > self.rejectMeasCountMax:
                self.reset()
            return self.X, self.P, 0
        else:
            self.rejectMeasCount = 0

        innErr = z - self.X
        innCov = self.P + self.R
        K = (1/innCov)*self.P
        self.X = self.X + K*innErr
        self.P = (1 - K)*self.P

        return self.X, self.P, K

# 7-d Kalman incorporating state estimate from IMU
class KalmanFilter(SensorFilter, object):
    def __init__(self, X=np.array([2.0, 2.0, 2.0, 2.0, 0.0, 0.0, 0.0]).T):
        super(KalmanFilter, self).__init__()
        # state
        # [dist_from_wall_x, dist_from_wall_y, dist_from_wall_-y, dist_from_wall_z, vel_x, vel_y, vel_z]
        self.X = X
        self.X0 = X
        # create state-transition matrix
        self.A = np.eye(7)  # will add dt in prediction
        # process noise cov guess
        self.P = np.zeros((7, 7))
        # process noise
        self.Q = np.zeros((7, 7))
        self.sigma = 0.001
        # meas noise variance
        self.R = np.eye(4)*0.002

        # bad meas
        self.rejectMeasCount = np.array([0, 0, 0, 0])

    # reset state estimate and covariance
    def reset(self, axis):
        self.X[axis] = self.maxDist
        self.P[axis, axis] = 0

    def predict_no_drone(self):
        self.A = np.eye(7)
        self.Q = np.eye(7)*0.0002
        self.X = np.dot(self.A, self.X)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        return self.X, self.A

    def predict(self, dt, vel):
        # update state transition

        # if last measurement is outside or "too small" of max distance range, do not integrate velocity
        self.A = np.eye(7)
        for i, rejectVal in enumerate(self.rejectMeasCount):
            if rejectVal == 0:
                if i == 0:
                    self.A[0, 4] = -dt
                elif i == 1:
                    self.A[1, 5] = dt
                elif i == 2:
                    self.A[2, 5] = -dt
                else:
                    self.A[3, 6] = -dt

        self.X[4] = vel[0]
        self.X[5] = vel[1]
        self.X[6] = vel[2]

        # a priori estimate of state and covariance
        self.X = np.dot(self.A, self.X)

        G = np.array([dt**2/2, dt**2/2, dt**2/2, dt**2/2, dt, dt, dt]).T
        self.Q = G*G.T*self.sigma**2

        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

        return self.X, self.P

    def update_with_measurement(self, z):
        for i, meas in enumerate(z):
            # check for bad measurements i.e. 8190 return value in case of VL53L0x
            if self.reject_meas(meas):
                # use max dist
                z[i] = self.maxDist
                self.rejectMeasCount[i] += 1
                # a series of bad measurements usually indicates either:
                # reflective/transparent surfaces or not near any walls
                # if more than 1 sec of continuous bad meas, then probably not near:
                if self.rejectMeasCount[i] > self.rejectMeasCountMax:
                    self.reset(i)
            else:
                self.rejectMeasCount[i] = 0

        # calculate H
        H = np.concatenate((np.eye(4), np.zeros((4, 3))), 1)
        # innovation residual and covariance
        innErr = z - np.dot(H, self.X)
        innCov = np.dot(H, np.dot(self.P, H.T)) + self.R
        # kalman gain
        K = np.dot(self.P, np.dot(H.T, inv(innCov)))

        # a posteriori estimate of state and covariance
        self.X = self.X + np.dot(K, innErr)
        self.P = np.dot(np.eye(7) - np.dot(K, H), self.P)

        # ensure symmetry of covariance
        self.P = (self.P + self.P.T) * 0.5

        return self.X, self.P, K

