#!/usr/bin/env python

# a general PID with anti-windup
class PID():
    def __init__(self, kp=0.2, ki=0.1, kd=0, k_min=-1.0, k_max=1.0):
        # standard parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.E = 0.0
        self.e = 0.0
        self.e_dot = 0.0
        self.e_old = 0.0

        # anti-windup
        self.k_min = k_min
        self.k_max = k_max

    def reset(self):
        self.E = 0.0
        self.e = 0.0
        self.e_dot = 0.0
        self.e_old = 0.0

    def run(self, dist, dist_ref):
        self.e = dist_ref - dist

        k_des = self.kp*self.e + self.ki*(self.E + self.e) + self.kd*(self.e - self.e_old)

        # if over max output, and positive error
        if k_des >= self.k_max and self.e >= 0:
            k = self.k_max
        # if under min output, and negative error
        elif k_des <= self.k_min and self.e <= 0:
            k = self.k_min
        # if within bounds, continue integration
        else:
            k = k_des
            self.E = self.E + self.e

        self.e_dot = self.e - self.e_old
        self.e_old = self.e

        return k
