#!/usr/bin/env python
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger
import time


# Inherits tellopy class and exposes what is needed for state feedback

class Tello(tello.Tello):

    def __init__(self, ip, port, timeout):
        # init tellopy library
        super(Tello, self).__init__()

        # not necessary with Tellopy
        self.ip = ip
        self.port = port

        # state variables of interest
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.recv_t = 0.0

        # new data to publish
        self.new_data = False

        self.timeout = timeout


    # update_state
    def update_state(self, event, sender, data, **args):
        drone = sender
        if event is drone.EVENT_LOG_DATA:
            d = [float(i) for i in data.format_cvs().split(',')]
            self.recv_t = time.time()  # time since epoch in sec

            # corrections to meters and seconds
            self.acc = [9.81 * d[6], -9.81 * d[7], -9.81 * d[8]]
            self.gyro = [d[9], d[10], d[11]]
            self.q = [d[12], d[13], d[14], d[15]]
            self.vel = [d[16], d[17], -d[18]]

            self.new_data = True

        #if event is drone.EVENT_FLIGHT_DATA:
        #    return

    def toggle_new_data(self):
        self.new_data = not self.new_data

    #
    # Functions for action commands from main
    #
    # connect to drone socket
    def drone_connect(self):
        #self.subscribe(self.EVENT_FLIGHT_DATA, self.update_state)
        self.subscribe(self.EVENT_LOG_DATA, self.update_state)

        self.connect()
        try:
            self.wait_for_connection(self.timeout)
        except:
            print('ERROR: Could not find drone!')
            return False
        print("Drone Connected.")
        return True

    # disconnect from socket
    def drone_disconnect(self):
        self.quit()

    # prompt to land drone
    # tello API makes this very easy
    def drone_land(self):
        self.land()

    # prompt for drone takeoff
    def drone_takeoff(self):
        self.takeoff()

    #
    # Functions for commanding velocities
    #
    # send command
    def cmd_vel(self, cmd):
        self.set_roll(cmd[0])
        self.set_pitch(cmd[1])
        self.set_yaw(cmd[2])
        self.set_throttle(cmd[3])
