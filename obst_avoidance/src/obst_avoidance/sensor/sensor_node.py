#!/usr/bin/env python
import rospy
from obst_avoidance.msg import sensor_meas
from std_msgs.msg import String
import bluetooth

# B8:27:EB:7D:75:63 raspberry pi
# 94:E9:79:E6:E1:90 pc

class Sensor_Node():
    def __init__(self):
        # ROS node and topic setup
        rospy.init_node("Sensor_Node", anonymous=False)
        rospy.Subscriber("cmd_sensor", String, self.__action)
        self.pub_meas = rospy.Publisher('sensor_meas', sensor_meas, queue_size=1)
        # Bluetooth parameters
        self.port = rospy.get_param('bt_port', 3)
        self.backlog = 1
        self.size = 1024
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.settimeout(rospy.get_param('sensor_timeout', 5.0))

        self.bt_on = False
        self.listening = False

    # connect to port and listen for
    def connect(self):
        if self.bt_on:
            return
        # bind to bluetooth socket
        try:
            self.sock.bind(('', self.port))
            self.sock.listen(self.backlog)

        except Exception as e:
            print(e)
            return

        self.bt_on = True

    # disconnect client and close socket
    def disconnect(self):
        if not self.bt_on:
            return
        # try to close socket, if still listening
        try:
            self.sock.close()

        except Exception as e:
            print(e)
            return

        self.bt_on = False

    def __action(self, msg):
        if msg.data == "connect":
            self.connect()
        elif msg.data == "disconnect":
            self.disconnect()
        elif msg.data == "start":
            if self.bt_on:
                self.listening = True
            else:
                print("Bind BT socket first.")
        elif msg.data == "stop":
            if self.bt_on:
                self.listening = False
            else:
                print("Bind BT socket first.")

if __name__ == '__main__':
    s = Sensor_Node()

    client = None
    client_created = False

    while not rospy.is_shutdown():
        # wait for main to enable bt
        if not s.listening:
            rospy.sleep(0.1)
            continue
        # listen, catch errors
        try:
            # create client once, if not done already
            if not client_created:
                print("Looking for Sensor...")
                client, clientInfo = s.sock.accept()
                client_created = True
                print("Sensor Connected.")
            # receive data, hangs waiting
            data = client.recv(s.size)
            if data:
                # [count, d1, d2, d3, d4]
                d = [int(i) for i in data.split(',')]
                msg = sensor_meas()
                msg.meas = d[1:5]
                msg.header.stamp = rospy.Time.now()
                msg.header.seq = d[0]
                s.pub_meas.publish(msg)
        # errors from board side/connection issues
        except bluetooth.btcommon.BluetoothError as btErr:
            # if no board, shutdown socket and node
            if str(btErr) == "timed out":
                print("Could not find board!")
                rospy.signal_shutdown("No board.")
                continue

            errCode = eval(btErr[0])[0]
            if client_created:
                client.close()
                client_created = False
        except Exception as e:
            print(e)
