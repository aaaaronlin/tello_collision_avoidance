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
        rospy.Subscriber("cmd_rpi", String, self.__action)
        self.pub_meas = rospy.Publisher('sensor_meas', sensor_meas, queue_size=1)
        # Bluetooth parameters
        self.port = rospy.get_param('bt_port', 3)
        self.backlog = 1
        self.size = 1024
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        self.connected = False

    # connect to port and listen for
    def connect(self):
        if self.connected:
            return
        # try to listen to bluetooth socket
        try:
            self.sock.bind(('', self.port))
            self.sock.listen(self.backlog)

        except Exception as e:
            print(e)
            return

        self.connected = True

    # disconnect client and close socket
    def disconnect(self):
        if not self.connected:
            return
        # try to close socket, if still listening
        try:
            self.sock.close()

        except Exception as e:
            print(e)
            return

        self.connected = False

    def __action(self, msg):
        print(msg)


if __name__ == '__main__':
    s = Sensor_Node()

    s.connect()

    client = None
    client_created = False

    while not rospy.is_shutdown():
        # try to publish message from rpi
        try:
            # create client once
            if not client_created:
                client, clientInfo = s.sock.accept()
                client_created = True
                print("Client Created.")
            # receive data, hangs waiting
            data = client.recv(s.size)
            if data:
                d = [int(i) for i in data.split(',')]
                msg = sensor_meas()
                msg.meas = d
                s.pub_meas.publish(msg)
        except Exception as e:
            print("BT Error: " + str(e))
            break

    if client is not None:
        client.close()

    s.disconnect()
