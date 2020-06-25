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

        self.listening = False

    # connect to port and listen for
    def connect(self):
        try:
            self.sock.bind(('', self.port))
            self.sock.listen(self.backlog)

        except Exception as e:
            print(e)
            return

        self.listening = True

    # disconnect client and close socket
    def disconnect(self):
        try:
            self.sock.close()

        except Exception as e:
            print(e)
            return

        self.listening = False

    def __action(self, msg):
        print(msg.data)


if __name__ == '__main__':
    s = Sensor_Node()

    s.connect()

    client = None

    while not rospy.is_shutdown():
        # hangs until a message comes in
        try:
            client, clientInfo = s.sock.accept()

            data = client.recv(s.size)
            if data:
                print(data)
        except:
            break

    if client is not None:
        client.close()

    s.disconnect()
