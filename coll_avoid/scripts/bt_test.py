# Test script for raspberry pi + sensor module
# With the module running, execute this program using: rosrun coll_avoid bt_test.py

#!/usr/bin/env python
import bluetooth

server_address = ''
port = 3
backlog = 1
size = 1024
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind((server_address, port))
s.listen(backlog)
try:
    client, clientInfo = s.accept()
    while 1:
        data = client.recv(size)
        if data:
            print(data)
except:
    print("Closing socket")
    client.close()
    s.close()

