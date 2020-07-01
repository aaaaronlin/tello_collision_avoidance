# wifi_drone_obst_avoidance

Source repository for controlling Wifi Drones with a Raspberry Pi Zero W running ROS Kinetic.


## Setup Packages

On Raspberry Pi:

Dependencies:

```

sudo apt-get update
sudo apt-get install python-pip python-dev ipython

sudo apt-get install bluetooth libbluetooth-dev

```

VL53L0x Python Library: https://github.com/pimoroni/VL53L0X_rasp_python


```

sudo python2 setup.py install

```

PyBluez 0.22: https://github.com/pybluez/pybluez
(use sudo pip install pybluez==0.22)



On Linux PC:

ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu

Tellopy (build from source): https://github.com/hanyazou/TelloPy

PyBluez 0.22: https://github.com/pybluez/pybluez
(use sudo pip install pybluez==0.22)

## Building The Repository


```

mkdir -p ~/catkin_ws_tello/src/

cd ~/catkin_ws_tello/src/

git clone https://github.com/KhazanahAmericasInc/wifi_drone_avoidance/

cd ..

catkin build

```

## Using the Repository


