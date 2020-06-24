# wifi_drone_obst_avoidance

Source repository for controlling Wifi Drones with a Raspberry Pi Zero W running ROS Kinetic.


## Necessary Packages

On Raspberry Pi:

VL53L0x Python Library: https://github.com/johnbryanmoore/VL53L0X_rasp_python

PyBluez: https://github.com/pybluez/pybluez/wiki/Installation-on-Raspberry-Pi-3



On Linux PC:

ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu

Tellopy (build from source): https://github.com/hanyazou/TelloPy



## Building The Repository


```
mkdir -p ~/catkin_ws_tello/src/

cd ~/catkin_ws_tello/src/

git clone https://github.com/KhazanahAmericasInc/wifi_drone_avoidance/

cd ..

catkin build

```

