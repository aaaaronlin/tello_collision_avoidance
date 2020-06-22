# wifi_drone_obst_avoidance

Source repository for controlling Wifi Drones with a Raspberry Pi Zero W running ROS Kinetic.


## Necessary Packages

On Raspberry Pi:

```
pip install tellopy


sudo apt-get install build-essential python-dev

cd your_git_directory

git clone https://github.com/johnbryanmoore/VL53L0X_rasp_python.git

cd VL53L0X_rasp_python

make

```


## Building The Repository


```
mkdir -p ~/catkin_ws_tello/src/

cd ~/catkin_ws_tello/src/

git clone https://github.com/KhazanahAmericasInc/wifi_drone_avoidance/

cd ..

catkin build

```

