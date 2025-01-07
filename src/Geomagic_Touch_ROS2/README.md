# Geomagic_Touch_ROS2
ROS2 Port of the ROS Driver for the Geomagic Touch Haptic Interface 

## General

This repository contains a port of the Geomagic Touch ROS driver provided by Bharatm11 at: https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers.git

## installation of Geomagic drivers 

To use the Geomagic Touch interface you first need to install both the Touch Device drivers and Openhaptics drivers provided at: (Ubuntu 20.04)
https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

## Usage
To use the ROS2 driver, connect the interface to the computer and check that there is connection
Then simply launch via:
```
ros2 launch omni_common omni_state.launch.py
```
