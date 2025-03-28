#!/bin/bash

#Configurer l'adresse ip
sudo ifconfig wlp0s20f3 192.168.42.146 netmask 255.255.255.0 up

sudo chmod 777 /dev/ttyACM0

#Lancer le launchfile du phantom
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_to_touch_haptic_teleoperation phantom_launch.py 