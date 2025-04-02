#!/bin/bash

#Configurer l'adresse ip
sudo ifconfig wlp0s20f3 192.168.42.146 netmask 255.255.255.0 up

#Lancer le launchfile de l'ur
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_to_touch_haptic_teleoperation ur_launch.py 
