#!/bin/bash

#Configurer l'adresse ip
sudo ifconfig wlp1s0 192.168.1.200 netmask 255.255.255.0 up

#Lancer le launchfile de l'ur
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_to_touch_haptic_teleoperation ur_launch.py 