#!/bin/bash

#Configurer l'adresse ip
ETH_IFACE=$(ip -o link show up | awk -F': ' '{print $2}' | grep -E '^e(n|th)' | head -n1)

if [ -z "$ETH_IFACE" ]; then
  echo "Aucune interface Ethernet active détectée."
  exit 1
fi

echo "Interface Ethernet détectée : $ETH_IFACE"

# Attribuer IP statique
sudo ifconfig "$ETH_IFACE" 192.168.42.130 netmask 255.255.255.0 up
echo "IP statique configurée sur $ETH_IFACE"

# Ajouter une route pour forcer le trafic via cette interface
sudo ip route add 192.168.42.0/24 dev "$ETH_IFACE"
echo "Route ajoutée pour le sous-réseau 192.168.42.0/24"
#Lancer le launchfile de l'ur
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_control ur_launch.py 
sleep 10
ros2 control switch_controllers --activate forward_position_controller --deactivate scaled_joint_trajectory_controller
