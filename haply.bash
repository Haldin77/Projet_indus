#!/bin/bash

#Configurer l'adresse ip
ETH_IFACE=$(ip -o link show up | awk -F': ' '{print $2}' | grep -E '^e(n|th)' | head -n1)

if [ -z "$ETH_IFACE" ]; then
  echo "Aucune interface Ethernet active détectée."
  exit 1
fi

echo "Interface Ethernet détectée : $ETH_IFACE"

# Attribuer IP statique
sudo ifconfig "$ETH_IFACE" 192.168.42.146 netmask 255.255.255.0 up
echo "IP statique configurée sur $ETH_IFACE"

# Ajouter une route pour forcer le trafic via cette interface
sudo ip route add 192.168.42.0/24 dev "$ETH_IFACE"
echo "Route ajoutée pour le sous-réseau 192.168.42.0/24"
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1

#Lancer le launchfile du phantom
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_control haply_launch.py 