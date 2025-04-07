#!/bin/bash

# Détection des interfaces Ethernet actives
ETH_IFACES=$(ip -o link show up | awk -F': ' '{print $2}' | grep -E '^e(n|th)')

# Vérification si des interfaces Ethernet ont été détectées
if [ -z "$ETH_IFACES" ]; then
  echo "Aucune interface Ethernet active détectée."
  exit 1
fi

# Affichage des interfaces détectées
echo "Interfaces Ethernet détectées :"
echo "$ETH_IFACES"

# Fonction pour configurer une interface avec une IP statique
configure_ip() {
  local iface=$1
  local ip=$2
  local netmask=$3
  
  # Vérifier si l'interface existe
  if ! ip link show "$iface" &>/dev/null; then
    echo "L'interface $iface n'existe pas."
    return 1
  fi
  
  # Attribuer l'adresse IP statique à l'interface
  sudo ip addr add "$ip"/"$netmask" dev "$iface"
  if [ $? -ne 0 ]; then
    echo "Erreur lors de la configuration de l'adresse IP sur $iface."
    return 1
  fi

  # Activer l'interface
  sudo ip link set "$iface" up

  echo "IP statique $ip configurée sur $iface"
}

# Configurer la première interface
FIRST_IFACE=$(echo "$ETH_IFACES" | head -n 1)
configure_ip "$FIRST_IFACE" "192.168.56.1" "24"

# Configurer la deuxième interface (si elle existe)
SECOND_IFACE=$(echo "$ETH_IFACES" | sed -n '2p')
if [ -n "$SECOND_IFACE" ]; then
  configure_ip "$SECOND_IFACE" "192.168.42.130" "24"
else
  echo "Aucune deuxième interface Ethernet active détectée."
fi


#Lancer le launchfile de l'ur
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_control ur_launch.py 
sleep 2000
ros2 control switch_controllers --activate forward_position_controller --deactivate scaled_joint_trajectory_controller
