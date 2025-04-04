#!/bin/bash

# Installation de ros2
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop-full -y

# Sourcing de ROS 2 après son installation
source /opt/ros/humble/setup.bash

# Installation des dépendances du projet
sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    net-tools \
    nano \
    libncurses-dev \
    libncurses5 \
    libncurses5-dev \
    libncursesw5 \
    libncursesw5-dev \
    python3-vcstool \
    python3-rosdep \
    ament-cmake \
    python3-colcon-common-extensions \
    wget \
    ros-humble-rosidl-default-generators \
    tree \
    ros-humble-ur \
    ros-humble-ros2-control \
    libmsgpack-dev \
    python3-pip -y

# Installation de dépendances Python
pip install HaplyHardwareAPI

# Ajout de l'utilisateur au groupe dialout
sudo usermod -a -G dialout $USER

# Build du workspace
colcon build

# Sourcing du setup.bash après build
source $(pwd)/install/setup.bash

# Ajouter le sourcing à .bashrc pour les futures sessions
echo "source $(pwd)/install/setup.bash" >> /home/$USER/.bashrc
