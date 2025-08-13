#!/bin/bash
set -eux

# Met à jour les dépôts
apt-get update

# Installe les paquets ROS utiles pour tes tests
apt-get install -y \
    ros-foxy-demo-nodes-cpp \
    ros-foxy-demo-nodes-py

# Installe colcon_cmake si nécessaire
pip3 install -U colcon-cmake

# Nettoie le cache apt
rm -rf /var/lib/apt/lists/*
