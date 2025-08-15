#!/usr/bin/env bash
set -eo pipefail

WS="/workspaces/robot_ws"

# 1) Paquets utiles pour tests rapides et contrôle ROS 2
apt-get update
apt-get install -y --no-install-recommends \
  ros-foxy-demo-nodes-cpp \
  ros-foxy-demo-nodes-py \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers
rm -rf /var/lib/apt/lists/*


# 2) Outils Python pour ROS 2
python3 -m pip install -U pip
python3 -m pip install -U colcon-cmake colcon-common-extensions

# 3) Corrige les permissions de tout ce qu’il y a dans scripts/
if [ -f "$WS/scripts/fix_permission.sh" ]; then
  bash "$WS/scripts/fix_permission.sh"
fi

# 4) Wrappers build/clean (liens symboliques vers scripts/)
mkdir -p /usr/local/bin
ln -sf "$WS/scripts/build" /usr/local/bin/build
ln -sf "$WS/scripts/clean" /usr/local/bin/clean

# 5) Petit check
command -v build && command -v clean && echo "[install.sh] OK"
