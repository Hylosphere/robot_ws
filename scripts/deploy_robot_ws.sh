#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/robot_ws"

# Cloner si le dépôt n'existe pas encore
if [ ! -d "$WS/.git" ]; then
  echo "Repo absent, clonage…"
  git clone git@github.com:xylophone97/robot_ws.git "$WS"
fi

cd "$WS"
echo "[git] fetch + reset sur origin/main"
git fetch --all --prune
git reset --hard origin/main
git submodule update --init --recursive || true

# Charger ROS 2 Foxy
set +u
source /opt/ros/foxy/setup.bash || true
set -u

# Build du workspace
colcon build --symlink-install

echo "✅ Déploiement terminé."
