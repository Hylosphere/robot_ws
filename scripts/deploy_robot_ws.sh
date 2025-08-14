#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/robot_ws"

if [ ! -d "$WS/.git" ]; then
  echo "Repo absent, clonage…"
  git clone git@github.com:xylophone97/robot_ws.git "$WS"
fi

cd "$WS"
echo "[git] fetch + reset sur origin/main"
git fetch --all --prune
git reset --hard origin/main
git submodule update --init --recursive || true

# Mise à jour des permissions des scripts
if [ -d "$WS/scripts" ]; then
  echo "[fix-permissions] Mise à jour des droits d'exécution sur scripts/"
  chmod +x "$WS/scripts/"*.sh 2>/dev/null || true
fi

# Build si Docker installé ET que ce n'est pas un devcontainer Mac
if command -v docker >/dev/null 2>&1 && [ -z "${DEVCONTAINER:-}" ]; then
  echo "🐳 Docker détecté. Build via conteneur..."
  docker run --rm -it \
    --network host \
    -v "$WS":/workspaces/robot_ws \
    -w /workspaces/robot_ws \
    vsc-robot_ws-*:latest \
    bash -lc 'source /opt/ros/foxy/setup.bash && colcon build --symlink-install'
else
  echo "⚙️ Build local sur la Jetson..."
  source /opt/ros/foxy/setup.bash || true
  colcon build --symlink-install || true
fi

echo "✅ Déploiement terminé."
