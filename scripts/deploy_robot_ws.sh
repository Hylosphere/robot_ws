#!/usr/bin/env bash
set -eo pipefail

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

# Permissions des scripts
if [ -d "$WS/scripts" ]; then
  echo "[fix-permissions] Mise à jour des droits d'exécution sur scripts/"
  chmod +x "$WS/scripts/"*.sh 2>/dev/null || true
fi

# --- Politique de build ---
# Par défaut: build LOCAL (Jetson)
# Pour activer Docker, exporter USE_DOCKER=1 et fournir DEV_IMAGE si besoin.
USE_DOCKER="${USE_DOCKER:-0}"
DEV_IMAGE="${DEV_IMAGE:-}"

if [ "$USE_DOCKER" = "1" ] && command -v docker >/dev/null 2>&1; then
  if [ -z "$DEV_IMAGE" ]; then
    echo "❌ USE_DOCKER=1 mais aucune image n'est fournie (DEV_IMAGE vide)."
    echo "   Exemple: USE_DOCKER=1 DEV_IMAGE=vsc-robot_ws:latest ./scripts/deploy_robot_ws.sh"
    exit 1
  fi
  echo "🐳 Build via conteneur Docker: $DEV_IMAGE"
  docker run --rm -it \
    --network host \
    -v "$WS":/workspaces/robot_ws \
    -w /workspaces/robot_ws \
    "$DEV_IMAGE" \
    bash -lc 'source /opt/ros/foxy/setup.bash && colcon build --symlink-install'
else
  echo "⚙️ Build local (sans Docker)…"
  # Source ROS si présent (Jetson)
  [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash || true
  colcon build --symlink-install
fi

echo "✅ Déploiement terminé."
