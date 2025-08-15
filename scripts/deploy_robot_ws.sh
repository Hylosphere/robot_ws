#!/usr/bin/env bash
set -euo pipefail

REPO_SSH="git@github.com:xylophone97/robot_ws.git"
WS="${HOME}/robot_ws"

# --- utilitaires ---
have_cmd() { command -v "$1" >/dev/null 2>&1; }
have_pkg() { dpkg -s "$1" >/dev/null 2>&1; }

ensure_ros2_control() {
  local -a pkgs=(ros-foxy-ros2-control ros-foxy-ros2-controllers)
  local -a missing=()
  for p in "${pkgs[@]}"; do have_pkg "$p" || missing+=("$p"); done
  if [ "${#missing[@]}" -gt 0 ]; then
    echo "[apt] Installation des paquets: ${missing[*]}"
    sudo apt-get update
    if ! sudo apt-get install -y --no-install-recommends "${missing[@]}"; then
      echo "⚠️  Impossible d’installer ${missing[*]}."
      echo "   - Vérifie que ROS 2 Foxy est installé (Ubuntu 20.04)"
      echo "   - Vérifie que le dépôt ROS (packages.ros.org) est configuré"
      echo "   - Commande utile: sudo apt-cache policy ros-foxy-ros2-control"
    fi
  else
    echo "[apt] ros2_control et ros2_controllers déjà présents."
  fi
}

ensure_colcon() {
  if ! have_cmd colcon; then
    echo "[pip] Installation colcon-common-extensions + colcon-cmake"
    python3 -m pip install -U pip
    python3 -m pip install -U colcon-common-extensions colcon-cmake
  fi
}

fix_permissions_if_any() {
  if [ -d "$WS/scripts" ]; then
    echo "[fix-permissions] Mise à jour des droits dans $WS/scripts/"
    find "$WS/scripts" -type f -name "*.sh" -exec chmod +x {} \;
  fi
}

# --- clone / update ---
if [ ! -d "$WS/.git" ]; then
  echo "[git] Repo absent, clonage depuis $REPO_SSH"
  git clone "$REPO_SSH" "$WS"
else
  echo "[git] fetch + reset sur origin/main"
  git -C "$WS" fetch --all --prune
  git -C "$WS" reset --hard origin/main
  git -C "$WS" submodule update --init --recursive || true
fi

# --- prérequis ---
ensure_ros2_control
ensure_colcon
fix_permissions_if_any

# --- build local (PAS de Docker sur la Jetson) ---
echo "[build] Build local avec colcon"
if [ -f /opt/ros/foxy/setup.bash ]; then
  # pas de -u (certaines var AMENT ne sont pas exportées au début)
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
fi

cd "$WS"
colcon build --symlink-install

echo "✅ Déploiement terminé."
