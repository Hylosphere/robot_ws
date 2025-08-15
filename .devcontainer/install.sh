#!/usr/bin/env bash
set -eo pipefail

WS="${WORKSPACE:-/workspaces/robot_ws}"

echo "[install] apt ROS 2 utilitaires + ros2_control..."
apt-get update
apt-get install -y --no-install-recommends \
  ros-foxy-demo-nodes-cpp \
  ros-foxy-demo-nodes-py  \
  ros-foxy-ros2-control   \
  ros-foxy-ros2-controllers
rm -rf /var/lib/apt/lists/*

echo "[install] pip outils colcon..."
python3 -m pip install -U pip
python3 -m pip install -U colcon-cmake colcon-common-extensions

# Colcon peut avoir été installé dans /usr/local/bin : on s'assure que PATH le voit
export PATH="/usr/local/bin:${PATH}"
hash -r || true

# Vérif non bloquante (ne pas faire échouer le postCreate)
if ! command -v colcon >/dev/null 2>&1; then
  echo "⚠️  colcon non trouvé dans PATH (${PATH})."
  echo "    Essayez 'export PATH=/usr/local/bin:\$PATH' puis 'which colcon'."
else
  echo "✅ colcon: $(command -v colcon)"
fi

# Permissions du dossier scripts/ si présent
if [ -f "$WS/scripts/fix_permission.sh" ]; then
  echo "[install] correction des permissions dans scripts/..."
  bash "$WS/scripts/fix_permission.sh" || true
fi

echo "✅ install.sh terminé."
