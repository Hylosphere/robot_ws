#!/usr/bin/env bash
set -exo pipefail

# 1) apt: petits utilitaires/tests (démo ROS)
apt-get update
apt-get install -y --no-install-recommends \
  ros-foxy-demo-nodes-cpp \
  ros-foxy-demo-nodes-py
rm -rf /var/lib/apt/lists/*

# 2) pip: extensions colcon utiles
python3 -m pip install -U pip
python3 -m pip install -U colcon-cmake colcon-common-extensions

# 3) Commande build
cat >/usr/local/bin/build <<'SH'
#!/usr/bin/env bash
set -eo pipefail   # pas de -u pour éviter AMENT_TRACE_SETUP_FILES

WS="${WORKSPACE:-/workspaces/robot_ws}"

# Source ROS 2
[ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash || true

# Source workspace si déjà build
if [ -f "$WS/install/local_setup.bash" ]; then
  source "$WS/install/local_setup.bash"
elif [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

cd "$WS"
exec colcon build --symlink-install "$@"
SH
chmod +x /usr/local/bin/build

# 4) Commande clean
cat >/usr/local/bin/clean <<'SH'
#!/usr/bin/env bash
set -eo pipefail

WS="${WORKSPACE:-/workspaces/robot_ws}"

echo "Suppression de $WS/build, $WS/install et $WS/log..."
rm -rf "$WS/build" "$WS/install" "$WS/log"
echo "Workspace nettoyé."
SH
chmod +x /usr/local/bin/clean

# 5) Vérification
command -v build
command -v clean
echo "install.sh terminé"
