# syntax=docker/dockerfile:1.7

# =========================
#  Args communs
# =========================
ARG ROS_DISTRO=foxy
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================
#  STAGE 1 : IMAGE GENERIQUE (PC dev) - Ubuntu 20.04 / Focal
#  Base officielle OSRF avec ROS 2 Foxy installé via APT
# =========================================================
FROM ros:${ROS_DISTRO}-ros-base-focal AS image-generic
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Outils de base
RUN set -eux; \
    apt-get update; \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime; echo "$TZ" > /etc/timezone; \
    apt-get install -y --no-install-recommends \
      python3-pip python3-venv \
      build-essential git cmake openssh-client \
      nano less bash-completion net-tools iputils-ping \
      iproute2 dnsutils usbutils udev vim htop tmux ripgrep \
      ca-certificates lsb-release locales \
      # ✅ assure CycloneDDS présent côté dev
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ; \
    dpkg-reconfigure -f noninteractive tzdata; \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8; \
    rm -rf /var/lib/apt/lists/*

# Outils Python & colcon
RUN python3 -m pip install --no-cache-dir -U pip && \
    python3 -m pip install --no-cache-dir \
      rosdep vcstool colcon-common-extensions \
      pytest pytest-cov pytest-rerunfailures && \
    rosdep init || true

# Sourcing automatique (handles /opt/ros/${ROS_DISTRO} et /opt/ros/${ROS_DISTRO}/install)
RUN printf '%s\n' \
'for f in /opt/ros/'"${ROS_DISTRO}"'/install/setup.bash /opt/ros/'"${ROS_DISTRO}"'/setup.bash /opt/ros/'"${ROS_DISTRO}"'/install/setup.sh /opt/ros/'"${ROS_DISTRO}"'/setup.sh; do' \
'  [ -f "$f" ] && . "$f" && break' \
'done' \
'[ -f /workspaces/robot_ws/install/local_setup.bash ] && . /workspaces/robot_ws/install/local_setup.bash || true' \
'[ -f /workspaces/robot_ws/install/setup.bash       ] && . /workspaces/robot_ws/install/setup.bash       || true' \
>> /root/.bashrc

WORKDIR /workspaces/robot_ws

# CycloneDDS par défaut (interface auto)
RUN printf '%s\n' \
'<?xml version="1.0" encoding="UTF-8"?>' \
'<CycloneDDS>' \
'  <Domain id="any">' \
'    <General>' \
'      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>' \
'      <AllowMulticast>spdp</AllowMulticast>' \
'    </General>' \
'    <Discovery>' \
'      <ParticipantIndex>auto</ParticipantIndex>' \
'    </Discovery>' \
'  </Domain>' \
'</CycloneDDS>' > /etc/cyclonedds.xml

# Env ROS/DDS par défaut (override possibles via compose)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
    ROS_DOMAIN_ID=7 \
    ROS_LOCALHOST_ONLY=0

# Entrypoint commun
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Garde le conteneur vivant par défaut
CMD ["bash","-lc","sleep infinity"]

# =========================================================
#  STAGE 2 : IMAGE JETSON (prod robot) - L4T r32.7.1 / JP4.6
#  Base NVIDIA : ROS2 Foxy déjà présent sous /opt/ros/foxy/install
# =========================================================
FROM dustynv/ros:${ROS_DISTRO}-ros-base-l4t-r32.7.1 AS image-jetson
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# IMPORTANT: supprimer les dépôts ros.org (clé expirée sur bionic) pour éviter les erreurs d'apt
RUN set -eux; \
    find /etc/apt -type f -name '*.list' -print0 | xargs -0 sed -i '/packages\.ros\.org/d'; \
    apt-get update; \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime; echo "$TZ" > /etc/timezone; \
    apt-get install -y --no-install-recommends \
      python3-pip python3-venv \
      build-essential git cmake openssh-client \
      nano less bash-completion net-tools iputils-ping \
      iproute2 dnsutils usbutils udev vim htop tmux \
      ca-certificates lsb-release locales \
    ; \
    dpkg-reconfigure -f noninteractive tzdata; \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8; \
    rm -rf /var/lib/apt/lists/*

# Outils Python & colcon via pip (évite APT rosdep sur bionic)
RUN python3 -m pip install --no-cache-dir -U pip && \
    python3 -m pip install --no-cache-dir \
      rosdep vcstool colcon-common-extensions \
      pytest pytest-cov pytest-rerunfailures && \
    rosdep init || true

# Sourcing automatique (gère /opt/ros/foxy/install/* sur Jetson)
RUN printf '%s\n' \
'for f in /opt/ros/'"${ROS_DISTRO}"'/install/setup.bash /opt/ros/'"${ROS_DISTRO}"'/setup.bash /opt/ros/'"${ROS_DISTRO}"'/install/setup.sh /opt/ros/'"${ROS_DISTRO}"'/setup.sh; do' \
'  [ -f "$f" ] && . "$f" && break' \
'done' \
'[ -f /workspaces/robot_ws/install/local_setup.bash ] && . /workspaces/robot_ws/install/local_setup.bash || true' \
'[ -f /workspaces/robot_ws/install/setup.bash       ] && . /workspaces/robot_ws/install/setup.bash       || true' \
>> /root/.bashrc

WORKDIR /workspaces/robot_ws

# CycloneDDS en auto (l’entrypoint peut remplacer l’interface au runtime si DDS_INTERFACE est fourni)
RUN printf '%s\n' \
'<?xml version="1.0" encoding="UTF-8"?>' \
'<CycloneDDS>' \
'  <Domain id="any">' \
'    <General>' \
'      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>' \
'      <AllowMulticast>spdp</AllowMulticast>' \
'    </General>' \
'    <Discovery>' \
'      <ParticipantIndex>auto</ParticipantIndex>' \
'    </Discovery>' \
'  </Domain>' \
'</CycloneDDS>' > /etc/cyclonedds.xml

# Env par défaut
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
    ROS_DOMAIN_ID=7 \
    ROS_LOCALHOST_ONLY=0

# Entrypoint (ne lance pas ros2)
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash","-lc","sleep infinity"]
