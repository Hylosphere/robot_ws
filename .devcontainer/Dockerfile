# syntax=docker/dockerfile:1.7

# =========================
#  ARGS COMMUNS
# =========================
ARG ROS_DISTRO=foxy
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================
#  STAGE 1 : IMAGE GENERIQUE (PC Dev / Mac M2)
#  Objectif : Simulation, GUI, VNC, Outils de confort
# =========================================================
FROM ros:${ROS_DISTRO}-ros-base-focal AS image-generic
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# 1. Installation des paquets système & ROS
# On fait tout en un seul RUN pour réduire la taille de l'image (couches Docker)
RUN set -eux; \
    apt-get update; \
    # Config Timezone
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime; echo "$TZ" > /etc/timezone; \
    \
    apt-get install -y --no-install-recommends \
      # --- Outils de base ---
      build-essential cmake git openssh-client \
      python3-pip python3-venv \
      nano vim htop tmux less bash-completion \
      net-tools iputils-ping iproute2 dnsutils \
      ca-certificates lsb-release locales \
      # --- Interface Graphique (VNC / NoVNC) ---
      xvfb x11vnc dbus-x11 supervisor \
      xfce4 xfce4-goodies \
      novnc websockify \
      mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
      # --- Dépendances Robot (C++) ---
      libserial-dev \
      nlohmann-json3-dev \
      # --- ROS 2 Packages (Simu & Visu) ---
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros2-control \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ; \
    # Config Locales
    dpkg-reconfigure -f noninteractive tzdata; \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8; \
    # Nettoyage cache apt
    rm -rf /var/lib/apt/lists/*

# 2. Configuration Python & Colcon
RUN python3 -m pip install --no-cache-dir -U pip && \
    python3 -m pip install --no-cache-dir \
      rosdep vcstool colcon-common-extensions \
      pytest pytest-cov pytest-rerunfailures && \
    rosdep init || true

# 3. Auto-Sourcing ROS dans .bashrc
# Cette commande ajoute une boucle qui cherche et source setup.bash automatiquement
RUN printf '%s\n' \
'for f in /opt/ros/'"${ROS_DISTRO}"'/install/setup.bash /opt/ros/'"${ROS_DISTRO}"'/setup.bash; do' \
'  [ -f "$f" ] && . "$f" && break' \
'done' \
'[ -f /workspaces/robot_ws/install/setup.bash ] && . /workspaces/robot_ws/install/setup.bash || true' \
>> /root/.bashrc

WORKDIR /workspaces/robot_ws

# 4. Configuration VNC / Supervisor
ENV DISPLAY=:0 \
    RESOLUTION=1600x900

# Lien symbolique pour NoVNC (accès direct via http://localhost:8080/vnc.html)
RUN ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# Création propre du fichier de config Supervisor
RUN mkdir -p /etc/supervisor/conf.d && \
    printf '[supervisord]\n\
nodaemon=true\n\
\n\
[program:Xvfb]\n\
command=/usr/bin/Xvfb :0 -screen 0 1600x900x24\n\
\n\
[program:xfce4]\n\
command=/usr/bin/startxfce4\n\
\n\
[program:x11vnc]\n\
command=/usr/bin/x11vnc -display :0 -forever -shared -nopw -rfbport 5900\n\
\n\
[program:novnc]\n\
command=/usr/bin/websockify --web /usr/share/novnc/ 8080 localhost:5900\n' \
> /etc/supervisor/conf.d/supervisord.conf

# 5. Config CycloneDDS & Variables ROS
RUN printf '<?xml version="1.0" encoding="UTF-8"?>\n<CycloneDDS><Domain id="any"><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>' > /etc/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
    ROS_DOMAIN_ID=7 \
    ROS_LOCALHOST_ONLY=0

# 6. Entrypoint
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Par défaut : Lancer le bureau virtuel
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]


# =========================================================
#  STAGE 2 : IMAGE JETSON (Prod Robot) - L4T r32.7.1
#  Objectif : Performance, Pas de GUI, Pilotes Hardware
# =========================================================
FROM dustynv/ros:${ROS_DISTRO}-ros-base-l4t-r32.7.1 AS image-jetson
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Nettoyage des sources listes qui posent problème sur Jetson (packages.ros.org bionic vs focal)
RUN set -eux; \
    find /etc/apt -type f -name '*.list' -print0 | xargs -0 sed -i '/packages\.ros\.org/d'; \
    apt-get update; \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime; echo "$TZ" > /etc/timezone; \
    \
    apt-get install -y --no-install-recommends \
      build-essential cmake git openssh-client \
      python3-pip python3-venv \
      nano vim htop tmux \
      net-tools iputils-ping \
      ca-certificates lsb-release locales \
      # --- Dépendances Robot ---
      libserial-dev \
      nlohmann-json3-dev \
    ; \
    dpkg-reconfigure -f noninteractive tzdata; \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8; \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U pip colcon-common-extensions rosdep
WORKDIR /workspaces/robot_ws

# Config CycloneDDS identique au dev
RUN printf '<?xml version="1.0" encoding="UTF-8"?>\n<CycloneDDS><Domain id="any"><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>' > /etc/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash","-lc","sleep infinity"]