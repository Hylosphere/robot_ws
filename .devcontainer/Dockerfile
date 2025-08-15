# Dockerfile
ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# 1) Bases système + miroirs ARM64 + outils build
RUN set -eux; \
  sed -i 's|http://archive.ubuntu.com/ubuntu|http://ports.ubuntu.com/ubuntu-ports|g' /etc/apt/sources.list || true; \
  sed -i 's|http://security.ubuntu.com/ubuntu|http://ports.ubuntu.com/ubuntu-ports|g' /etc/apt/sources.list || true; \
  printf 'Acquire::Retries "5";\nAcquire::http::No-Cache "true";\n' > /etc/apt/apt.conf.d/80-retries; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    locales tzdata ca-certificates curl gnupg2 lsb-release \
    python3 python3-pip python3-venv \
    build-essential git cmake; \
  rm -rf /var/lib/apt/lists/*; \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime; dpkg-reconfigure -f noninteractive tzdata; \
  locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
  openssh-client \
  && rm -rf /var/lib/apt/lists/*


# 2) Dépôt ROS 2 (clé via keyring, pas apt-key)
RUN set -eux; \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg; \
  echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros2.list; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    ros-foxy-ros-base \
    ros-foxy-ros2-controllers \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-rmw-cyclonedds-cpp \
    python3-rosdep; \
  rm -rf /var/lib/apt/lists/*

# 3) rosdep (tolérant si relancé plusieurs fois)
RUN set -eux; \
  rosdep init || true; \
  rosdep update

# 4) Outils dev ROS: colcon + vcstool + tests
RUN set -eux; \
  python3 -m pip install --no-cache-dir -U pip; \
  python3 -m pip install --no-cache-dir \
    colcon-common-extensions \
    vcstool \
    pytest pytest-cov pytest-rerunfailures

# 5) Sourcing ROS pour tous les shells (auto, tolérant si le build n'a pas encore été fait)
RUN echo 'if [ -f /opt/ros/foxy/setup.bash ]; then source /opt/ros/foxy/setup.bash; fi' >> /root/.bashrc \
 && echo 'if [ -f /workspaces/robot_ws/install/local_setup.bash ]; then source /workspaces/robot_ws/install/local_setup.bash; \
          elif [ -f /workspaces/robot_ws/install/setup.bash ]; then source /workspaces/robot_ws/install/setup.bash; fi' >> /root/.bashrc

# 6) Dossier de travail (ton workspace)
WORKDIR /workspaces/robot_ws
