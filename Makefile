# Makefile pour Robot WS

SHELL := /bin/bash
WS    := $(CURDIR)
ROS_DISTRO ?= foxy

# -------------------------------
# ROS 2 Build System (Inside Container)
# -------------------------------
build:
	@set -eo pipefail; \
	source /opt/ros/$(ROS_DISTRO)/setup.bash; \
	colcon build --symlink-install

clean:
	rm -rf build install log

rebuild: clean build

deps:
	source /opt/ros/$(ROS_DISTRO)/setup.bash; \
	rosdep update; \
	rosdep install --from-paths src --rosdistro $(ROS_DISTRO) -i -y || true

# -------------------------------
# Dev (Mac M2 - Mode VNC)
# -------------------------------
dev-build:
	# On force le rebuild sans cache pour bien installer le bureau VNC
	docker compose --profile dev build --no-cache rover

dev-up:
	# Lancement du conteneur (Accès via http://localhost:8080)
	docker compose --profile dev up -d rover

dev-shell:
	docker compose --profile dev exec rover bash

dev-logs:
	docker compose --profile dev logs -f rover

dev-down:
	docker compose --profile dev down

# -------------------------------
# Jetson (Production)
# -------------------------------
jetson-build:
	docker compose --profile jetson build --no-cache rover-jetson

jetson-up:
	docker compose --profile jetson up -d rover-jetson

jetson-shell:
	docker compose --profile jetson exec rover-jetson bash

jetson-logs:
	docker compose --profile jetson logs -f rover-jetson

jetson-down:
	docker compose --profile jetson down

# -------------------------------
# Utilitaires Docker
# -------------------------------
ps:
	docker compose --profile dev ps || true
	docker compose --profile jetson ps || true

prune:
	docker system prune -af || true