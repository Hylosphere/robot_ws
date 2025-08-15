# Makefile — build/clean colcon pour ROS 2
# Utilisation :
#   make build                    # build complet
#   make clean                    # nettoie build/ install/ log/
#   make rebuild                  # clean + build
#   make build PKGS="pkg1 pkg2"   # build de paquets sélectionnés
#   make build COLCON_ARGS="--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
#   make deps                     # (optionnel) rosdep install
#   make test                     # lance colcon test

SHELL := /bin/bash
WS    := $(CURDIR)
ROS_DISTRO ?= foxy
ROS_SETUP  := /opt/ros/$(ROS_DISTRO)/setup.bash

# Arguments colcon par défaut (modifiable : make build COLCON_ARGS="...")
COLCON_ARGS ?= --symlink-install

# Sélection des paquets (ex: make build PKGS="waveshare_driver rover_bringup")
PKGS ?=
ifneq ($(strip $(PKGS)),)
  PKG_SELECT := --packages-select $(PKGS)
endif

# -------- Fonctions internes (shell) --------
define source_ros
if [ ! -f "$(ROS_SETUP)" ]; then \
  echo "❌ ROS non trouvé: $(ROS_SETUP). (Définis ROS_DISTRO=... ou installe ROS)"; \
  exit 1; \
fi; \
source "$(ROS_SETUP)"; \
if [ -f "$(WS)/install/local_setup.bash" ]; then \
  source "$(WS)/install/local_setup.bash"; \
elif [ -f "$(WS)/install/setup.bash" ]; then \
  source "$(WS)/install/setup.bash"; \
fi
endef

# -------- Cibles utilisateur --------
.PHONY: build clean rebuild deps test echo-env

build:
	@set -eo pipefail; \
	$(call source_ros); \
	cd "$(WS)"; \
	echo "▶️  colcon build $(COLCON_ARGS) $(PKG_SELECT)"; \
	colcon build $(COLCON_ARGS) $(PKG_SELECT)

clean:
	@set -e; \
	echo "🧹 Suppression de $(WS)/build $(WS)/install $(WS)/log"; \
	rm -rf "$(WS)/build" "$(WS)/install" "$(WS)/log"; \
	echo "✅ Workspace nettoyé."

rebuild: clean build

deps:
	@set -eo pipefail; \
	$(call source_ros); \
	cd "$(WS)"; \
	rosdep update; \
	rosdep install --from-paths src --rosdistro $(ROS_DISTRO) -i -y || true

test:
	@set -eo pipefail; \
	$(call source_ros); \
	cd "$(WS)"; \
	colcon test --event-handlers console_cohesion+ --return-code-on-test-failure

echo-env:
	@echo "WS=$(WS)"; \
	echo "ROS_DISTRO=$(ROS_DISTRO)"; \
	echo "ROS_SETUP=$(ROS_SETUP)"; \
	echo "COLCON_ARGS=$(COLCON_ARGS)"; \
	echo "PKGS=$(PKGS)"
