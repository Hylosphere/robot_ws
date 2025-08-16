#!/usr/bin/env bash
# Robust bootstrap + shell entrypoint (Jetson/PC)
set -eE -o pipefail

# -------------------------
# Paramètres
# -------------------------
ROS_DISTRO="${ROS_DISTRO:-foxy}"
WS_DIR="${WS_DIR:-/workspaces/robot_ws}"
REPO_URL="${REPO_URL:-git@github.com:xylophone97/robot_ws.git}"
REPO_BRANCH="${REPO_BRANCH:-main}"
DDS_INTERFACE="${DDS_INTERFACE:-auto}"

log()  { echo "[entrypoint] $*"; }
warn() { echo "[entrypoint][WARN] $*" >&2; }
err()  { echo "[entrypoint][ERROR] $*" >&2; }

# Eviter 'unbound variable' dans les scripts ROS
export COLCON_TRACE="${COLCON_TRACE:-0}"

# Sourcing sans nounset
safe_source() { set +u; . "$1"; set -u; }

# -------------------------
# 1) Source ROS si dispo
# -------------------------
set -u
source_any() {
  for f in \
    "/opt/ros/${ROS_DISTRO}/install/setup.bash" \
    "/opt/ros/${ROS_DISTRO}/setup.bash" \
    "/opt/ros/${ROS_DISTRO}/install/setup.sh" \
    "/opt/ros/${ROS_DISTRO}/setup.sh"
  do
    [ -f "$f" ] && { safe_source "$f"; return 0; }
  done
  return 1
}
if ! source_any; then
  warn "/opt/ros/${ROS_DISTRO}/setup.* introuvable (sur Jetson Foxy est sous /opt/ros/foxy/install)."
fi

# -------------------------
# 2) DDS config
# -------------------------
cat >/etc/cyclonedds.xml <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>${DDS_INTERFACE}</NetworkInterfaceAddress>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# -------------------------
# 3) Workspace & GIT
# -------------------------
mkdir -p "${WS_DIR}"
cd "${WS_DIR}"
git config --global --add safe.directory "${WS_DIR}" || true

# Si REPO_URL est SSH mais FS RO, basculer en HTTPS (pas d'écriture dans /root/.ssh)
root_writable=1
touch /root/.rwcheck 2>/dev/null || root_writable=0
rm -f /root/.rwcheck 2>/dev/null || true

if [[ "${REPO_URL}" == git@github.com:* ]] && [ "${root_writable}" -eq 0 ]; then
  # git@github.com:org/repo.git -> https://github.com/org/repo.git
  REPO_URL="https://github.com/${REPO_URL#git@github.com:}"
  log "FS root en lecture seule → bascule en HTTPS: ${REPO_URL}"
fi

# Si on reste en SSH et FS writable, préparer known_hosts (sans planter si RO)
if [[ "${REPO_URL}" == git@github.com:* ]] && [ "${root_writable}" -eq 1 ]; then
  mkdir -p /root/.ssh 2>/dev/null || true
  chmod 700 /root/.ssh 2>/dev/null || true
  if ! grep -q "github.com" /root/.ssh/known_hosts 2>/dev/null; then
    ssh-keyscan -t rsa,ecdsa -H github.com >> /root/.ssh/known_hosts 2>/dev/null || true
    chmod 600 /root/.ssh/known_hosts 2>/dev/null || true
  fi
fi

bootstrap_repo() {
  log "Bootstrap repo: URL=${REPO_URL} BRANCH=${REPO_BRANCH}"
  if [ ! -d .git ]; then
    git init
    git remote remove origin 2>/dev/null || true
    git remote add origin "${REPO_URL}"
    git fetch --prune origin "+refs/heads/*:refs/remotes/origin/*" || { err "git fetch a échoué"; return 1; }

    if git show-ref --verify --quiet "refs/remotes/origin/${REPO_BRANCH}"; then
      git checkout -B "${REPO_BRANCH}" "origin/${REPO_BRANCH}" || return 1
    else
      warn "Branche '${REPO_BRANCH}' absente côté distant → fallback 'main'"
      git checkout -B main origin/main || return 1
    fi
  else
    git remote set-url origin "${REPO_URL}" || true
    git fetch --prune origin "+refs/heads/*:refs/remotes/origin/*" || { err "git fetch a échoué"; return 1; }

    if git show-ref --verify --quiet "refs/remotes/origin/${REPO_BRANCH}"; then
      CURBRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '')"
      [ "${CURBRANCH}" != "${REPO_BRANCH}" ] && git checkout -B "${REPO_BRANCH}" "origin/${REPO_BRANCH}" || true
      git reset --hard "origin/${REPO_BRANCH}" || return 1
    else
      warn "Branche '${REPO_BRANCH}' absente → fallback 'main'"
      git checkout -B main origin/main || return 1
      git reset --hard origin/main || return 1
    fi
  fi
  return 0
}

if ! bootstrap_repo; then
  warn "Bootstrap git non terminé. Tu peux corriger à la main dans le shell."
fi

# Overlay si déjà build
if [ -f "${WS_DIR}/install/local_setup.bash" ]; then
  safe_source "${WS_DIR}/install/local_setup.bash"
elif [ -f "${WS_DIR}/install/setup.bash" ]; then
  safe_source "${WS_DIR}/install/setup.bash"
fi

# -------------------------
# 4) Exécution
# -------------------------
if [ "$#" -gt 0 ]; then
  log "Exécution de: $*"
  exec "$@"
else
  log "Shell interactif"
  exec bash -l
fi
