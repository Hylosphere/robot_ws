#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -d "$WS/scripts" ]; then
  echo "[perms] Setting +x on all scripts in $WS/scripts"
  # Make common script types executable
  find "$WS/scripts" -type f \( -name "*.sh" -o -name "*.bash" -o -name "*.py" \) -exec chmod +x {} +
  # Also make known entry points executable if they exist (no-op if they don't)
  chmod +x "$WS/scripts/build" 2>/dev/null || true
  chmod +x "$WS/scripts/clean" 2>/dev/null || true
  chmod +x "$WS/scripts/deploy" 2>/dev/null || true
fi

echo "[perms] Done."
