#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
set +u; source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"; set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$(rbnx path robonix-api):${PYTHONPATH:-}"
mkdir -p rbnx-build
exec 9>rbnx-build/bridge.lock
flock -n 9 || exit 2
"${LOCALIZATION_PYTHON:-/usr/bin/python3}" -m localization_outdoor.atlas_bridge &
pid=$!
printf '%s\n' "$pid" > rbnx-build/bridge.pid
trap 'kill -TERM "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
wait "$pid"
