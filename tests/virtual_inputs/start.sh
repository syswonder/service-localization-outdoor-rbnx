#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")" && pwd)}"
cd "$PKG"
set +u; source /opt/ros/humble/setup.bash; set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$(rbnx path robonix-api):${PYTHONPATH:-}"
exec /tmp/stack-venv/bin/python fixture.py
