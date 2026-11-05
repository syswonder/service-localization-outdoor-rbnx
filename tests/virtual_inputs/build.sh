#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")" && pwd)}"
rbnx codegen -p "$PKG" --python /tmp/stack-venv/bin/python
mkdir -p "$PKG/rbnx-build"
touch "$PKG/rbnx-build/.rbnx-built"
