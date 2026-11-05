#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
rbnx validate "$PKG"
rbnx codegen -p "$PKG" --python "${LOCALIZATION_PYTHON:-/usr/bin/python3}"
mkdir -p "$PKG/rbnx-build"
touch "$PKG/rbnx-build/.rbnx-built"
