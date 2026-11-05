#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
python3 - "$PKG" <<'PYCODE'
from pathlib import Path
import os,signal,sys
root=Path(sys.argv[1]).resolve();f=root/'rbnx-build/bridge.pid'
if not f.exists():raise SystemExit(0)
pid=int(f.read_text());proc=Path('/proc')/str(pid)
try:
    if b'localization_outdoor.atlas_bridge' not in (proc/'cmdline').read_bytes() or (proc/'cwd').resolve()!=root:
        raise SystemExit('PID does not belong to this localization instance')
    os.kill(pid,signal.SIGTERM)
except FileNotFoundError:pass
PYCODE
