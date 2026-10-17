#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ -x "$ROOT/.venv/bin/python" ]]; then
    PY="$ROOT/.venv/bin/python"
elif [[ -x "$ROOT/../.venv/bin/python" ]]; then
    PY="$ROOT/../.venv/bin/python"
else
    PY="python3"
fi
if ! "$PY" -c 'import serial, PySide6' 2>/dev/null; then
    echo "Missing dependencies. In $ROOT, create .venv and install requirements.txt (see README.md)." >&2
    exit 1
fi
exec "$PY" -m controls.app "$@"
