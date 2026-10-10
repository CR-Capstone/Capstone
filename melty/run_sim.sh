#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$project_dir"

if [ -x "$project_dir/.venv/bin/python" ]; then
    python="$project_dir/.venv/bin/python"
elif [ -x "$project_dir/../.venv/bin/python" ]; then
    python="$project_dir/../.venv/bin/python"
else
    printf '%s\n' 'Create a virtual environment and install requirements.txt first; see README.md.' >&2
    exit 1
fi

export PYTHONPATH="$project_dir/build/python${PYTHONPATH:+:$PYTHONPATH}"
if ! "$python" -c 'import melty_sim, PySide6' >/dev/null 2>&1; then
    printf '%s\n' 'Build the Melty native module and install dependencies first; see README.md.' >&2
    exit 1
fi
exec "$python" -m sim.ui.main "$@"
