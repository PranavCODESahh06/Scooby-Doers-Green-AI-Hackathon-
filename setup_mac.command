#!/bin/zsh
set -e
cd "$(dirname "$0")"
if command -v python3.14 >/dev/null 2>&1; then
    PY=python3.14
elif command -v python3 >/dev/null 2>&1; then
    PY=python3
else
    echo "Python 3 not found. Install Python and try again."
    read '?Press Enter to close...'
    exit 1
fi
"$PY" -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
printf '\nSETUP COMPLETE!\nOpen this whole folder in VS Code, then press F5 and choose AquaMind — Run 3D Simulation.\n'
printf 'To run from Terminal: ./.venv/bin/python AquaMind_FINAL.py\n'
read '?Press Enter to close...'
