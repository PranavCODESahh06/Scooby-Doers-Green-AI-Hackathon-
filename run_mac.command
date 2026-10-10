#!/bin/zsh
set -e
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  echo "Run setup_mac.command first to install free dependencies."
  read '?Press Enter to close...'
  exit 1
fi
./.venv/bin/python AquaMind_FINAL.py
