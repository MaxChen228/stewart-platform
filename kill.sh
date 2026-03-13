#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")"
python3 dashboard_ctl.py stop
