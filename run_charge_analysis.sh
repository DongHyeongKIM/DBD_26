#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$project_dir"

if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi

if ! .venv/bin/python -c "import numpy, pandas, matplotlib" 2>/dev/null; then
  .venv/bin/python -m pip install -r requirements.txt
fi

mkdir -p .cache/matplotlib .cache/pycache
export MPLCONFIGDIR="$project_dir/.cache/matplotlib"
export PYTHONPYCACHEPREFIX="$project_dir/.cache/pycache"

exec .venv/bin/python src/charge_log_analysis.py \
  --dataset Dataset \
  --result result/charging \
  "$@"
