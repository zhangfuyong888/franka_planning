#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
lab_env="${VENV_DIR:-.venv}"
if [[ -e "$lab_env" ]]; then echo "环境已存在：$lab_env。请选择其他 VENV_DIR；不会覆盖。" >&2; exit 2; fi
"${PYTHON:-python3}" -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,13), "需要Python3.10-3.12"'
"${PYTHON:-python3}" -m venv "$lab_env"
"$lab_env/bin/python" -m pip install -r requirements/locked-runtime.txt
"$lab_env/bin/python" -m pip install --no-build-isolation --no-deps -e .
"$lab_env/bin/python" -m pip check
"$lab_env/bin/python" -m pytest -q
