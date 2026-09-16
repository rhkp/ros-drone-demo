#!/usr/bin/env bash
set -euo pipefail

# Evaluate a trained detector on a reserved validation/test split.
# Override DATASET_DIR, MODEL_PATH, REPORT_PATH, and EVAL_SPLIT as needed.
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"
script="${repo_dir}/helm/drone-ml-pipeline/verification/evaluate_detector.py"

exec "${python_bin}" "${script}" "$@"
