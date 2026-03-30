#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
TARGET="dual_capture_app"

usage() {
    cat <<EOF
Usage: $(basename "$0") [app args...]

Examples:
  $(basename "$0")
  $(basename "$0") --phoenix-index 0 --helios-index 0
  $(basename "$0") --output-dir calibration/data/session_01

This script configures/builds the dual capture app if needed, then runs it.
Press SPACE in the app window to save the current RGB + intensity pair.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target "${TARGET}" -j4

exec "${BUILD_DIR}/apps/${TARGET}" "$@"
