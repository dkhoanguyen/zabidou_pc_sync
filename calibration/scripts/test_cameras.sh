#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

usage() {
    cat <<EOF
Usage: $(basename "$0") <phoenix|helios|dual> [app args...]

Examples:
  $(basename "$0") phoenix
  $(basename "$0") helios --helios-format Coord3D_ABCY16s
  $(basename "$0") dual --phoenix-index 0 --helios-index 0

This script configures/builds the requested preview app if needed, then runs it.
EOF
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

MODE="$1"
shift

case "${MODE}" in
    phoenix)
        TARGET="camera_test"
        ;;
    helios)
        TARGET="helios_test"
        ;;
    dual)
        TARGET="dual_camera_test"
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown mode: ${MODE}" >&2
        usage
        exit 1
        ;;
esac

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target "${TARGET}" -j4

exec "${BUILD_DIR}/apps/${TARGET}" "$@"
