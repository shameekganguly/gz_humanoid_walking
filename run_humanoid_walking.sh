#!/bin/bash
set -e

# Detect workspace and package root directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "${SCRIPT_DIR}/../../install/setup.sh" ]; then
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [ -f "${PWD}/install/setup.sh" ]; then
  WS_ROOT="${PWD}"
else
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

PACKAGE_DIR="${WS_ROOT}/src/gz_humanoid_walking"

# Source virtual environment and installation setup
if [ -f "${WS_ROOT}/../vcs_colcon_installation/bin/activate" ]; then
  source "${WS_ROOT}/../vcs_colcon_installation/bin/activate"
fi

if [ -f "${WS_ROOT}/install/setup.sh" ]; then
  source "${WS_ROOT}/install/setup.sh"
fi

# Export resource and plugin search paths
export GZ_SIM_RESOURCE_PATH="${PACKAGE_DIR}/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${WS_ROOT}/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

WORLD_FILE="${PACKAGE_DIR}/worlds/humanoid_walking_mujoco.sdf"

# Parse arguments
HEADLESS=false
EXTRA_ARGS=()

for arg in "$@"; do
  case "$arg" in
    --headless)
      HEADLESS=true
      ;;
    -h|--help)
      echo "Usage: $0 [options] [gz sim args...]"
      echo ""
      echo "Options:"
      echo "  --headless    Run Gazebo simulation headless without GUI"
      echo "  -h, --help    Show this help message"
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

if [ "$HEADLESS" = true ]; then
  echo "==> Launching headless simulation: ${WORLD_FILE}"
  gz sim -s --headless-rendering "${WORLD_FILE}" "${EXTRA_ARGS[@]}"
else
  echo "==> Launching GUI simulation: ${WORLD_FILE}"
  gz sim "${WORLD_FILE}" "${EXTRA_ARGS[@]}"
fi
