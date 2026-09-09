#!/bin/bash
set -e

# Detect workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If invoked from package directory or workspace root, resolve WS_ROOT
if [ -f "${SCRIPT_DIR}/../../install/setup.sh" ]; then
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [ -f "${PWD}/install/setup.sh" ]; then
  WS_ROOT="${PWD}"
else
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

echo "==> Colcon workspace root: ${WS_ROOT}"
cd "${WS_ROOT}"

echo "==> Cleaning build artifacts for pinocchio and gz_humanoid_walking..."
rm -rf build/pinocchio build/gz_humanoid_walking build/eiquadprog

echo "==> Cleaning install artifacts for pinocchio and gz_humanoid_walking..."
rm -rf install/lib/*pinocchio* install/lib/cmake/pinocchio install/include/pinocchio install/share/pinocchio
rm -rf install/lib/*gz_humanoid_walking* install/include/gz_humanoid_walking install/share/gz_humanoid_walking

echo "==> Clean completed successfully."
