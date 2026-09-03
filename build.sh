#!/bin/bash
set -e

# Detect workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If invoked from package directory or workspace root, resolve WS_ROOT
if [ -f "${SCRIPT_DIR}/../../install/setup.sh" ] || [ -f "${SCRIPT_DIR}/../../GEMINI.md" ]; then
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [ -f "${PWD}/install/setup.sh" ] || [ -f "${PWD}/GEMINI.md" ]; then
  WS_ROOT="${PWD}"
else
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

echo "==> Colcon workspace root: ${WS_ROOT}"
cd "${WS_ROOT}"

# Source virtual environment and installation setup
if [ -f "../vcs_colcon_installation/bin/activate" ]; then
  source "../vcs_colcon_installation/bin/activate"
elif [ -f "${WS_ROOT}/../vcs_colcon_installation/bin/activate" ]; then
  source "${WS_ROOT}/../vcs_colcon_installation/bin/activate"
fi

if [ -f "install/setup.sh" ]; then
  source "install/setup.sh"
fi

# Ensure git submodules are initialized and updated
PACKAGE_DIR="${WS_ROOT}/src/gz_humanoid_walking"
if [ ! -f "${PACKAGE_DIR}/vendored/pinocchio/CMakeLists.txt" ] || [ ! -f "${PACKAGE_DIR}/vendored/eiquadprog/CMakeLists.txt" ]; then
  echo "==> Initializing git submodules in ${PACKAGE_DIR}..."
  (cd "${PACKAGE_DIR}" && git submodule update --init --recursive)
fi

# 1. Build vendored pinocchio if needed
echo "==> Building vendored pinocchio..."
colcon build \
  --base-paths src/gz_humanoid_walking/vendored \
  --packages-select pinocchio \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARK=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_SDF_SUPPORT=ON \
  --merge-install \
  --symlink-install

# 2. Build gz_humanoid_walking with testing enabled
echo "==> Building gz_humanoid_walking..."
colcon build \
  --packages-select gz_humanoid_walking \
  --cmake-args \
    -DBUILD_TESTING=ON \
  --merge-install \
  --symlink-install

# Run unit tests
echo "==> Running unit tests..."
ctest --test-dir build/gz_humanoid_walking --output-on-failure

echo "==> Build and tests completed successfully."
