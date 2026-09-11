# Demo of Humanoid Walking with Gazebo Sim

A dynamic bipedal walking demo with full whole-body QP control, Linear Inverted Pendulum Model (LIPM) preview generation, and Pinocchio rigid-body dynamics running in Gazebo Sim (MuJoCo physics backend).

<img width="600" height="492" alt="output_gz_mujoco_no_lidar_forces_600" src="https://github.com/user-attachments/assets/2cb987ba-933a-453c-b1fb-6032107c76c9" />

_Demo with torso-mounted lidar sensor_

<img width="600" height="492" alt="output_gz_mujoco_with_lidar" src="https://github.com/user-attachments/assets/777dc946-ddfb-495c-8604-4c375e0544ba" />


## Workspace Prerequisites

### 1. System Dependencies
Install required Boost development libraries.

```bash
sudo apt-get update
sudo apt-get install -y libboost-filesystem-dev libboost-serialization-dev
```

### 2. Gazebo Source Installation
Follow the upstream instructions from [Gazebosim.org Source Installation](https://gazebosim.org/docs/latest/install_ubuntu_src/) to set up the colcon workspace and build with testing turned off (`-DBUILD_TESTING=OFF`).

> [!NOTE]
> The Rotary collection of branches should be imported using:
> ```bash
> vcs import --input https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-rotary.yaml src
> ```
> because only the Rotary collection currently includes the MuJoCo physics engine plugin.

### 3. Clone this Repository
Clone `gz_humanoid_walking` into the `src/` directory of your workspace:

```bash
# From workspace root:
git clone --recursive git@github.com:shameekganguly/gz_humanoid_walking.git src/gz_humanoid_walking
```

All following instructions assume the following directory structure:

```text
(workspace)/
└── src/
    ├── gz-sim/
    ├── (other gz-* packages and sdformat)
    └── gz_humanoid_walking/
```

## Quick Start Scripts

Helper scripts are provided in `src/gz_humanoid_walking/` that can be executed either from the package directory or from the colcon workspace root.

### 1. Build and Test
Automatically initializes git submodules (`pinocchio`, `eiquadprog`), compiles vendored dependencies and `gz_humanoid_walking`, and runs all unit tests:

```bash
# From workspace root:
./src/gz_humanoid_walking/build.sh

# Or from src/gz_humanoid_walking/:
./build.sh
```

### 2. Clean Workspace Artifacts
Cleans all build and install artifacts for `pinocchio` and `gz_humanoid_walking`:

```bash
# From workspace root:
./src/gz_humanoid_walking/clean.sh

# Or from src/gz_humanoid_walking/:
./clean.sh
```

## Running the Simulation

Use the `run_humanoid_walking.sh` script to automatically configure environment paths and launch the simulation. Any additional arguments passed to the script are forwarded directly to `gz sim`:

```bash
# Launch GUI simulation (from workspace root or package directory):
./src/gz_humanoid_walking/run_humanoid_walking.sh

# Or run headless simulation without GUI:
./src/gz_humanoid_walking/run_humanoid_walking.sh --headless

# Pass additional gz sim arguments (e.g. iterations or verbosity):
./src/gz_humanoid_walking/run_humanoid_walking.sh --headless --iterations 1000 -v 4
```

## Repository Structure

- `src/`: Controller, IK, dynamics wrapper, and Gazebo System plugin implementation (see [`src/README.md`](src/README.md) for detailed architecture and control formulation).
- `vendored/`: Git submodules for `eiquadprog` and `pinocchio`.
- `models/`: Modular SDFormat robot and sensor models (`jvrc1`, `camera`, `torso_lidar`, `checkered_floor`).
- `worlds/`: SDFormat 1.12 simulation world definitions.
- `test/`: Unit test suite (`test_leg_ik`, `test_lipm_generator`, `test_pinocchio_dynamics`).
- `build.sh`: Workspace detection and automated build/test runner.
- `clean.sh`: Workspace cleaning utility for vendored dependencies and package targets.
- `run_humanoid_walking.sh`: Helper runner for GUI and headless simulation execution.

## AI use

Most of this demo was created using Gemini 3.7 Flash in the Google Antigravity IDE.

## Acknowledgements

We gratefully acknowledge the authors and maintainers of [isri-aist/jvrc_mj_description](https://github.com/isri-aist/jvrc_mj_description) (at ISRI-AIST, Japan), from which the JVRC-1 humanoid model was adapted.

## Licensing

This project is licensed under the [Apache License 2.0](LICENSE).

However, it embeds the following third-party dependencies as git submodules, which are governed by their own respective licenses:
* [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) - Licensed under [LGPL-3.0](vendored/eiquadprog/COPYING.LESSER)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio) - Licensed under [BSD-2-Clause](vendored/pinocchio/LICENSE)
