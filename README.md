> [!NOTE]
> ## You are on the `ros2_dev` branch — ROS2 support for kr_autonomous_flight.
>
> This branch ports the entire `kr_autonomous_flight` stack from **ROS1 Noetic** (Ubuntu 20.04) to **ROS2 Jazzy Jalisco** (Ubuntu 24.04) — the latest ROS2 LTS. It is built on top of the [`feature/integrate_lidar_3d_planner_default`](https://github.com/KumarRobotics/kr_autonomous_flight/tree/feature/integrate_lidar_3d_planner_default) branch, which adds support for the **3D planner** and both **LIDAR + VIO** and **LIDAR-only** autonomous flight configurations. It is provided to help developers who want to use kr_autonomous_flight with ROS2.
>
> **Note:** this ROS2 port has **not been as extensively tested or experimented with** as the ROS1 version. If you want the version used to produce the results in our papers, or the most battle-tested setup, please use the [`master`](https://github.com/KumarRobotics/kr_autonomous_flight/tree/master) branch (ROS1 Noetic, Ubuntu 20.04), or the [`feature/integrate_lidar_3d_planner_default`](https://github.com/KumarRobotics/kr_autonomous_flight/tree/feature/integrate_lidar_3d_planner_default) branch if you specifically need 3D planning or LIDAR-only flight under ROS1.
>
> Issues and pull requests that improve the ROS2 port are very welcome.

![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/falcon4-compressed.jpg)

This is the autonomous flight code stack used at KumarRobotics, providing a complete solution for GPS-denied quadcopter autonomy. It has been tested extensively in challenging urban and rural (under forest canopy) environments.

![Docker Build Base](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-base.yaml/badge.svg)
![Docker Build Client](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-client.yaml/badge.svg)
![Docker Build Control](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-control.yaml/badge.svg)
![Docker Build Estimation](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-estimation.yaml/badge.svg)
![Docker Build Map_plan](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-map-plan.yaml/badge.svg)
![Docker Build State_machine](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-state-machine.yaml/badge.svg)
![Docker Build Sim](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-sim.yaml/badge.svg)

> **Documentation status.** Everything you need to install, build, run the simulation, and run the LIDAR autonomy stack on the `ros2_dev` branch is contained in this README. The upstream [Wiki](https://github.com/KumarRobotics/kr_autonomous_flight/wiki) is frozen at the ROS1 Noetic workflow and will **not** be updated for the ROS2 port. The `master` branch still points at the wiki because the ROS1 instructions there remain accurate for ROS1 users.

## Table of contents

- [System requirements](#system-requirements)
- [Install ROS2 Jazzy](#install-ros2-jazzy)
- [Build from source](#build-from-source)
- [Use docker (recommended)](#use-docker-recommended)
- [Converting ROS1 bags to ROS2](#converting-ros1-bags-to-ros2)
- [Run Gazebo simulation](#run-gazebo-simulation)
- [Run with SLAM (LIDAR + VIO)](#run-with-slam-lidar--vio)
- [Run LIDAR-only autonomous flight](#run-lidar-only-autonomous-flight)
- [Run full coverage experiments](#run-full-coverage-experiments)
- [Real-robot deployment](#real-robot-deployment)
- [Troubleshooting](#troubleshooting)
- [High-level code structure](#high-level-code-structure)
- [Videos](#videos)
- [Contributing](#contributing)
- [Citation](#citation)
- [Acknowledgement](#acknowledgement)
- [License](#license)

## System requirements

- **OS:** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS:** ROS2 Jazzy Jalisco (the current ROS2 LTS, EOL May 2029)
- **Python:** Python 3.12 (ships with Ubuntu 24.04 and is the Jazzy default)
- **Compiler:** GCC 13+ (default on 24.04)
- **CPU:** any modern x86_64 workstation. 8+ cores recommended so that `colcon build` completes in reasonable time.
- **RAM:** 16 GB minimum, 32 GB recommended for simulation + RViz2.
- **GPU:** not required for the core autonomy stack. A CUDA-capable NVIDIA GPU is only needed if you plan to run learned VIO / learned SLAM modules or to run the Gazebo-based full simulation with heavy rendering. If you use docker images that depend on CUDA, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) first.
- **Gazebo:** ROS2 Jazzy ships with **Gazebo Harmonic** (formerly Ignition Gazebo) as the default simulator. The legacy "Gazebo Classic" used by the ROS1 wiki is **no longer supported on Jazzy**. See [Run Gazebo simulation](#run-gazebo-simulation) below for the migration notes.

## Install ROS2 Jazzy

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html). The short version is:

```bash
# Enable the ROS2 apt repository (see the official guide for the GPG key setup).
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Source the setup script in every new shell (or add it to your ~/.bashrc).
source /opt/ros/jazzy/setup.bash
```

You should also install the tools used by `colcon`, `rosdep`, and the workspace management flow:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-pip git build-essential cmake
sudo rosdep init  # only the first time
rosdep update
```

## Build from source

```bash
# 1. Create a ROS2 workspace.
mkdir -p ~/kr_ws/src
cd ~/kr_ws/src

# 2. Clone this repository on the ros2_dev branch.
git clone --branch ros2_dev git@github.com:KumarRobotics/kr_autonomous_flight.git
# (Use https://github.com/KumarRobotics/kr_autonomous_flight.git if you do not
#  have SSH keys configured with GitHub.)

# 3. Pull third-party dependencies declared in external_all.yaml.
#    NOTE: external_all.yaml currently points at ROS1-only upstream packages.
#    The TODO at the top of that file tracks the ongoing port of each
#    dependency to its ROS2 equivalent. Expect `vcs import` to succeed but
#    `colcon build` to fail on some of those packages until they have been
#    ported. See Troubleshooting below.
cd kr_autonomous_flight
vcs import < external_all.yaml
vcs import < motion_primitives/deps_ssh.repos   # motion primitives dispersion deps

# 4. Install system dependencies via rosdep.
cd ~/kr_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Install libspdlog, which is required by the mapper / planner and is
#    not always resolved by rosdep on a fresh 24.04 install.
sudo apt install -y libspdlog-dev

# 6. Build the workspace with colcon. --symlink-install lets you edit Python
#    / launch / config files without rebuilding.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 7. Source the overlay. You must do this in every new shell, OR add it to
#    ~/.bashrc so it is picked up automatically.
source install/setup.bash
```

*(The ROS1 wiki `Building-from-Source` page prescribes `catkin build` against `ros-noetic-*` packages on Ubuntu 20.04; the command list above is the ROS2 Jazzy equivalent adapted for colcon and the `ros-jazzy-*` apt namespace. Not yet verified end-to-end on a clean ROS2 Jazzy workspace — see Troubleshooting.)*

## Use docker (recommended)

Using docker sidesteps most system-dependency headaches because each component is built inside a pinned Ubuntu 24.04 / ROS2 Jazzy image. Each subdirectory under `autonomy_core/` and `autonomy_sim/` ships its own `docker/` folder with `build.sh`, `run.sh`, and `entrypoint.sh` scripts.

### Prerequisites

```bash
# Install docker engine per https://docs.docker.com/engine/install/ubuntu/
# and put your user in the docker group:
sudo usermod -aG docker $USER
newgrp docker

# If you plan to use GPU-backed images (SLAM / VIO inference, rendering):
# install the NVIDIA Container Toolkit and verify it:
sudo docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu24.04 nvidia-smi
```

### Pull prebuilt images

Once published, the ROS2 Jazzy images will be tagged `kumarrobotics/autonomy:<component>-jazzy`. *(Placeholder — the `-jazzy` image tags are not yet published to Docker Hub; the CI badges at the top of this README build these images from the `ros2_dev` branch.)*

```bash
docker pull kumarrobotics/autonomy:base-jazzy
docker pull kumarrobotics/autonomy:client-jazzy
docker pull kumarrobotics/autonomy:control-jazzy
docker pull kumarrobotics/autonomy:estimation-jazzy
docker pull kumarrobotics/autonomy:map_plan-jazzy
docker pull kumarrobotics/autonomy:state_machine-jazzy
docker pull kumarrobotics/autonomy:sim-jazzy
```

### Build the images locally

Run these from the repository root. The `-jazzy` tag suffix is the convention used by the ROS2 CI workflows on this branch.

```bash
# Core base image — everything else FROMs this.
IMG=base;          docker build -t "kumarrobotics/autonomy:${IMG}-jazzy" -f "autonomy_core/${IMG}/Dockerfile" .

# Per-component images.
for IMG in client control estimation map_plan state_machine; do
    docker build -t "kumarrobotics/autonomy:${IMG}-jazzy" -f "autonomy_core/${IMG}/Dockerfile" .
done

# Gazebo simulation image.
IMG=sim;           docker build -t "kumarrobotics/autonomy:${IMG}-jazzy" -f "autonomy_sim/Dockerfile" .
```

### Run a container

Each component folder has a `run.sh` that wires up the correct mounts (source tree, X11 socket, GPU flags) and drops you into an interactive shell in the container:

```bash
# Example: drop into the base container.
./autonomy_core/base/docker/run.sh

# Example: drop into the simulation container.
./autonomy_sim/docker/run.sh
```

Inside the container, the repo is mounted as a ROS2 workspace, so the standard build / source flow applies:

```bash
# Inside the container:
source /opt/ros/jazzy/setup.bash
cd /root/kr_ws        # or whatever path run.sh maps your source tree to
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

*(Script names and the container mount path are as they exist on the `ros2_dev` branch; the exact tags follow the ROS1 wiki "Using Docker" convention but with a `-jazzy` suffix. Not yet verified against published images.)*

## Converting ROS1 bags to ROS2

Every demo rosbag that ships on the ROS1 `master` branch was recorded under Noetic and is stored in the legacy `.bag` format. `ros2 bag play` cannot read that format directly — you must first convert each bag to the ROS2 SQLite (`.db3` / mcap) format using the [`rosbags`](https://pypi.org/project/rosbags/) pip package:

```bash
pip install rosbags
rosbags-convert --src input.bag --dst output_bag_dir/
ros2 bag play output_bag_dir
```

`rosbags-convert` creates `output_bag_dir/` containing a `metadata.yaml` and a `.db3` file that `ros2 bag play` understands natively.

## Run Gazebo simulation

> **Gazebo version.** ROS2 Jazzy targets **Gazebo Harmonic** via the `ros_gz_*` bindings. The ROS1 wiki `Gazebo-Simulation-Setup` page is written against **Gazebo Classic 11**, which is not available on Ubuntu 24.04. The launch files under `autonomy_sim/gazebo_sim/` are being ported to Gazebo Harmonic; expect world and model file names to stay the same, but spawn / bridge commands to change. Install the bridge packages before running the simulation:
>
> ```bash
> sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
> ```

### Built-from-source flow

```bash
source ~/kr_ws/install/setup.bash
ros2 launch gazebo_utils full_sim.launch.py
```

### Docker flow

```bash
cd ~/kr_ws/src/kr_autonomous_flight
./autonomy_sim/docker/run.sh
# Inside the container:
ros2 launch gazebo_utils full_sim.launch.py
```

*(Translated from `roslaunch gazebo_utils full_sim.launch` on the ROS1 wiki. The launch file is present on `ros2_dev` at `autonomy_sim/gazebo_sim/gazebo_utils/launch/full_sim.launch.py`.)*

### Key configuration files you may want to edit

- `autonomy_core/control/control_launch/config/tracker_params_mp.yaml` — motion-primitive tracker parameters.
- `autonomy_core/map_plan/map_plan_launch/config/mapper.yaml` — voxel mapper parameters.
- `autonomy_sim/gazebo_sim/gazebo_utils/launch/full_sim.launch.py` — top-level simulation launch; exposes `x`, `y`, `z` (spawn pose) and `world_name` as launch arguments.
- `autonomy_sim/gazebo_sim/gazebo_utils/launch/simulation.launch.py` — lower-level simulation orchestration.
- `autonomy_sim/gazebo_sim/gazebo_utils/config/falcon4_os1_so3_gains.yaml` — controller gains for the default Falcon4 + OS1 platform.

Change the spawn pose and world via `key:=value` launch arguments, the same as `roslaunch`:

```bash
ros2 launch gazebo_utils full_sim.launch.py x:=-10.0 y:=-10.0 z:=1.0
ros2 launch gazebo_utils full_sim.launch.py world_name:=/absolute/path/to/your.sdf
```

*(Gazebo Harmonic prefers SDF worlds over the legacy `.world` XML format that Gazebo Classic used; the ROS1 wiki examples of editing `<arg name="world_name" value="PATH_TO_YOUR_WORLD_FILE/YOUR_FILE_NAME.world"/>` should be updated to point at an SDF file. Not yet verified.)*

### Flying a waypoint mission

Waypoint files still use the same YAML format:

```yaml
- position: [40.03, -70.81, 5.00]
- position: [-86.49, -68.23, 5.00]
```

Load them via the client GUI (`ros2 launch client_launch client.launch.py`), which publishes waypoints to the state machine.

## Run with SLAM (LIDAR + VIO)

The ROS1 wiki `Running-with-SLAM` page is architectural — it documents that **any SLAM module with 6 DOF pose output** can be plugged in, as long as it publishes the transform from the robot's odometry frame to the map frame on `/<robot_ns>/odom` → `/<robot_ns>/map`. The local-global replanner in this repository listens for that transform to correct drift in the planning frame.

The default SLAM module at KumarRobotics is [SLOAM](https://github.com/KumarRobotics/SLOAM) (semantic LIDAR odometry and mapping), with VIO coming from MSCKF-VIO.

### Bring-up sequence

Run each of the following in its own terminal, sourcing `~/kr_ws/install/setup.bash` first. The ordering below mirrors what the ROS1 launch files did under the hood.

```bash
# Terminal 1: drivers (Ouster LIDAR, OVC3 stereo+IMU, pixhawk bridge).
ros2 launch real_experiment_launch full_sensors_faster_lio.launch.py

# Terminal 2: VIO (MSCKF) — only needed for the LIDAR + VIO configuration.
ros2 launch estimation_launch msckf_vio.launch.py
ros2 launch estimation_launch estimation.launch.py

# Terminal 3: SLAM (SLOAM or SLIDE-SLAM).
# The ROS1 master branch pulls sloam_msgs + sloam via external_all.yaml. Those
# packages must be ported to ROS2 first. The xurobotics fork of
# https://github.com/XuRobotics/SLIDE_SLAM already has a ros2_dev branch with
# a working ROS2 port of sloam_msgs; use that as the upstream until this
# repo's external YAML points at it.

# Terminal 4: mapper + planner + state machine (the autonomy core).
ros2 launch state_machine_launch system_mp.launch.py

# Terminal 5: client GUI to command the robot.
ros2 launch client_launch client.launch.py
```

Once everything is up, use the client GUI to arm, takeoff, publish waypoints, and monitor state transitions. RViz2 is launched from inside `system_mp.launch.py`; if you need a standalone visualization, run:

```bash
rviz2 -d autonomy_core/map_plan/map_plan_launch/rviz/mapper.rviz
```

*(The ROS1 `Running-with-SLAM` wiki page does not contain a step-by-step command list — the sequence above is reconstructed from the launch files present on this branch. Not yet verified end-to-end on a ROS2 Jazzy workspace because several dependencies — `sloam_msgs`, MSCKF-VIO — are still ROS1-only.)*

### Frame conventions

- `<robot_ns>/odom` — drift-prone, smooth odometry frame (output of VIO or LIDAR-inertial odometry).
- `<robot_ns>/map` — drift-corrected, possibly jumpy SLAM frame.
- The local-global replanner reads the `odom` → `map` transform continuously and re-anchors the global plan whenever the SLAM module publishes a loop closure.

## Run LIDAR-only autonomous flight

The `ros2_dev` branch (mirroring `feature/integrate_lidar_3d_planner_default`) supports running the full autonomy stack **without** a VIO pipeline — only the LIDAR-inertial odometry from [Faster-LIO](https://github.com/gaoxiang12/faster-lio) and an external IMU are required. This is the recommended configuration for forest and urban canopy environments where stereo VIO degrades.

### Build with the LIDAR-only dependency set

If you only need the LIDAR-only flow, you can skip the full external repo list and pull just the LIDAR odometry packages:

```bash
cd ~/kr_ws/src/kr_autonomous_flight
vcs import < external_lidar_odometry.yaml
cd ~/kr_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

*(Translated from `vcs import < external_lidar_odometry.yaml` on the ROS1 wiki, which used `catkin build` instead of colcon.)*

### Launch sequence

```bash
# Terminal 1: sensor drivers tuned for the LIDAR-only flow.
ros2 launch real_experiment_launch drivers_for_faster_lio.launch.py

# Terminal 2: Faster-LIO configured to publish odometry on the topics the
# autonomy stack expects.
ros2 launch real_experiment_launch faster_lio_for_autonomy.launch.py

# Terminal 3: full autonomy (mapper, planner, state machine, trackers).
ros2 launch real_experiment_launch full_autonomy.launch.py

# Terminal 4: client GUI.
ros2 launch client_launch client.launch.py
```

The launch files above are present on this branch under `autonomy_real/real_experiment_launch/launch/`. *(Port of `faster-lio` itself to ROS2 is an outstanding dependency; the `ros2_dev` branch ships the launch files but Faster-LIO must be built from an upstream ROS2 fork until the external YAML is updated.)*

## Run full coverage experiments

For the coverage-planning experiments documented in the [IEEE RAL 2022 paper](https://ieeexplore.ieee.org/document/9720968), pull the additional coverage-planner dependencies and rebuild:

```bash
cd ~/kr_ws/src/kr_autonomous_flight
vcs import < external_coverage_planner.yaml
cd ~/kr_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Then run the coverage utilities launch on top of the normal simulation or real-robot bring-up:

```bash
ros2 launch coverage_utils coverage_utils.launch.py
```

The coverage planner publishes waypoints directly onto the topic the autonomy stack listens on — no manual waypoint file is required. Follow the instructions in the upstream `polygon_coverage_planning` repo's `README_run_with_autonomy_stack.txt` for per-experiment parameter tuning. *(Translated from the wiki `Full-coverage-experiments` page, which uses `catkin build`; the polygon_coverage_planning upstream is still ROS1-only and must be ported before this flow runs under Jazzy.)*

## Real-robot deployment

Running on real hardware requires a non-trivial set of per-vehicle calibrations and bring-up steps. At a minimum you will need:

- **Flight controller:** Pixhawk 4 (FMUv5) flashed with PX4, connected to the companion computer over serial/USB via MAVROS.
- **Companion computer:** an Intel NUC (10th gen or newer) or equivalent x86_64 board running Ubuntu 24.04 + ROS2 Jazzy.
- **LIDAR:** Ouster OS1 (32/64/128) or Velodyne Puck.
- **External IMU:** VectorNav VN-100 (attached to OVC3) is recommended; the internal Pixhawk IMU is **not** sufficient for the SLAM stack.
- **Stereo camera + IMU:** Open Vision Computer V3 (OVC3) is the reference stereo+IMU head. ZED and Intel RealSense are possible alternatives.
- **Airframe:** the Falcon4 custom frame used in the papers; Tarot 650 and DJI F450 have also been used.

Detailed hardware setup — Pixhawk flashing, MAVROS configuration, Ouster PTP setup, OVC3 stereo + IMU calibration, preflight procedure, new-robot bring-up, WiFi network setup — is covered in the ROS1 wiki, **which has not yet been ported to ROS2**. The following wiki pages remain the authoritative references **for the ROS1 Noetic workflow**:

- [Hardware Requirements](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Hardware-Requirements) *(ROS1 only — port pending)*
- [Pixhawk with Companion Computer](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Pixhawk-with-Companion-Computer) *(ROS1 only — port pending)*
- [Ouster Setup](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Ouster-Setup) and [New Robot: Ouster OS 1 Setup PTP](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Ouster-OS-1---Setup---PTP) *(ROS1 only — port pending)*
- [Stereo IMU OVC3](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Stereo-IMU-OVC3) and [New Robot: Stereo IMU](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Stereo-IMU) *(ROS1 only — port pending)*
- [Calibration](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/Calibration) *(ROS1 only — port pending)*
- [New Robot: Install and setup](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Install-and-setup) and [New Robot: Launch experiment](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Launch-experiment) *(ROS1 only — port pending)*
- [New Robot: Preflight check](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Preflight-check) *(ROS1 only — port pending)*
- [WiFi Network](https://github.com/KumarRobotics/kr_autonomous_flight/wiki/WiFi-Network) *(ROS1 only — port pending)*

The network / driver / calibration steps are largely framework-independent, so the wiki is still useful as a hardware playbook — just remember to translate any `roslaunch` / `rostopic` / `ros-noetic-*` references to their ROS2 equivalents and expect driver package names to change (`ros-jazzy-mavros`, etc.).

## Troubleshooting

- **`colcon build` fails on a package that depends on `catkin`.** One of the external dependencies is still ROS1-only; see the TODO banner in `external_all.yaml`. Skip that package with `colcon build --packages-skip <pkg>` or `--packages-up-to <target>` while the port is in progress.
- **`rosdep install` cannot resolve a key.** Make sure `rosdep update` has been run at least once on this machine and that `/etc/ros/rosdep/sources.list.d/` references the Jazzy sources. If the key truly does not exist yet, install the apt package by hand and move on.
- **`ros2 launch <pkg> <file>.launch.py` says "package not found".** You forgot to `source ~/kr_ws/install/setup.bash` in that terminal, or the package failed to build and is not in `install/`. Check `colcon build --event-handlers console_direct+` for the real error.
- **`ros2 launch` says "launch file not found" even though the package is installed.** The launch file must live under `share/<pkg>/launch/` in the install tree. If you converted from a ROS1 `.launch` file, double-check the file is named `*.launch.py` (or `*.launch.xml`) and is listed under `install_directories` in the package's `CMakeLists.txt`.
- **Gazebo Harmonic windows do not open under docker.** You need `--gpus all` (for NVIDIA) and the X11 socket mounted. The `docker/run.sh` scripts in this repo already do both, but you may also need to `xhost +local:docker` on the host.
- **`ros2 bag play` refuses a `.bag` file.** That bag is ROS1 format. See [Converting ROS1 bags to ROS2](#converting-ros1-bags-to-ros2).
- **The client GUI complains about `rqt_quadrotor_safety`.** The client packages require `ros-jazzy-rqt` and `ros-jazzy-python-qt-binding`; make sure both are installed.
- **`ros2 launch gazebo_utils full_sim.launch.py` cannot find `gazebo_ros`.** The ROS1 `gazebo_ros` bridge is not available on Jazzy. Install `ros-jazzy-ros-gz` + `ros-jazzy-ros-gz-sim` (Gazebo Harmonic bridge) instead.
- **SLOAM / sloam_msgs will not build.** Those packages are still ROS1-only on the upstream. Use the `ros2_dev` branch of [XuRobotics/SLIDE_SLAM](https://github.com/XuRobotics/SLIDE_SLAM) which has a working ROS2 port of `sloam_msgs` and swap it in via `src/`.

## High-level code structure

![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/autonomy_stack_pipeline.png)

## Videos

[Real-world experiments in large-scale autonomous flight with real-time semantic SLAM in forests](https://www.youtube.com/watch?v=Ad3ANMX8gd4)

[Real-world experiments in fast, autonomous, GPS-Denied quadrotor flight](https://m.youtube.com/watch?v=6eeetSVHXPk)

[Simulation experiments in fast, autonomous flight in urban and rural environments](https://www.youtube.com/watch?v=l1esgtJ4C6s)

*(The footage above was captured under the ROS1 Noetic version of the stack but still accurately illustrates the system's capabilities — the ROS2 port does not change the algorithms, only their middleware layer.)*

## Contributing

Report issues: Open an [issue](https://github.com/KumarRobotics/kr_autonomous_flight/issues) on GitHub.

Merge code changes: Open a [pull request](https://github.com/KumarRobotics/kr_autonomous_flight/pulls) on GitHub. Pull requests against the `ros2_dev` branch that port more of the external dependencies or verify one of the launch sequences above end-to-end are particularly welcome.

## Citation

If you use our code in your work, please cite:

```bibtex
@article{mohta2018fast,
  title={Fast, autonomous flight in GPS-denied and cluttered environments},
  author={Mohta, Kartik and Watterson, Michael and Mulgaonkar, Yash and Liu, Sikang and Qu, Chao and Makineni, Anurag and Saulnier, Kelsey and Sun, Ke and Zhu, Alex and Delmerico, Jeffrey and Thakur, Dinesh and Karydis, Konstantinos and Atanasov, Nikolay and Loianno, Giuseppe and Scaramuzza, Davide and Daniilidis, Kostas and Taylor, Camillo Jose and Kumar, Vijay},
  journal={Journal of Field Robotics},
  volume={35},
  number={1},
  pages={101--120},
  year={2018}
}
```

```bibtex
@article{mohta2018experiments,
  title={Experiments in fast, autonomous, gps-denied quadrotor flight},
  author={Mohta, Kartik and Sun, Ke and Liu, Sikang and Watterson, Michael and Pfrommer, Bernd and Svacha, James and Mulgaonkar, Yash and Taylor, Camillo Jose and Kumar, Vijay},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7832--7839},
  year={2018},
  organization={IEEE}
}
```

```bibtex
@article{liu2022large,
  title={Large-Scale Autonomous Flight With Real-Time Semantic SLAM Under Dense Forest Canopy},
  author={Liu, Xu and Nardari, Guilherme V. and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W. and Romero, Roseli A. F. and Taylor, Camillo J. and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters},
  year={2022},
  volume={7},
  number={2},
  pages={5512-5519},
}
```

## Acknowledgement

This is a multi-year project. We gratefully acknowledge contributions from our current members and lab alumni. The contributors can be found in the papers listed above. In addition, [Laura Jarin-Lipschitz](https://github.com/ljarin) and [Justin Thomas](https://github.com/justinthomas) also contributed to this work. We also thank our [funding agencies](https://www.kumarrobotics.org/research/).

## License

This code is released under the Penn Software License. Please refer to [`LICENSE.txt`](LICENSE.txt) for details.
