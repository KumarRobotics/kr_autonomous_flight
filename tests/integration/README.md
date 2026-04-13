# `tests/integration/` - runtime checks for `kr_autonomous_flight` (`ros2_dev`)

This directory holds **runtime** tests for the ROS2 Jazzy port. Unlike the
static suite under [`tests/static/`](../static/check_ros2_port.sh), tests here
exercise actual ROS2 tooling and may start processes.

## `launch_smoke_test.sh`

A bash driver that runs `ros2 launch --print-description <abs_path>` against
every `*.launch.py` file under `autonomy_core/`, `autonomy_real/`, and
`autonomy_sim/` with a per-file timeout (`LAUNCH_TIMEOUT`, default `20s`).
`--print-description` resolves the `LaunchDescription` without starting any
nodes, so it works against an unbuilt workspace - only `ros2` itself needs to
be on `PATH`. Each file is reported as `[OK]`, `[FAIL]`, or `[TIMEOUT]`, and
the script prints a trailing summary of the form `N launches, P OK, F failed,
T timeout`. Run it with:

```bash
source /opt/ros/jazzy/setup.bash
tests/integration/launch_smoke_test.sh                 # default: repo root = two dirs up
LAUNCH_TIMEOUT=30 VERBOSE=1 tests/integration/launch_smoke_test.sh
```

Exit codes follow the autotools convention: `0` all-pass, `1` any failure or
timeout, `77` `ros2`/`timeout` missing (skipped). `NO_COLOR=1` disables ANSI
colors.

### Prerequisites

- `source /opt/ros/jazzy/setup.bash`
- `ros2` on `PATH`
- GNU `timeout` (coreutils)

Nothing else: no built workspace, no Python deps.

## Not yet implemented - contributions welcome

The following runtime checks all require a **built** Jazzy workspace and a
real system toolchain, so they are not yet wired up. Contributors should add
them here as they become feasible:

- **`colcon build` smoke test**: run `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` from a clean workspace and assert success for each of the 22 packages that have buildable C++/Python content.
- **Action server round-trip**: spin up `action_planner` and send a `kr_planning_msgs::action::PlanTwoPoint` goal; assert the server accepts, runs, and returns a result within N seconds.
- **state_machine replan client**: similar round-trip for `state_machine_core::action::Replan`.
- **Tracker plugin load test**: instantiate the `action_trackers::LandTracker`, `TakeOffTracker`, `StoppingPolicy`, and `ActionTrajectoryTracker` plugins via pluginlib and verify they construct without throwing.
- **fla_ukf component load test**: load the `fla_ukf::FLAUKFNodelet` component into a ComponentContainer and verify the node comes up without throwing.
- **mavros_interface component load test**: same for `mavros_interface::SO3CmdToMavros`.
- **TF chain smoke test**: publish a dummy TF tree `map -> odom -> base_link` and assert the `tf2_ros::Buffer` inside `mapper::TFListener` resolves a `base_link` query.
- **mapper VoxelMap round-trip**: publish a fake `sensor_msgs/PointCloud2`, feed it into the local-global mapper, assert it produces a `kr_planning_msgs/VoxelMap` output.
- **Gazebo Harmonic bring-up test**: `ros2 launch gazebo_utils full_sim.launch.py` on a headless Gazebo Harmonic install; assert the node graph contains the expected publishers/subscribers after 5s.
- **RGBD / LIDAR demo end-to-end**: use a converted ROS2 bag (via `rosbags-convert`) and play it through the full `run_in_sim.launch.py` pipeline for 10s; assert no process crashes.

Each of these requires a real ROS2 Jazzy environment with the full build
toolchain (GTSAM via upstream, PCL, Eigen, Boost, Gazebo Harmonic, CUDA for
YOLO inference, etc.) and **cannot be run by the static suite**.

## See also

- [`tests/static/check_ros2_port.sh`](../static/check_ros2_port.sh) - static
  ROS2 port-consistency checks (no workspace needed).
- [`tests/python/`](../python/) - pytest-based checks for the Python glue
  (launch files, action definitions, parameter files).
