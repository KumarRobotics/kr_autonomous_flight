# kr_autonomous_flight — ROS2 port test suite

This directory hosts the automated checks that guard the ROS1 → ROS2 Jazzy port of `kr_autonomous_flight`. Its primary job is to catch mechanical regressions in the migration so that a reviewer or CI runner can confirm, with zero setup, that the in-tree packages are still ROS2-shaped.

The suite is structured so that **anyone can run the primary layer with nothing more than a POSIX shell and ripgrep** — no ROS2 install, no Python environment, no compiled workspace required. Additional layers are available for CI ergonomics and for optional runtime smoke tests on a real ROS2 Jazzy machine.

For a detailed, per-package breakdown of what was migrated and the open follow-up items, see [`../ROS2_MIGRATION_REPORT.md`](../ROS2_MIGRATION_REPORT.md).

---

## Layers

```
tests/
  static/
    check_ros2_port.sh    # Layer 1: bash + ripgrep + awk. Runs anywhere.
    lib.sh                # shared helpers (colors, counters, file listers)
  python/
    test_*.py             # Layer 2: pytest mirror of the bash runner.
    conftest.py           # pytest config + shared fixtures
  integration/
    launch_smoke_test.sh  # Layer 3: ros2 launch --print-description smoke test.
  README.md               # this file
```

Each layer has a different dependency profile and a different set of trade-offs:

| Layer | Dependencies | What it checks | When to run |
|:---|:---|:---|:---|
| 1. Static (`tests/static/`) | bash, ripgrep, awk, POSIX coreutils | Mechanical port shape: no `rospy`, no `ros::NodeHandle`, `ament_*` scaffolds, `.action` files wired up, Docker base images, etc. | **Every commit.** No setup, zero runtime cost. |
| 2. Python (`tests/python/`) | Python 3 + `pytest` | Same checks as layer 1, structured for CI ergonomics and assertion diffs. | **Every PR.** CI gate. |
| 3. Integration (`tests/integration/`) | `ros2` on `$PATH` (no workspace build required) | `ros2 launch --print-description` parses every `*.launch.py` without actually starting any nodes. | **Pre-release sanity check** on a Jazzy workstation. |

None of the three layers requires `colcon build` to have succeeded — that is explicitly outside the scope of this suite. See the [What this suite cannot catch](#what-this-suite-cannot-catch) section below for the runtime items that need a real built workspace.

---

## Layer 1 — Static checks (`tests/static/check_ros2_port.sh`)

Pure bash + ripgrep + awk. The runner is a drop-in: you do not need ROS2 installed, you do not need Python, you do not need to source anything. The only host requirements are `bash`, `rg` (ripgrep), `awk`, and POSIX coreutils.

```bash
bash tests/static/check_ros2_port.sh
```

**Current state:** *(run the command above to get the live count — a typical run currently produces something like `N checks, P passed, F failed, S skipped`. The exact numbers move as the port progresses.)*

Options:

```bash
VERBOSE=1 bash tests/static/check_ros2_port.sh     # dump full details on every match
NO_COLOR=1 bash tests/static/check_ros2_port.sh    # disable ANSI colors (useful for CI logs)
```

The runner is organized into 14 labeled sections (A–N); each section corresponds to one family of regressions that the port could reintroduce. See [What this suite can catch](#what-this-suite-can-catch) for the full list.

The bash layer is the **canonical** spec — when in doubt, consult the `check_ros2_port.sh` source. It is deliberately self-contained so that a reviewer can read it top-to-bottom in ~10 minutes.

---

## Layer 2 — pytest mirror (`tests/python/`)

A one-to-one Python mirror of the bash runner, structured so that CI can report per-check pass / fail with assertion diffs. This layer exists because bash `[FAIL]` lines are harder for most CI dashboards to reason about than `pytest` output.

```bash
pip install pytest
pytest tests/python -v
```

The pytest files are named `test_section_<letter>.py` to make the mapping to the bash sections obvious. Each pytest file defines one test function per check, collects the same data the bash runner does (via the `conftest.py` fixtures that walk the in-tree packages), and asserts the same invariants.

**Critical:** the two layers must stay in sync. If you add a check to `tests/static/check_ros2_port.sh`, add a matching `pytest` case to `tests/python/test_section_<letter>.py` in the same PR. See [Adding a new check](#adding-a-new-check) below.

---

## Layer 3 — Launch smoke test (`tests/integration/launch_smoke_test.sh`)

A runtime parse-only check that walks every `*.launch.py` in the tree and runs `ros2 launch --print-description` on it. `--print-description` resolves the `LaunchDescription` without actually starting any nodes, so it:

- **Does** catch missing `IncludeLaunchDescription` paths, undeclared `LaunchConfiguration` references, Python import errors in `generate_launch_description()`, and syntax errors in the launch file itself.
- **Does not** catch anything that needs a running node (rclcpp_action handshakes, TF chain correctness, component loading, etc.) — for that, you need a built workspace and a live `ros2 launch`.

```bash
source /opt/ros/jazzy/setup.bash
bash tests/integration/launch_smoke_test.sh
```

Each file is flagged one of:

- `[OK]` — the launch description was parsed cleanly.
- `[FAIL]` — Python raised an exception or `ros2 launch` returned a non-zero exit code.
- `[TIMEOUT]` — the launch file blocked past `LAUNCH_TIMEOUT` seconds (default 20). Typically means a blocking call at import time or a misbehaving `OpaqueFunction`.

Options:

```bash
LAUNCH_TIMEOUT=30 bash tests/integration/launch_smoke_test.sh
VERBOSE=1 bash tests/integration/launch_smoke_test.sh      # dump full captured output on failure
NO_COLOR=1 bash tests/integration/launch_smoke_test.sh
```

If `ros2` or GNU `timeout` is missing, the script exits with the autotools "skipped" code (`77`) rather than failing — so a CI runner without a ROS2 environment treats it as skipped rather than broken.

Additional runtime checks (component load tests, action round-trip, mapper voxelmap smoke test) are **TODO** and will live under `tests/integration/` when added.

---

## What this suite can catch

One-bullet summary of each of the 14 static sections (A–N):

- **A. `package.xml` build_type.** Every in-tree `package.xml` is `format="3"` and declares `<export><build_type>ament_cmake</build_type></export>` or `ament_python`.
- **B. `CMakeLists.txt` ament scaffold.** Every `CMakeLists.txt` calls `find_package(ament_cmake REQUIRED)` and ends with `ament_package()`. No `catkin_package()` / `catkin REQUIRED COMPONENTS` remnants.
- **C. C++ header hygiene.** No `#include <ros/ros.h>` / `<nodelet/nodelet.h>` / `<tf/transform_listener.h>` / `<actionlib/*>` / `<dynamic_reconfigure/*>` in any in-tree `.cpp` / `.h` / `.hpp`.
- **D. C++ API hygiene.** No `ros::NodeHandle`, `ros::Publisher`, `ros::Time::now()`, `ROS_INFO`, `ROS_WARN`, etc. in stripped (comment-free) source.
- **E. Python `rospy` hygiene.** No `import rospy`, `rospy.init_node`, `rospy.Publisher`, `rospy.get_param`, `rospy.Time.now()` in stripped Python source.
- **F. `tf` → `tf2` hygiene.** No `import tf` (without the `2`) in Python; no `tf/transform_*` C++ headers.
- **G. Launch files.** Every `*.launch` XML (if any remain) is accompanied by a `*.launch.py`. No stray `<launch>` XML files that don't also have a Python twin.
- **H. `.action` interface generation.** Every `.action` file lives in a package whose `CMakeLists.txt` calls `rosidl_generate_interfaces(... DEPENDENCIES ...)` and whose `package.xml` declares `rosidl_default_generators` + `rosidl_default_runtime` + `<member_of_group>rosidl_interface_packages</member_of_group>`.
- **I. Pluginlib plugins.** Every package with a `PLUGINLIB_EXPORT_CLASS` call exports the plugin description via `pluginlib_export_plugin_description_file(...)`.
- **J. Components.** Every file with `RCLCPP_COMPONENTS_REGISTER_NODE` has a corresponding `rclcpp_components_register_nodes(...)` call in its `CMakeLists.txt`.
- **K. Docker hygiene.** No `FROM nvidia/cudagl:*` or `FROM ros:noetic-*` in any in-tree Dockerfile; no `catkin build` / `source /opt/ros/noetic/setup.bash` in any in-tree shell script or Dockerfile.
- **L. GitHub Actions.** Every `docker-build-*.yaml` triggers off `branches: [ros2_dev]` and produces a `-jazzy` tag.
- **M. `external_*.yaml` TODOs.** Every `external_*.yaml` manifest has a top-of-file TODO comment flagging that its entries are still ROS1-only.
- **N. Dangling launch includes.** Every `IncludeLaunchDescription(...)` argument that references a local `*.launch.py` points at a file that actually exists.

Plus — from layer 3 — the launch-graph smoke test confirms that every in-tree `*.launch.py` can be parsed by `ros2 launch --print-description` without raising a Python exception.

---

## What this suite cannot catch

These are runtime-only items. They require a real ROS2 Jazzy machine and (usually) a successful `colcon build`. They are **out of scope** for this suite and are tracked in the [Migration Report](../ROS2_MIGRATION_REPORT.md):

- **`colcon build` end-to-end.** Blocked on external dependency ports (`kr_trackers_manager`, `kr_mav_control`, `DecompROS`, `msckf_vio`, `mrsl_quadrotor`, `faster-lio`, etc.).
- **`rclcpp_action` goal / feedback / result round-trips** against live clients.
- **Component loading.** `fla_ukf::FLAUKFNodelet` and `mavros_interface::SO3CmdToMavros` need to actually load inside a `component_container`.
- **Pluginlib tracker load.** `kr_trackers_manager` needs to `createInstance` each of the 4 ported trackers and call `Initialize` on them.
- **Gazebo Harmonic bring-up.** Blocked on the `mrsl_quadrotor` upstream's Gazebo Classic → Harmonic migration.
- **`mapper` VoxelMap ingest.** The `message_filters` TF-synced point-cloud pipeline hasn't been exercised.
- **TF tree correctness.** Whether `<robot_ns>/odom` → `<robot_ns>/map` is populated at the right rate by the SLAM module.
- **Full SLAM demo on a converted ROS2 bag.**

---

## Adding a new check

When you discover a new regression family (e.g. "no one should use `std::bind` with a raw pointer in an `rclcpp` callback"), add it in **both** layers in the same PR:

1. **`tests/static/check_ros2_port.sh`** — add a new `section "O. <label>"` (or extend an existing section) with the `pass` / `fail` / `skip` calls from `tests/static/lib.sh`. Keep the check pure bash + ripgrep + awk so that the runner still has zero dependencies.
2. **`tests/python/test_section_<letter>.py`** — add a matching pytest function that asserts the same invariant. Use the fixtures in `tests/python/conftest.py` to enumerate the in-tree files (they mirror the `list_*` helpers in `tests/static/lib.sh`).
3. **`ROS2_MIGRATION_REPORT.md`** — add a row to the table in Section 5 ("What the static test suite enforces") so the report stays in sync with the runner.

The bash runner is the **canonical** specification; the pytest layer is the CI-ergonomic mirror. If the two ever disagree, fix the bash runner first and then sync the pytest mirror.

## Troubleshooting

- **`tests/static/check_ros2_port.sh: command not found: rg`.** Install ripgrep (`sudo apt install ripgrep`). The runner intentionally uses ripgrep instead of grep because it is significantly faster on a large source tree.
- **`[SKIP]` on a check that should not be skipped.** Skips fire when a subsection has no applicable files (e.g. "check every `.action` file" when there are no `.action` files in that package). If a section is skipping unexpectedly, check that the `list_*` helper in `lib.sh` is finding the files you expect.
- **`launch_smoke_test.sh` exits with code 77.** Either `ros2` or GNU `timeout` is missing from `$PATH`. Source `/opt/ros/jazzy/setup.bash` and install `coreutils` if you are on a non-GNU system.
- **`launch_smoke_test.sh` reports `TIMEOUT` on a file that parses fine locally.** Raise `LAUNCH_TIMEOUT` (default 20 seconds) via the environment variable. Some launch files that resolve `OpaqueFunction`s can legitimately take 30+ seconds on a cold cache.
