# `tests/static/` — ROS2 port static test suite

A permanent static-analysis suite for the `kr_autonomous_flight`
`ros2_dev` branch, used to catch regressions in the ongoing
ROS1 Noetic -> ROS2 Jazzy port.

## What it is

Two Bash files and this README:

```
tests/static/
  check_ros2_port.sh   # runner (18 sections A-R + 2 sub-checks)
  lib.sh               # colors, counters, comment-strippers, file listers
  README.md            # this file
```

No Python or CMake, no build step. Only needs `bash`, `awk`, `grep`,
`find`, `sed`, `tr`. Runs unchanged in CI containers and on a laptop.

## How to run

From any directory:

```bash
tests/static/check_ros2_port.sh
```

With verbose output (dumps the first 10 hits per failing check):

```bash
tests/static/check_ros2_port.sh --verbose
# or
VERBOSE=1 tests/static/check_ros2_port.sh
```

With an explicit repo root:

```bash
tests/static/check_ros2_port.sh /path/to/kr_autonomous_flight
```

The script exits with status `0` on success and `1` on any failure.
The final line reports `N checks, P passed, F failed, S skipped`.
Warnings are non-fatal and reported separately.

Disable ANSI colors with `NO_COLOR=1` (also automatic when stdout is
not a TTY).

## What it checks

| Section | Check |
|---------|-------|
| **A** | ROS1 C++ idioms (`ros/ros.h`, `ros::NodeHandle`, `ROS_INFO`, old `.h` message headers, `tf2/*.h`, `nodelet::Nodelet`, `actionlib::Simple*`, etc.) |
| **B** | ROS1 Python idioms (`import rospy`, `rospy.*` APIs, `from tf`, `ros_numpy`, `rospkg.*`, `actionlib.Simple*`) |
| **C** | `package.xml` uses format 3 + `ament_cmake`/`ament_python` buildtool, no `catkin`, no `message_generation/runtime` |
| **D** | `CMakeLists.txt` has `ament_package()` and no catkin leftovers (`catkin_package`, `${catkin_*}`, `add_message_files`, `generate_messages`, etc.) |
| **E** | No XML `*.launch` files remain; every `*.launch.py` imports `LaunchDescription` and defines `generate_launch_description()` |
| **F** | `*.action` files are rosidl-compatible (no bare `Header`, `time`, `duration`; snake_case field names; exactly two `---` separators) |
| **G** | Every `install(PROGRAMS ...)` path exists on disk (handles multi-line blocks via `in_install >= 1` state machine) |
| **H** | No legacy `nodelet_plugins.xml` (plural) — the singular `nodelet_plugin.xml` in `action_trackers` is allowed |
| **I** | No stray XML `.launch` files anywhere in the repo |
| **J** | Within each `*.launch.py`, every `LaunchConfiguration('x')` has a matching `DeclareLaunchArgument('x', ...)` in the same file (whole-file slurp to handle multi-line declarations) |
| **K** | No hardcoded `/home/<user>/`, `/root/`, `/opt/slideslam_docker_ws`, `/opt/bags/` paths in source |
| **L** | Every `Node(package='<managed>', executable='<y>')` resolves to an `add_executable` target or an `install(PROGRAMS)` entry in `<managed>`'s `CMakeLists.txt` |
| **M** | No duplicate raw `declare_parameter("key", ...)` calls across files within the same package (wrapper calls `*_declare_or_get<T>(...)` are ignored) |
| **N** | Dockerfiles / CI workflows / shell scripts contain no `noetic`, `ubuntu:20.04`, `roslaunch`, `rosrun`, `rosbag`, `catkin build`, `source /opt/ros/noetic`, etc. |
| **O** | Every `#include <pkg/...>` in a managed package's C/C++ sources resolves to an entry in that package's `package.xml` (via `<depend>`, `<build_depend>`, `<exec_depend>`, `<test_depend>` or `<buildtool_depend>`). Owning-package is resolved by walking up the directory tree. System libs (`Eigen`, `boost`, `pcl`, `gtsam`, `opencv2`, `gtest`, `benchmark`, `yaml-cpp`, `fmt`, `glog`, `tbb`, POSIX headers) are exempt because they're pulled via `find_package` + `target_link_libraries`, not `<depend>`. An include-prefix → package-name map is built from `<pkg>/include/<subdir>` so that cross-package includes whose top-level dir name differs from the package name (e.g. `mpl_collision/` → `motion_primitive_library`) resolve correctly. |
| **P** | For every `Node(package='<managed>', executable='<y>', parameters=[{'k': v, ...}])` in a `*.launch.py`, every literal dict key `k` is declared in the target package's C++/Python source as `declare_parameter("k", ...)` (template or plain form), `get_param_or(node, "k", ...)`, or `declare_parameter_if_not_declared(node, "k", ...)`. Catches the classic ROS2 bug where a launch file passes a parameter that the target node silently ignores because it never calls `declare_parameter`. Only dict-literal `parameters=[{...}]` is analyzed; yaml-file-path parameters, `ComposableNode`s, and dynamically-built `params` variables are skipped. Packages outside the 22 managed list are also skipped. |
| **Q** | Every `*.sh` / `*.bash` under `autonomy_*/` and `tests/` passes `bash -n`. Skipped (with a clear reason) when `bash -n` isn't usable in the runtime sandbox. |
| **R** | For every cross-package path reference in a `*.launch.py` — `PathJoinSubstitution([FindPackageShare('X'), ...])`, `PathJoinSubstitution([<var>, ...])` with `<var> = get_package_share_directory('X')` or `<var> = FindPackageShare('X')`, `os.path.join(get_package_share_directory('X'), ...)`, or `os.path.join(<var>, ...)` — the resolved path exists in `<X>`'s source tree. Catches launch files whose config / rviz / sub-launch path references went stale during the port. External packages (not in the 22-package managed map) are skipped; directory-only targets (no extension) are skipped. Carve-outs: `msckf_calib.yaml` + `msckf_calib_auto_generated.yaml` (generated at first run by `msckf_calib_gen`), `mapper_3d.yaml` + `tracker_params_mp_3d.yaml` (referenced by `polypixel_full_sim.launch.py`'s `use_3d` branch but never shipped upstream — pre-existing bug). |

Two sub-checks report as **warnings** (non-fatal):

| Sub-check | Warning |
|-----------|---------|
| **RG1** | `**/cfg/*.cfg` files exist (ROS2 has no `dynamic_reconfigure` drop-in; migrate to parameter callbacks) |
| **RG2** | Package-level `README.md` files mention `roslaunch`, `rosrun`, `rosbag` |

## Intentional exemptions

The suite encodes these domain exemptions and will **not** flag them:

1. **`action_trackers` pluginlib pattern.** The four tracker plugins
   (`land_tracker.cpp`, `take_off_tracker.cpp`, `stopping_policy.cpp`,
   `trajectory_tracker_upgraded.cpp`) legitimately use
   `PLUGINLIB_EXPORT_CLASS(X, kr_trackers_manager::Tracker)` and
   `#include <pluginlib/class_list_macros.hpp>`. This is the correct
   ROS2 pattern for plugins that are not `rclcpp::Node` components.
   Section A flags only `PLUGINLIB_EXPORT_CLASS(..., nodelet::...)`
   and the `.h` form of the macro header, never the
   `kr_trackers_manager::Tracker` export.

2. **`nodelet_plugin.xml` (singular) in `action_trackers`** is the
   pluginlib descriptor for the tracker plugins and is installed via
   `pluginlib_export_plugin_description_file` in the CMakeLists.
   Section H only flags the plural `nodelet_plugins.xml`.

3. **`kr_trackers_manager/Tracker.h`** is an external package header
   that the tracker plugins include. The external package is still
   ROS1-only per the `external_*.yaml` TODOs. Section A explicitly
   filters this include out of its hit list.

4. **`README.md` top banner** mentions "ROS1 Noetic" / "Ubuntu 20.04"
   as part of describing what the repo is porting **from**. The
   repo-root `README.md` is not part of the scope for section N
   (which only checks `Dockerfile*`, CI workflows and shell scripts).

5. **Python TODO comments** that mention `rospy` / `rospkg` are
   stripped before grepping — section B only matches active code.

## Known regression guards

The suite encodes explicit test cases that would have caught earlier
port bugs:

1. **Shell scripts beyond `run.sh`.** Section N globs
   `**/*.sh` and `**/*.bash`, not just `run*.sh`. An earlier
   manual pass missed 7 `entrypoint.sh` and 2 `deploy.sh` scripts.

2. **`tf2/*.h` -> `.hpp` in Jazzy.** Section A specifically greps
   for `#include <tf2[^>]+\.h>` because an earlier pass left seven
   such includes in place.

3. **`.py` files outside `src/` / `script/`.** Section B walks all
   `*.py` files with `find`, not just common source directories,
   because an earlier pass missed 9 rogue `import rospy` lines.

4. **Multi-line `install(PROGRAMS ...)` blocks.** Sections G and L
   use a state machine with `in_install >= 1`, not `== 1`, because
   the earlier SlideSLAM check-M had a bug where it only matched
   the first line of multi-line blocks.

## Implementation notes for future maintainers

- **No character-by-character paren matching.** Every check that
  needs to read a multi-line CMake or Python construct uses a
  `tr '\n' ' '` one-liner to flatten the file into a single logical
  line, then `grep -oE` to extract the tokens of interest. A bash
  paren matcher once ran for 8 hours in SlideSLAM and had to be
  killed; do not re-introduce that pattern here.

- **`strip_cxx_comments` / `strip_py_comments`** in `lib.sh` are
  conservative: they only handle single-line `//...` and `#...`
  comments with a simple quoted-string heuristic. Multi-line
  `/* ... */` comments that span lines are out of scope. In
  practice this is good enough because the repo's launch files
  and trackers don't use multi-line block comments to hide
  forbidden tokens.

- **Section J (launch-arg consistency)** slurps the whole file by
  first flattening newlines to spaces with `tr`, then greps out
  every `DeclareLaunchArgument('name')` and `LaunchConfiguration('name')`
  occurrence. An earlier SlideSLAM version of this check worked
  line-by-line and produced 48 false positives on multi-line
  `DeclareLaunchArgument(\n 'name',\n default_value=...)` calls.
  Do not revert to the line-based form.

- **Counters and helpers** (`pass`, `fail`, `skip`, `warn`,
  `section`, `print_summary`) live in `lib.sh`. Color codes
  respect `NO_COLOR` and auto-disable on non-TTY stdout.

- **Managed-package list** lives in `check_ros2_port.sh` as
  `MANAGED_PKGS=(...)`. If a new package is added to the repo
  (or an in-tree package is renamed), update this array and the
  section L resolver will automatically pick it up.

- **Verbose mode** prints the first 10 matching hits per failing
  check when `VERBOSE=1` or `--verbose` is passed. For
  section F (action files) it prints up to 5 offending lines per
  file.

## False-positive risks to watch

- **Section A "ros::Duration"** also matches `rclcpp::ros::Duration`
  and any struct that happens to be named `ros`. In practice the
  repo has no such names; if one is ever added, exempt it with a
  targeted `grep -v`.

- **Section J** matches only quoted-string argument names. Dynamic
  names built from variables (`DeclareLaunchArgument(name_var, ...)`)
  are ignored entirely. This is intentional — within-file static
  analysis can't resolve runtime strings.

- **Section M** matches `declare_parameter(` via a simple regex.
  Calls split across multiple source lines
  (`node_->declare_parameter(\n  "key", ...);`) will be missed if
  the key is on the continuation line. Update to a multi-line
  slurp if that case is ever added to the repo.

- **Section N shell-script check** only skips whole lines that
  begin with `#`. Inline `# ...` comments are NOT stripped because
  shell `#` has too many overloaded meanings (`#!/hashbang`,
  `${var#prefix}`, `$#`, quoted `#`). As a result, a legitimate
  inline comment that contains a forbidden token (e.g.
  `echo foo  # replaces the old roslaunch call`) will be flagged.
  If that becomes a real false positive, either move the comment
  to its own line or add a targeted exemption.
