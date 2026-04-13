# tests/python -- static-analysis pytest suite

This directory holds a pure-Python pytest suite that mirrors the `tests/static`
bash checks. It's designed to run on any machine that has `python3` + `pytest`,
with **no ROS2 runtime required** -- no `rclpy`, no `rosidl`, no colcon.

The suite catches things that the compiler won't, and that a smoke test is
too slow to check:

- package.xml manifests that still say `catkin`
- CMakeLists that install files that don't exist
- `.launch.py` files whose `LaunchConfiguration('x')` has no matching
  `DeclareLaunchArgument('x', ...)`
- `Node(package='foo', executable='bar')` references that don't resolve to
  anything in `foo`'s CMakeLists / setup.py
- `.action` files with bare `Header` / `time` / `duration` types
- ROS1 idioms (`rospy`, `roslaunch`, `ROS_INFO`, `nodelet::Nodelet`, ...)
  sneaking back into C++, Python, Dockerfiles, or CI workflows
- tf2 headers still ending in `.h` instead of `.hpp`
- `declare_parameter("same_key", ...)` called from two files in the same
  package (a double-declare crash waiting to happen)

## Running it

From the repo root:

```bash
python3 -m pytest tests/python -q
```

Or, if you want one specific file:

```bash
python3 -m pytest tests/python/test_package_xml.py -v
```

Each test file is parametrised so failing cases name the offending file in
the test ID; re-run just the failures with `-k`:

```bash
python3 -m pytest tests/python -k state_machine_launch
```

## Adding a new check

1. Drop a new `test_<something>.py` into this directory.
2. Put the rationale in a module docstring -- the whole point of a static
   suite is that future contributors can read _why_ a rule exists.
3. If you need the list of managed packages or the repo root, import them
   from `conftest.py` via the `packages` / `repo_root` fixtures.
4. Do not pull in `rclpy` or `rosidl`. We keep this suite runnable on hosts
   that don't have a ROS2 install.

## Dependencies

Only `pytest` is required. A pinned version isn't needed; anything from
pytest 7.x onwards will work. Standard library only.
