"""Shared constants for the tests/python/ suite.

This module is imported at collection time by nearly every test file in the
directory. We keep these things OUT of ``conftest.py`` because pytest treats
conftest specially (it's discovered but not necessarily importable by name in
all environments); plain ``_packages.py`` is unambiguously a regular module
sitting next to the tests.

Contents:

- ``REPO_ROOT``              absolute path to the repo top
- ``MANAGED_PACKAGE_DIRS``   tuple of ``(relpath, package_name)`` for every
                             managed package. ``relpath`` is the directory
                             under the repo root; ``package_name`` is the
                             string in ``<name>`` inside its package.xml.
                             A handful of packages (``mpl``,
                             ``state_machine_core``) have package names that
                             don't match their directory name.
- ``PKG_NAME_TO_DIR``        dict mapping package name -> absolute directory
- ``MANAGED_PKG_NAMES``      set of every package name above
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Set, Tuple


def _find_repo_root() -> Path:
    """Walk up from this file until we see both autonomy_core/ and tests/."""

    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "autonomy_core").is_dir() and (candidate / "tests").is_dir():
            return candidate
    raise RuntimeError(
        "could not locate kr_autonomous_flight repo root from {}".format(here)
    )


REPO_ROOT: Path = _find_repo_root()


MANAGED_PACKAGE_DIRS: Tuple[Tuple[str, str], ...] = (
    ("autonomy_core/client/client_launch",                 "client_launch"),
    ("autonomy_core/client/rqt_quadrotor_safety",          "rqt_quadrotor_safety"),
    ("autonomy_core/control/control_launch",               "control_launch"),
    ("autonomy_core/estimation/estimation_launch",         "estimation_launch"),
    ("autonomy_core/estimation/fla_ukf",                   "fla_ukf"),
    ("autonomy_core/interface/mavros_interface",           "mavros_interface"),
    ("autonomy_core/interface/px4_interface_launch",       "px4_interface_launch"),
    ("autonomy_core/map_plan/action_planner",              "action_planner"),
    ("autonomy_core/map_plan/coverage_utils",              "coverage_utils"),
    ("autonomy_core/map_plan/jps3d",                       "jps3d"),
    ("autonomy_core/map_plan/mapper",                      "mapper"),
    ("autonomy_core/map_plan/map_plan_launch",             "map_plan_launch"),
    ("autonomy_core/map_plan/mpl",                         "motion_primitive_library"),
    ("autonomy_core/map_plan/traj_opt_ros",                "traj_opt_ros"),
    ("autonomy_core/state_machine/action_trackers",        "action_trackers"),
    ("autonomy_core/state_machine/state_machine_core",     "state_machine"),
    ("autonomy_core/state_machine/state_machine_launch",   "state_machine_launch"),
    ("autonomy_real/real_experiment_launch",               "real_experiment_launch"),
    ("autonomy_sim/gazebo_sim/gazebo_utils",               "gazebo_utils"),
    ("autonomy_sim/unity_sim/dcist_utils",                 "dcist_utils"),
    ("autonomy_sim/unity_sim/fake_lidar",                  "fake_lidar"),
    ("autonomy_sim/unity_sim/fake_sloam",                  "fake_sloam"),
)


PKG_NAME_TO_DIR: Dict[str, Path] = {
    pkg: REPO_ROOT / relpath for relpath, pkg in MANAGED_PACKAGE_DIRS
}


MANAGED_PKG_NAMES: Set[str] = set(PKG_NAME_TO_DIR.keys())


EXEMPT_PLUGIN_CPP: Set[Path] = {
    (
        REPO_ROOT
        / "autonomy_core"
        / "state_machine"
        / "action_trackers"
        / "src"
        / name
    ).resolve()
    for name in (
        "stopping_policy.cpp",
        "take_off_tracker.cpp",
        "land_tracker.cpp",
        "trajectory_tracker_upgraded.cpp",
    )
}
