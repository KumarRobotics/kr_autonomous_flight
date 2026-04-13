"""Static checks on every launch file under the managed autonomy_* trees.

In ROS2 launch is Python, not XML. Catching leftover ``.launch`` XML files
and verifying the Python files are well-formed is cheap and nips a whole
class of "launch crashes at ros2 launch" bugs in the bud.

Checks implemented here:

1. ``test_no_xml_launch``
   No ``*.launch`` (ROS1 XML) files may exist under autonomy_core/,
   autonomy_real/, or autonomy_sim/. There's no allowlist -- everything was
   migrated to ``.launch.py`` during the ROS2 port.

2. ``test_launch_py_structure``
   Every ``*.launch.py`` must import from ``launch`` and define
   ``generate_launch_description``; it must *not* contain a literal XML
   ``<launch>`` tag (that's a red flag for cut-and-paste migration attempts).

3. ``test_launch_arg_consistency``
   Every ``LaunchConfiguration('x')`` reference in a file must have a
   matching ``DeclareLaunchArgument('x', ...)`` in the *same* file. A small
   allow-list of launch builtins (log_level, launch_prefix, ...) is
   exempted. The whole file is read at once -- we deliberately do *not*
   iterate line by line, because python multi-line string literals make
   naive per-line matching wrong.

4. ``test_node_executables_resolve``
   For every ``Node(package='X', executable='Y')`` where X is one of the
   managed packages, Y must resolve to an ``add_executable(Y ...)`` target
   in X's CMakeLists, an explicit ``install(PROGRAMS .../Y ...)`` entry, a
   script under a directory installed via ``install(DIRECTORY scripts/
   ...)``, or a console-script entry-point in X's setup.py (for
   ament_python packages). External packages are skipped.

Test IDs are one per file to make failures self-documenting.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Set, Tuple

import pytest

from _packages import MANAGED_PACKAGE_DIRS, REPO_ROOT


_SEARCH_ROOTS = ("autonomy_core", "autonomy_real", "autonomy_sim")


# Builtin launch arguments that the launch system provides for free; we
# neither declare nor need to declare these, but they turn up in
# LaunchConfiguration() calls. Matches what rosdistro's rclcpp_components
# and launch itself ship.
_BUILTIN_LAUNCH_ARGS: Set[str] = {
    "log_level",
    "launch_prefix",
    "use_sim_time",
    "node_log_level",
    "output_log_level",
}


def _collect_launch_py() -> List[Path]:
    paths: List[Path] = []
    for sub in _SEARCH_ROOTS:
        base = REPO_ROOT / sub
        if base.is_dir():
            paths.extend(sorted(base.rglob("*.launch.py")))
    return paths


def _collect_xml_launch() -> List[Path]:
    paths: List[Path] = []
    for sub in _SEARCH_ROOTS:
        base = REPO_ROOT / sub
        if base.is_dir():
            paths.extend(sorted(base.rglob("*.launch")))
    return paths


_LAUNCH_PY_FILES = _collect_launch_py()
_LAUNCH_PY_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _LAUNCH_PY_FILES
]


# Map from package name (not directory) to its directory. We need to look up
# a CMakeLists / setup.py by the string that appears in ``package='X'``.
_PKG_NAME_TO_DIR: Dict[str, Path] = {
    pkg: REPO_ROOT / relpath for relpath, pkg in MANAGED_PACKAGE_DIRS
}
_MANAGED_PKG_NAMES: Set[str] = set(_PKG_NAME_TO_DIR.keys())


# -- regexes ---------------------------------------------------------------

# LaunchConfiguration('x') or LaunchConfiguration("x"). We accept any single
# keyword argument position, i.e. LaunchConfiguration('name', default=...).
_LAUNCH_CONFIG_RE = re.compile(r"LaunchConfiguration\(\s*['\"]([^'\"]+)['\"]")

# DeclareLaunchArgument('x', ...) or DeclareLaunchArgument(name='x', ...)
_DECLARE_ARG_RE = re.compile(
    r"DeclareLaunchArgument\(\s*(?:name\s*=\s*)?['\"]([^'\"]+)['\"]"
)

# Node(package='X', executable='Y'). Regex-matching a full Node(...) call
# with nested parentheses is unreliable (launch files routinely contain
# tuples like ``('~/odom', 'odom')`` inside the kwargs), so we instead
# use ``_iter_node_calls`` below to do a paren-balanced extraction of the
# call body and apply these simple kwarg regexes to it.
_NODE_START_RE = re.compile(r"\bNode\s*\(")
_PACKAGE_KW_RE = re.compile(r"package\s*=\s*['\"]([^'\"]+)['\"]")
_EXECUTABLE_KW_RE = re.compile(r"executable\s*=\s*['\"]([^'\"]+)['\"]")


def _iter_node_calls(text: str):
    """Yield the body string of every ``Node(...)`` call in ``text``.

    This walks the source balancing ``()``, ``[]`` and ``{}`` brackets while
    ignoring characters inside single and triple-quoted string literals so
    that a ``)`` inside a tuple or a string does not prematurely close the
    match. It's not a full Python parser but it handles the launch-file
    idioms we see in this repo.
    """

    i = 0
    n = len(text)
    while True:
        match = _NODE_START_RE.search(text, i)
        if match is None:
            return
        body_start = match.end()  # just past the opening '('
        depth = 1
        j = body_start
        in_str = None  # current string delimiter, or None
        while j < n and depth > 0:
            ch = text[j]
            if in_str is not None:
                if ch == "\\" and j + 1 < n:
                    j += 2
                    continue
                if text.startswith(in_str, j):
                    j += len(in_str)
                    in_str = None
                    continue
                j += 1
                continue
            # Not currently inside a string.
            if ch in ("'", '"'):
                # Triple-quoted string?
                if text.startswith(ch * 3, j):
                    in_str = ch * 3
                    j += 3
                    continue
                in_str = ch
                j += 1
                continue
            if ch == "#":
                # Comment to end of line.
                eol = text.find("\n", j)
                j = n if eol == -1 else eol
                continue
            if ch in "([{":
                depth += 1
            elif ch in ")]}":
                depth -= 1
                if depth == 0:
                    yield text[body_start:j]
                    j += 1
                    break
            j += 1
        i = j


def test_no_xml_launch() -> None:
    """No ROS1-era ``*.launch`` XML files under the autonomy_* trees."""

    offenders = _collect_xml_launch()
    rel_offenders = [
        str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in offenders
    ]
    assert not offenders, (
        "ROS1-era .launch XML files are still present: {}".format(rel_offenders)
    )


@pytest.mark.parametrize("launch_path", _LAUNCH_PY_FILES, ids=_LAUNCH_PY_IDS)
def test_launch_py_structure(launch_path: Path) -> None:
    """Every ``*.launch.py`` must import from launch and define the factory."""

    text = launch_path.read_text(errors="replace")

    assert "from launch import" in text or "from launch." in text, (
        "{} does not import anything from the 'launch' module".format(launch_path)
    )
    assert "def generate_launch_description" in text, (
        "{} does not define generate_launch_description()".format(launch_path)
    )
    assert "<launch>" not in text, (
        "{} contains a literal <launch> tag -- looks like XML cut-and-paste"
        .format(launch_path)
    )


@pytest.mark.parametrize("launch_path", _LAUNCH_PY_FILES, ids=_LAUNCH_PY_IDS)
def test_launch_arg_consistency(launch_path: Path) -> None:
    """Every LaunchConfiguration() must have a DeclareLaunchArgument() peer."""

    text = launch_path.read_text(errors="replace")

    used: Set[str] = set(_LAUNCH_CONFIG_RE.findall(text))
    declared: Set[str] = set(_DECLARE_ARG_RE.findall(text))

    missing = sorted(name for name in used if name not in declared
                     and name not in _BUILTIN_LAUNCH_ARGS)
    assert not missing, (
        "{}: LaunchConfiguration() references without matching "
        "DeclareLaunchArgument(): {}".format(launch_path, missing)
    )


# -- executable resolution -------------------------------------------------


def _extract_cmake_targets(cmake_text: str, package_dir: Path) -> Set[str]:
    """Return the set of executable names discoverable in a CMakeLists.

    Sources, in order:
      - ``add_executable(name ...)``
      - ``install(PROGRAMS path1 path2 ... DESTINATION ...)`` -- the basename
        of each path is the runtime executable name.
      - ``install(DIRECTORY scripts/ DESTINATION ...)`` -- in this case we
        enumerate every file directly inside the real ``scripts/`` folder
        and add its basename.
    """

    targets: Set[str] = set()

    for match in re.finditer(r"add_executable\s*\(\s*([A-Za-z0-9_\-]+)", cmake_text):
        targets.add(match.group(1))

    for match in re.finditer(
        r"install\s*\(\s*PROGRAMS\b(?P<body>.*?)\)", cmake_text, re.DOTALL | re.IGNORECASE
    ):
        body = match.group("body")
        dest_idx = body.upper().find("DESTINATION")
        if dest_idx >= 0:
            body = body[:dest_idx]
        for raw in body.split():
            token = raw.strip().strip(",").strip('"').strip("'")
            if token and "${" not in token:
                targets.add(Path(token).name)

    for match in re.finditer(
        r"install\s*\(\s*DIRECTORY\s+(?P<dirs>.*?)DESTINATION",
        cmake_text,
        re.DOTALL | re.IGNORECASE,
    ):
        dirs_body = match.group("dirs")
        for raw in dirs_body.split():
            token = raw.strip().strip(",").strip('"').strip("'").rstrip("/")
            if not token or "${" in token:
                continue
            candidate = package_dir / token
            if candidate.is_dir():
                for child in candidate.iterdir():
                    if child.is_file():
                        targets.add(child.name)

    return targets


def _extract_setup_py_entry_points(setup_text: str) -> Set[str]:
    """Extract ``console_scripts`` names from a setup.py entry_points block.

    Each entry is of the form ``'name = module:func'``; we want the bit
    before the ``=``.
    """

    targets: Set[str] = set()
    block = re.search(
        r"console_scripts['\"]\s*:\s*\[(?P<body>.*?)\]",
        setup_text,
        re.DOTALL,
    )
    if not block:
        return targets
    for line in block.group("body").splitlines():
        stripped = line.strip().strip(",").strip("'").strip('"')
        if not stripped:
            continue
        if "=" not in stripped:
            continue
        name = stripped.split("=", 1)[0].strip()
        if name:
            targets.add(name)
    return targets


def _executables_for_package(pkg_name: str) -> Set[str]:
    """All executable names recognised for ``pkg_name``, by any install path."""

    pkg_dir = _PKG_NAME_TO_DIR.get(pkg_name)
    if pkg_dir is None:
        return set()

    result: Set[str] = set()

    cmake = pkg_dir / "CMakeLists.txt"
    if cmake.is_file():
        result |= _extract_cmake_targets(cmake.read_text(errors="replace"), pkg_dir)

    setup = pkg_dir / "setup.py"
    if setup.is_file():
        result |= _extract_setup_py_entry_points(setup.read_text(errors="replace"))

    return result


@pytest.mark.parametrize("launch_path", _LAUNCH_PY_FILES, ids=_LAUNCH_PY_IDS)
def test_node_executables_resolve(launch_path: Path) -> None:
    """Every Node(package=managed_pkg, executable=X) must name a real target."""

    text = launch_path.read_text(errors="replace")
    problems: List[Tuple[str, str]] = []

    for body in _iter_node_calls(text):
        pkg_match = _PACKAGE_KW_RE.search(body)
        exe_match = _EXECUTABLE_KW_RE.search(body)
        if not pkg_match or not exe_match:
            continue
        pkg = pkg_match.group(1)
        exe = exe_match.group(1)
        if pkg not in _MANAGED_PKG_NAMES:
            continue
        available = _executables_for_package(pkg)
        if exe not in available:
            problems.append((pkg, exe))

    assert not problems, (
        "{} references executables that don't exist in their managed package: "
        "{}".format(launch_path, problems)
    )
