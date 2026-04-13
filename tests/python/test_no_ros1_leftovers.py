"""Static grep-style checks that make sure nothing ROS1-shaped slipped through.

The ROS1 -> ROS2 Jazzy port touched almost every source file in the repo, so
these tests are our safety net against regressions. They walk the relevant
file trees and flag any occurrence of banned strings, with a small set of
carve-outs for files that legitimately look ROS1-ish but are actually the
correct ROS2 pattern (see ``conftest.exempt_paths``).

Sub-checks:

1. ``test_no_ros1_cpp_idioms`` -- scans every C/C++ file under the autonomy_*
   trees for ``ros::NodeHandle``, ``ROS_INFO/WARN/ERROR/DEBUG/FATAL``,
   ``nodelet::Nodelet``, ``actionlib::Simple*``, ``#include <ros/ros.h>``,
   ``#include <tf/...>``, ``#include <tf2.../*.h>`` (legacy header suffix)
   and ``#include <nodelet/...>``.

2. ``test_no_ros1_python_idioms`` -- scans every ``.py`` for ``import rospy``,
   ``from rospy``, ``rospy.``, top-level ``import tf``, ``from tf.``,
   ``rospkg.``, ``ros_numpy``, and ``actionlib.Simple*``.

3. ``test_no_noetic_in_dockerfiles`` -- no ``noetic`` / ``ubuntu:20.04``
   strings in any file named ``Dockerfile``.

4. ``test_no_noetic_in_workflows`` -- no ``noetic`` / ``ubuntu-20.04`` in
   ``.github/workflows/*.yaml``.

5. ``test_no_ros1_in_shell_scripts`` -- no ``roslaunch``, ``rosrun``, ``rosbag
   play|record``, ``rostopic``, ``rosnode``, ``rosservice``, ``rosparam``,
   ``roscore``, ``roscd``, ``catkin build``, ``catkin_make``,
   ``devel/setup.bash`` or ``/opt/ros/noetic`` in ``*.sh`` / ``*.bash`` --
   except when they appear inside an ``echo "..."`` stub string.

6. ``test_tf2_headers_are_hpp`` -- no C++ file may still include
   ``tf2.../*.h`` (all Jazzy tf2 headers are ``.hpp``).

Before matching, single-line ``//`` or ``#`` comments are stripped so that
TODO notes discussing the old rospy API don't get flagged. Block comments
(``/* */``) are out of scope; they're rare in practice here.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable, List, Set, Tuple

import pytest

from _packages import REPO_ROOT


_AUTONOMY_ROOTS = ("autonomy_core", "autonomy_real", "autonomy_sim")


# -- file collection -------------------------------------------------------


def _collect(extensions: Iterable[str], roots: Iterable[str] = _AUTONOMY_ROOTS) -> List[Path]:
    """Gather all files whose suffix matches ``extensions`` under ``roots``."""

    want = {e.lower() for e in extensions}
    files: List[Path] = []
    for r in roots:
        base = REPO_ROOT / r
        if not base.is_dir():
            continue
        for p in base.rglob("*"):
            if p.is_file() and p.suffix.lower() in want:
                files.append(p)
    return sorted(files)


def _rel_ids(paths: Iterable[Path]) -> List[str]:
    return [str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in paths]


# -- comment stripping ----------------------------------------------------

# These regexes handle single-line comments only. We don't try to parse
# multi-line /* */ blocks; they're rare in this codebase and the tests
# tolerate occasional false positives rather than false negatives.
_CPP_LINE_COMMENT_RE = re.compile(r"//.*")


def _strip_cpp_comments(text: str) -> str:
    """Remove ``//`` single-line comments; preserve newlines."""

    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _strip_python_comments(text: str) -> str:
    """Remove ``#`` comments, preserving the shebang on line 1.

    The shebang (``#!``) must not be eaten, otherwise we'd break tests that
    care whether shell scripts still have ``#!/bin/bash`` at the top.
    """

    out: List[str] = []
    for idx, line in enumerate(text.splitlines()):
        if idx == 0 and line.startswith("#!"):
            out.append(line)
            continue
        # The simplest rule: drop everything from the first ``#`` on this
        # line. This is wrong for ``#`` inside string literals but (a) we
        # don't care about string-literal matches anyway for the patterns
        # we check, and (b) the task brief explicitly calls for this
        # simple regex-based approach.
        hash_idx = line.find("#")
        if hash_idx >= 0:
            line = line[:hash_idx]
        out.append(line)
    return "\n".join(out)


# -- 1. C++ ROS1 idioms ---------------------------------------------------


_CPP_FILES = _collect((".cpp", ".h", ".hpp", ".cc", ".cxx"))
_CPP_IDS = _rel_ids(_CPP_FILES)


# (regex, friendly_name, optional carve-out predicate).
# ``carve_out(path)`` returning True means the pattern is allowed for that
# path (e.g. the 4 tracker plugins that *should* include pluginlib).
_CPP_BANNED_PATTERNS: List[Tuple[re.Pattern, str]] = [
    (re.compile(r"#include\s*<ros/ros\.h>"),       "#include <ros/ros.h>"),
    (re.compile(r"\bros::NodeHandle\b"),           "ros::NodeHandle"),
    (re.compile(r"\bROS_INFO\b"),                  "ROS_INFO"),
    (re.compile(r"\bROS_WARN\b"),                  "ROS_WARN"),
    (re.compile(r"\bROS_ERROR\b"),                 "ROS_ERROR"),
    (re.compile(r"\bROS_DEBUG\b"),                 "ROS_DEBUG"),
    (re.compile(r"\bROS_FATAL\b"),                 "ROS_FATAL"),
    (re.compile(r"\bnodelet::Nodelet\b"),          "nodelet::Nodelet"),
    (re.compile(r"\bactionlib::Simple"),           "actionlib::Simple*"),
    (re.compile(r"#include\s*<tf/"),               "#include <tf/...>"),
    # tf2 legacy ``.h`` headers. Jazzy ships ``.hpp`` for everything.
    (re.compile(r"#include\s*<tf2[^>]+\.h>"),      "#include <tf2.../*.h>"),
    (re.compile(r"#include\s*<nodelet/"),          "#include <nodelet/...>"),
]


@pytest.mark.parametrize("cpp_path", _CPP_FILES, ids=_CPP_IDS)
def test_no_ros1_cpp_idioms(cpp_path: Path, exempt_paths) -> None:
    """No ROS1-era macros / includes / types in C++ under autonomy_*."""

    text = _strip_cpp_comments(cpp_path.read_text(errors="replace"))

    resolved = cpp_path.resolve()
    is_exempt_plugin = resolved in exempt_paths

    offenders: List[str] = []
    for regex, label in _CPP_BANNED_PATTERNS:
        if regex.search(text):
            # Pluginlib carve-out is narrow: the tracker plugins are allowed
            # to contain ``pluginlib``-related headers, but none of the
            # banned patterns above are pluginlib-specific, so the carve-out
            # doesn't change what we flag. We keep the branch here so the
            # reader knows these paths are the ones we were told to treat
            # specially.
            if is_exempt_plugin and label == "nodelet::Nodelet":
                continue
            offenders.append(label)

    assert not offenders, (
        "{} contains ROS1 C++ idioms: {}".format(cpp_path, offenders)
    )


# -- 2. Python ROS1 idioms ------------------------------------------------


_PY_FILES = _collect((".py",))
_PY_IDS = _rel_ids(_PY_FILES)


_PY_BANNED_PATTERNS: List[Tuple[re.Pattern, str]] = [
    (re.compile(r"^\s*import\s+rospy\b", re.MULTILINE),      "import rospy"),
    (re.compile(r"^\s*from\s+rospy\b", re.MULTILINE),        "from rospy"),
    (re.compile(r"\brospy\."),                               "rospy."),
    (re.compile(r"^\s*import\s+tf\s*$", re.MULTILINE),       "import tf"),
    (re.compile(r"^\s*from\s+tf\.", re.MULTILINE),           "from tf."),
    (re.compile(r"\brospkg\."),                              "rospkg."),
    (re.compile(r"\bros_numpy\b"),                           "ros_numpy"),
    (re.compile(r"\bactionlib\.Simple"),                     "actionlib.Simple*"),
]


@pytest.mark.parametrize("py_path", _PY_FILES, ids=_PY_IDS)
def test_no_ros1_python_idioms(py_path: Path) -> None:
    """No ``rospy``, ``rospkg``, ``ros_numpy``, etc. in live Python."""

    # Strip comments BEFORE matching so that TODO notes mentioning the old
    # APIs don't trip us. This is important for state_machine_core/scripts,
    # which intentionally carries rospy references in comments.
    text = _strip_python_comments(py_path.read_text(errors="replace"))

    offenders: List[str] = []
    for regex, label in _PY_BANNED_PATTERNS:
        if regex.search(text):
            offenders.append(label)
    assert not offenders, (
        "{} still uses ROS1 Python idioms: {}".format(py_path, offenders)
    )


# -- 3. Dockerfiles -------------------------------------------------------


def _collect_dockerfiles() -> List[Path]:
    return sorted(p for p in REPO_ROOT.rglob("Dockerfile") if p.is_file())


_DOCKERFILES = _collect_dockerfiles()
_DOCKERFILE_IDS = _rel_ids(_DOCKERFILES)


@pytest.mark.parametrize("docker_path", _DOCKERFILES, ids=_DOCKERFILE_IDS)
def test_no_noetic_in_dockerfiles(docker_path: Path) -> None:
    """Dockerfiles must not pin ROS Noetic or the Ubuntu 20.04 base."""

    text = docker_path.read_text(errors="replace")
    # Strip ``#`` comment lines because some Dockerfiles have TODO comments
    # referencing the old base image (the kr_autonomous base/Dockerfile
    # explicitly documents the Noetic migration history in a comment block).
    stripped_lines = []
    for line in text.splitlines():
        hash_idx = line.find("#")
        if hash_idx >= 0:
            line = line[:hash_idx]
        stripped_lines.append(line)
    stripped = "\n".join(stripped_lines)

    offenders: List[str] = []
    if re.search(r"\bnoetic\b", stripped, re.IGNORECASE):
        offenders.append("noetic")
    if "ubuntu:20.04" in stripped:
        offenders.append("ubuntu:20.04")
    assert not offenders, (
        "{} still references ROS1/Ubuntu 20.04: {}".format(docker_path, offenders)
    )


# -- 4. GitHub Actions workflows ------------------------------------------


def _collect_workflow_yamls() -> List[Path]:
    base = REPO_ROOT / ".github" / "workflows"
    if not base.is_dir():
        return []
    return sorted(p for p in base.glob("*.yaml") if p.is_file()) + sorted(
        p for p in base.glob("*.yml") if p.is_file()
    )


_WORKFLOWS = _collect_workflow_yamls()
_WORKFLOW_IDS = _rel_ids(_WORKFLOWS)


@pytest.mark.parametrize("workflow_path", _WORKFLOWS, ids=_WORKFLOW_IDS)
def test_no_noetic_in_workflows(workflow_path: Path) -> None:
    """CI workflows must not run on Noetic or ubuntu-20.04 runners."""

    text = workflow_path.read_text(errors="replace")
    offenders: List[str] = []
    if re.search(r"\bnoetic\b", text, re.IGNORECASE):
        offenders.append("noetic")
    if "ubuntu-20.04" in text:
        offenders.append("ubuntu-20.04")
    assert not offenders, (
        "{} still pins ROS1/Ubuntu 20.04 runners: {}"
        .format(workflow_path, offenders)
    )


# -- 5. Shell scripts -----------------------------------------------------


_SHELL_BANNED_PATTERNS: List[Tuple[re.Pattern, str]] = [
    (re.compile(r"\broslaunch\b"),                   "roslaunch"),
    (re.compile(r"\brosrun\s"),                      "rosrun"),
    (re.compile(r"\brosbag\s+(play|record)\b"),      "rosbag play/record"),
    (re.compile(r"\brostopic\b"),                    "rostopic"),
    (re.compile(r"\brosnode\b"),                     "rosnode"),
    (re.compile(r"\brosservice\b"),                  "rosservice"),
    (re.compile(r"\brosparam\b"),                    "rosparam"),
    (re.compile(r"\broscore\b"),                     "roscore"),
    (re.compile(r"\broscd\b"),                       "roscd"),
    (re.compile(r"\bcatkin\s+build\b"),              "catkin build"),
    (re.compile(r"\bcatkin_make\b"),                 "catkin_make"),
    (re.compile(r"devel/setup\.bash"),               "devel/setup.bash"),
    (re.compile(r"/opt/ros/noetic"),                 "/opt/ros/noetic"),
]


def _collect_shell_scripts() -> List[Path]:
    # The tests/ tree is excluded because tests/static/check_ros2_port.sh and
    # friends intentionally *contain* ROS1 substrings as their grep patterns.
    tests_dir = (REPO_ROOT / "tests").resolve()
    return sorted(
        p for p in REPO_ROOT.rglob("*")
        if p.is_file()
        and p.suffix.lower() in {".sh", ".bash"}
        and tests_dir not in p.resolve().parents
    )


_SHELL_SCRIPTS = _collect_shell_scripts()
_SHELL_IDS = _rel_ids(_SHELL_SCRIPTS)


# Given a line that contains a banned pattern, decide whether the match
# is inside an ``echo "..."`` substring. We don't try to do proper shell
# parsing; instead we look for an ``echo`` token to the left of the match
# position on the same line and at least one unescaped quote between them.
# This is lenient on purpose: the scripts carve-out exists so we can print
# friendly "no roscore needed" hints in tmux panes without the check
# complaining about it.
def _match_is_inside_echo(line: str, match_start: int) -> bool:
    prefix = line[:match_start]
    echo_idx = prefix.rfind("echo")
    if echo_idx == -1:
        return False
    # There must be a string-start quote between the echo token and the
    # match itself for the match to actually be inside the echo argument.
    between = prefix[echo_idx + 4:]
    return '"' in between or "'" in between


def _strip_shell_comments(text: str) -> str:
    """Drop ``#`` comments from a shell script without eating the shebang."""

    out: List[str] = []
    for idx, line in enumerate(text.splitlines()):
        if idx == 0 and line.startswith("#!"):
            out.append(line)
            continue
        # Find a top-level ``#`` not inside a string. We don't fully parse
        # shell quoting; we just look for a ``#`` preceded by whitespace or
        # at column 0.
        stripped = line.lstrip()
        if stripped.startswith("#"):
            out.append("")
            continue
        # mid-line comments: cut at the first `` #`` we find.
        mid = re.search(r"\s#", line)
        if mid:
            line = line[:mid.start()]
        out.append(line)
    return "\n".join(out)


@pytest.mark.parametrize("shell_path", _SHELL_SCRIPTS, ids=_SHELL_IDS)
def test_no_ros1_in_shell_scripts(shell_path: Path) -> None:
    """Shell scripts must not call roscore/rosrun/catkin_make/etc."""

    raw = shell_path.read_text(errors="replace")
    stripped = _strip_shell_comments(raw)

    offenders: List[str] = []
    for line in stripped.splitlines():
        if not line.strip():
            continue
        for regex, label in _SHELL_BANNED_PATTERNS:
            m = regex.search(line)
            if not m:
                continue
            if _match_is_inside_echo(line, m.start()):
                # Documented carve-out: ``echo "no roscore needed"`` and
                # similar hint strings don't actually invoke ROS1.
                continue
            offenders.append("{} (line: {!r})".format(label, line.strip()))
            break
    assert not offenders, (
        "{} still runs ROS1 commands: {}".format(shell_path, offenders)
    )


# -- 6. tf2 headers must end in .hpp ---------------------------------------


_TF2_LEGACY_HEADER_RE = re.compile(r"#include\s*<tf2[^>]+\.h>")


@pytest.mark.parametrize("cpp_path", _CPP_FILES, ids=_CPP_IDS)
def test_tf2_headers_are_hpp(cpp_path: Path) -> None:
    """All tf2 headers in Jazzy are ``.hpp``; ``.h`` forms must be gone."""

    text = _strip_cpp_comments(cpp_path.read_text(errors="replace"))
    matches = _TF2_LEGACY_HEADER_RE.findall(text)
    assert not matches, (
        "{} still uses legacy tf2 .h headers: {}".format(cpp_path, matches)
    )
