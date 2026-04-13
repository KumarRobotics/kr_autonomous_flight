"""Static checks on every managed package's CMakeLists.txt.

The goal is to catch half-ported CMake files -- the kind where someone updated
the package.xml to ament_cmake but forgot to strip the catkin boilerplate
underneath, so the package "builds" under colcon but actually silently skips
half of its targets.

For each CMakeLists.txt in a managed package directory we assert:

1. ``ament_package()`` is called (substring match; we don't tokenize CMake).
2. None of the following ROS1-era forms appear:
     - ``find_package(catkin``
     - ``catkin_package(``
     - ``${catkin_INCLUDE_DIRS}``
     - ``${catkin_LIBRARIES}``
     - ``add_message_files(``
     - ``add_service_files(``
     - ``add_action_files(``
     - ``generate_messages(``
3. Every path token inside an ``install(PROGRAMS ... DESTINATION ...)`` block
   exists on disk, resolved relative to the CMakeLists' parent directory.
   Tokens that contain ``${...}`` variable references are skipped because we
   don't have a CMake interpreter handy at test time.

The tests are parametrised per-CMakeLists so that a failure names the file
directly rather than producing a wall of "for loop at line 42".
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import List

import pytest

from _packages import MANAGED_PACKAGE_DIRS, REPO_ROOT


def _collect_cmakelists() -> List[Path]:
    """Every CMakeLists.txt that sits at the root of a managed package."""

    paths: List[Path] = []
    for relpath, _ in MANAGED_PACKAGE_DIRS:
        cml = REPO_ROOT / relpath / "CMakeLists.txt"
        if cml.is_file():
            paths.append(cml)
    return paths


_CMAKELISTS = _collect_cmakelists()
_CMAKELISTS_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _CMAKELISTS
]


_BANNED_SUBSTRINGS = (
    "find_package(catkin",
    "catkin_package(",
    "${catkin_INCLUDE_DIRS}",
    "${catkin_LIBRARIES}",
    "add_message_files(",
    "add_service_files(",
    "add_action_files(",
    "generate_messages(",
)


# Regex that finds the body of an ``install(PROGRAMS ... DESTINATION ...)``
# block. We use DOTALL so the body can span newlines; the body match is
# lazy so we don't greedily eat multiple install() calls.
_INSTALL_PROGRAMS_RE = re.compile(
    r"install\s*\(\s*PROGRAMS\b(?P<body>.*?)\)",
    re.DOTALL | re.IGNORECASE,
)


@pytest.mark.parametrize("cmake_path", _CMAKELISTS, ids=_CMAKELISTS_IDS)
def test_cmakelists_calls_ament_package(cmake_path: Path) -> None:
    """``ament_package()`` must appear in every managed CMakeLists.txt."""

    text = cmake_path.read_text(errors="replace")
    assert "ament_package()" in text, (
        "{} does not call ament_package(); colcon will not register it as a "
        "ROS2 package".format(cmake_path)
    )


@pytest.mark.parametrize("cmake_path", _CMAKELISTS, ids=_CMAKELISTS_IDS)
def test_cmakelists_has_no_catkin_leftovers(cmake_path: Path) -> None:
    """No banned ROS1 substrings may appear in a managed CMakeLists.txt."""

    text = cmake_path.read_text(errors="replace")
    offenders = [s for s in _BANNED_SUBSTRINGS if s in text]
    assert not offenders, (
        "{} still contains ROS1-era CMake: {}".format(cmake_path, offenders)
    )


@pytest.mark.parametrize("cmake_path", _CMAKELISTS, ids=_CMAKELISTS_IDS)
def test_cmakelists_install_programs_targets_exist(cmake_path: Path) -> None:
    """Each path in ``install(PROGRAMS ...)`` must resolve to an existing file.

    This catches the common footgun where a developer adds a Python launcher
    to install(PROGRAMS) but forgets to ``chmod +x`` or renames the source
    and doesn't update CMakeLists -- colcon will happily no-op, and you only
    find out at runtime when ``ros2 run pkg bad.py`` dies.
    """

    text = cmake_path.read_text(errors="replace")
    parent = cmake_path.parent
    missing: List[str] = []

    for match in _INSTALL_PROGRAMS_RE.finditer(text):
        body = match.group("body")

        # Cut the body at ``DESTINATION`` so we don't try to resolve the
        # install target dir as a source path.
        destination_idx = body.upper().find("DESTINATION")
        if destination_idx >= 0:
            body = body[:destination_idx]

        for raw_token in body.split():
            token = raw_token.strip().strip(",").strip('"').strip("'")
            if not token:
                continue
            if "${" in token:
                # Can't resolve CMake variables statically.
                continue
            # Skip CMake keywords that may appear inside the block.
            upper = token.upper()
            if upper in {"PROGRAMS", "OPTIONAL", "RENAME"}:
                continue
            candidate = (parent / token).resolve()
            if not candidate.exists():
                missing.append(token)

    assert not missing, (
        "{} install(PROGRAMS ...) references missing files: {}"
        .format(cmake_path, missing)
    )
