"""Shared pytest fixtures for the kr_autonomous_flight static-analysis suite.

Fixture values are thin wrappers around the constants in ``_packages.py``.
The reason the constants live in a sibling module rather than in this
conftest: pytest discovers conftest.py specially, which makes
``from conftest import ...`` brittle across pytest releases. Plain modules
next to the tests are always importable.

Fixtures provided:

- ``repo_root``            -- absolute pathlib.Path of the repo root.
- ``packages``             -- list of ``(relpath, abspath, package_name)``
                              tuples for the 22 managed packages. Only
                              entries whose directory exists on disk are
                              returned, so the suite degrades gracefully if
                              a package is being renamed.
- ``managed_package_names``-- set of ``<name>`` values from every managed
                              package's package.xml.
- ``package_xml_files``    -- every package.xml reachable from the repo
                              root.
- ``cmakelists_files``     -- every CMakeLists.txt sitting at the root of a
                              managed package.
- ``exempt_paths``         -- absolute paths that grep-style checks must
                              not flag (currently the four
                              ``action_trackers`` tracker-plugin source
                              files, which legitimately use
                              ``PLUGINLIB_EXPORT_CLASS``).

No ROS2 Python runtime is imported; the suite is deliberately pure-stdlib so
it can run inside any CI container that ships Python 3 and pytest.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

import pytest

# Ensure the directory this conftest lives in is on sys.path so that test
# modules can ``from _packages import ...``. Pytest normally does this via
# rootdir discovery, but being explicit costs nothing and sidesteps the
# "works on my machine" surprises when users invoke pytest from odd cwds.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from _packages import (  # noqa: E402  (sys.path tweak must happen first)
    EXEMPT_PLUGIN_CPP,
    MANAGED_PACKAGE_DIRS,
    REPO_ROOT,
)


@pytest.fixture(scope="session")
def repo_root() -> Path:
    """Absolute path to the repository root."""

    return REPO_ROOT


@pytest.fixture(scope="session")
def packages() -> List[Tuple[str, Path, str]]:
    """List of managed packages as ``(dir_relpath, dir_abspath, package_name)``.

    Only directories that actually exist on disk are returned; this lets the
    fixture degrade gracefully if a package is deleted or renamed without the
    test suite being updated in the same commit.
    """

    result: List[Tuple[str, Path, str]] = []
    for relpath, pkg_name in MANAGED_PACKAGE_DIRS:
        abspath = REPO_ROOT / relpath
        if abspath.is_dir():
            result.append((relpath, abspath, pkg_name))
    return result


@pytest.fixture(scope="session")
def managed_package_names() -> set:
    """Set of package-xml ``<name>`` values for every managed package."""

    return {pkg_name for _, pkg_name in MANAGED_PACKAGE_DIRS}


@pytest.fixture(scope="session")
def package_xml_files() -> List[Path]:
    """Every package.xml reachable from the repo root."""

    return sorted(REPO_ROOT.rglob("package.xml"))


@pytest.fixture(scope="session")
def cmakelists_files() -> List[Path]:
    """Every CMakeLists.txt that sits alongside a managed package.xml."""

    result: List[Path] = []
    for relpath, _ in MANAGED_PACKAGE_DIRS:
        cml = REPO_ROOT / relpath / "CMakeLists.txt"
        if cml.is_file():
            result.append(cml)
    return result


@pytest.fixture(scope="session")
def exempt_paths() -> set:
    """Absolute paths that grep-style checks are allowed to skip.

    Currently this only contains the four ``action_trackers`` plugin source
    files, which legitimately carry the ``PLUGINLIB_EXPORT_CLASS`` macro --
    that's the intended ROS2 pluginlib usage pattern, not a leftover.
    """

    return set(EXEMPT_PLUGIN_CPP)


# --- helpers --------------------------------------------------------------


def read_text_safe(path: Path) -> str:
    """Read a text file tolerantly; returns an empty string on any failure."""

    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def package_name_from_xml(xml_path: Path) -> str:
    """Parse a package.xml and return its ``<name>`` child, or ``""``."""

    try:
        tree = ET.parse(xml_path)
    except ET.ParseError:
        return ""
    name_el = tree.getroot().find("name")
    if name_el is None or name_el.text is None:
        return ""
    return name_el.text.strip()
