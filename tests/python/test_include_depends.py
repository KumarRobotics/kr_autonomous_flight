"""Cross-package ``#include`` to ``<depend>`` consistency check.

For every C/C++ source file in every managed package, extract every
``#include <pkg/...>`` or ``#include "pkg/..."`` top-level component, find
the owning ``package.xml``, and verify every non-exempt ``pkg`` is listed in
the package.xml's ``<depend>`` / ``<build_depend>`` / ``<exec_depend>`` /
``<test_depend>`` / ``<buildtool_depend>`` tags.

Exempt names are things found via ``find_package()`` + ``target_link_libraries``
rather than via ament dependency resolution -- the C++ standard library,
Eigen, Boost, PCL, OpenCV, etc.  Self-includes (``pkg`` equals the owning
package's own name) are also skipped.

Mirrors Section O of ``tests/static/check_ros2_port.sh``.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Set, Tuple

import pytest

from _packages import MANAGED_PACKAGE_DIRS, REPO_ROOT


_CPP_SUFFIXES = {".cpp", ".cc", ".h", ".hpp"}

# Top-level pkg extraction: captures whatever appears after the opening
# ``<`` / ``"`` up to the first ``/``.  A slash is required, which filters
# out C++ standard-library-style single-token angle-bracket includes
# (``<string>``, ``<vector>``, ...).
_INCLUDE_RE = re.compile(
    r'^\s*#include\s*[<"]([A-Za-z_][A-Za-z0-9_]*)/',
    re.MULTILINE,
)

_CPP_LINE_COMMENT_RE = re.compile(r"//.*")


# Names that are pulled via ``find_package(...)`` + ``target_link_libraries``
# (or the C++ standard library). They must NOT appear in ``<depend>``.
EXEMPT_INCLUDE_PREFIXES: Set[str] = {
    # System C++ / C libraries
    "Eigen", "eigen3", "boost", "gtsam", "pcl", "opencv2", "OpenCV", "fmt",
    "yaml-cpp", "glog", "tbb", "gtest", "benchmark", "sys", "arpa", "netinet",
    "linux", "fcntl", "unistd", "pthread", "signal", "errno", "locale",
    "stddef", "stdio", "stdlib", "string", "time", "wchar", "wctype",
    # C++ stdlib headers that happen to have a slash
    "math", "chrono", "memory", "vector", "map", "set", "iostream", "fstream",
    "sstream", "cstdlib", "cstring", "algorithm", "functional", "utility",
    "tuple", "optional", "thread", "mutex", "atomic", "condition_variable",
    "future", "filesystem", "random", "regex", "typeinfo", "cassert", "cmath",
    "cstddef", "cstdint", "cstdio", "limits", "numeric", "queue", "stack",
    "deque", "list", "array", "bitset", "complex", "exception", "stdexcept",
    "system_error", "type_traits", "cctype", "iomanip", "cstdarg", "cfloat",
    "climits", "ctime", "cerrno", "csetjmp", "csignal", "ciso646", "cstdbool",
    "ctgmath", "cuchar", "cwchar", "cwctype", "rcutils", "rclcpp_lifecycle",
    # Known external include-prefix paths whose owning ROS package name is
    # DIFFERENT from the top-level include dir (e.g. plan_manage ships inside
    # a bigger opt_planner / ego_planner package).
    "plan_manage", "traj_utils",
}


def _strip_cpp_comments(text: str) -> str:
    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _parse_depends(xml_path: Path) -> Set[str]:
    """Return every ``<depend>``-family value from a package.xml."""

    out: Set[str] = set()
    try:
        tree = ET.parse(xml_path)
    except (ET.ParseError, OSError):
        return out
    root = tree.getroot()
    tags = (
        "depend", "build_depend", "exec_depend", "test_depend",
        "buildtool_depend", "build_export_depend", "run_depend",
    )
    for tag in tags:
        for el in root.iter(tag):
            if el.text:
                out.add(el.text.strip())
    return out


def _package_name(xml_path: Path) -> str:
    try:
        tree = ET.parse(xml_path)
    except (ET.ParseError, OSError):
        return ""
    name_el = tree.getroot().find("name")
    return (name_el.text or "").strip() if name_el is not None else ""


def _collect_cpp_files(package_dir: Path) -> List[Path]:
    return sorted(
        p for p in package_dir.rglob("*")
        if p.is_file() and p.suffix.lower() in _CPP_SUFFIXES
    )


def _build_include_to_package_map(managed_dirs) -> Dict[str, str]:
    """Walk every managed package's ``include/*`` subdirs and record the
    top-level directory name -> owning package.xml ``<name>`` mapping.

    Example: ``motion_primitive_library`` exports headers under
    ``mpl_collision/``, ``mpl_basis/``, ``mpl_planner/`` -- so the returned
    dict maps ``mpl_collision`` to ``motion_primitive_library``.
    """

    mapping: Dict[str, str] = {}
    for relpath, _ in managed_dirs:
        pkg_dir = REPO_ROOT / relpath
        if not pkg_dir.is_dir():
            continue
        pkg_xml = pkg_dir / "package.xml"
        if not pkg_xml.is_file():
            continue
        name = _package_name(pkg_xml)
        if not name:
            continue
        include = pkg_dir / "include"
        if not include.is_dir():
            continue
        for sub in include.iterdir():
            if sub.is_dir():
                mapping[sub.name] = name
    return mapping


_INCLUDE_TO_PKG = _build_include_to_package_map(MANAGED_PACKAGE_DIRS)


def _extract_includes(text: str) -> Set[str]:
    stripped = _strip_cpp_comments(text)
    return set(_INCLUDE_RE.findall(stripped))


_PARAM_IDS = [relpath.replace("\\", "/") for relpath, _ in MANAGED_PACKAGE_DIRS]
_PARAM_VALUES: List[Tuple[Path, str]] = [
    (REPO_ROOT / relpath, pkg) for relpath, pkg in MANAGED_PACKAGE_DIRS
]


@pytest.mark.parametrize("package_dir,package_name", _PARAM_VALUES, ids=_PARAM_IDS)
def test_includes_match_depends(package_dir: Path, package_name: str) -> None:
    """Every ``#include <pkg/...>`` in this package's sources is in package.xml."""

    if not package_dir.is_dir():
        pytest.skip("package directory does not exist: {}".format(package_dir))

    pkg_xml = package_dir / "package.xml"
    if not pkg_xml.is_file():
        pytest.skip("no package.xml under {}".format(package_dir))

    deps = _parse_depends(pkg_xml)
    owning_name = _package_name(pkg_xml) or package_name

    missing: Dict[str, List[str]] = {}
    for src in _collect_cpp_files(package_dir):
        try:
            text = src.read_text(errors="replace")
        except OSError:
            continue
        for pkg in _extract_includes(text):
            if pkg in EXEMPT_INCLUDE_PREFIXES:
                continue
            if pkg == owning_name:
                continue
            # If the include top-dir maps to an in-tree package, look for
            # that package name in the depend list instead of the raw path.
            resolved = _INCLUDE_TO_PKG.get(pkg, pkg)
            if resolved == owning_name:
                continue
            if resolved in deps or pkg in deps:
                continue
            key = "{0} (->{1})".format(pkg, resolved) if resolved != pkg else pkg
            missing.setdefault(key, []).append(
                str(src.relative_to(REPO_ROOT)).replace("\\", "/")
            )

    if missing:
        lines = ["#include <{0}/...> not declared in package.xml:".format(k)
                 + "\n    " + "\n    ".join(sorted(set(v)))
                 for k, v in sorted(missing.items())]
        pytest.fail(
            "{0}: {1} missing <depend> entr(y/ies)\n  ".format(owning_name, len(missing))
            + "\n  ".join(lines)
        )
