"""Flag duplicate ``declare_parameter("key", ...)`` calls within a package.

Motivation: in ROS2, calling ``node->declare_parameter("foo", ...)`` twice on
the same node throws ``rclcpp::exceptions::ParameterAlreadyDeclaredException``
at runtime. A recurring porting bug in this repo has been two source files in
the same package both declaring the same parameter, because they were split
out of what was originally one ROS1 nodelet. We catch that here statically.

What we check, per managed package:

- Walk every ``.cpp``/``.h``/``.hpp`` file in the package.
- Strip single-line ``//`` comments.
- Find every ``declare_parameter("name"`` / ``declare_parameter( "name"``,
  but *exclude* calls that are wrapped by a ``*_declare_or_get<T>(`` helper,
  which is the repo's known-safe pattern for "declare if missing, then read".
- Build a ``{parameter_name: [file1, file2, ...]}`` map and assert no key
  appears in more than one file within the same package.

We deliberately stop at the file level: a single file declaring the same
parameter twice is caught by the compiler / runtime anyway; the insidious
bug is the cross-file duplicate.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Set

import pytest

from _packages import MANAGED_PACKAGE_DIRS, REPO_ROOT


_CPP_SUFFIXES = {".cpp", ".h", ".hpp", ".cc"}


_CPP_LINE_COMMENT_RE = re.compile(r"//.*")

# Main match: declare_parameter("name" ... . We capture the name (double
# quotes only; ROS2 uses string literals, not raw char arrays).
_DECLARE_PARAM_RE = re.compile(r"declare_parameter\s*\(\s*\"([^\"]+)\"")

# If a declare_parameter(...) call appears inside a declare_or_get<T>(...)
# wrapper, we treat it as the safe form. Simplest heuristic: look backwards
# within ~120 characters for ``declare_or_get<``. If it's there, skip.
_DECLARE_OR_GET_WINDOW = 120


def _strip_cpp_comments(text: str) -> str:
    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _collect_cpp(package_dir: Path) -> List[Path]:
    return sorted(
        p for p in package_dir.rglob("*")
        if p.is_file() and p.suffix.lower() in _CPP_SUFFIXES
    )


def _find_declared_params(text: str) -> Set[str]:
    """Return every non-wrapper declare_parameter("X") name in the text."""

    names: Set[str] = set()
    for match in _DECLARE_PARAM_RE.finditer(text):
        start = match.start()
        window_start = max(0, start - _DECLARE_OR_GET_WINDOW)
        context = text[window_start:start]
        if "declare_or_get<" in context:
            # Safe wrapper form; don't count.
            continue
        names.add(match.group(1))
    return names


_PACKAGE_IDS = [relpath.replace("\\", "/") for relpath, _ in MANAGED_PACKAGE_DIRS]
_PACKAGE_DIRS = [REPO_ROOT / relpath for relpath, _ in MANAGED_PACKAGE_DIRS]


@pytest.mark.parametrize("package_dir", _PACKAGE_DIRS, ids=_PACKAGE_IDS)
def test_no_duplicate_declare_parameter(package_dir: Path) -> None:
    """No ``declare_parameter("x")`` key may appear in 2+ files of a package."""

    if not package_dir.is_dir():
        pytest.skip("{} not checked out".format(package_dir))

    # Map each parameter key to the list of files that declare it.
    occurrences: Dict[str, List[Path]] = {}
    for cpp_path in _collect_cpp(package_dir):
        text = _strip_cpp_comments(cpp_path.read_text(errors="replace"))
        for name in _find_declared_params(text):
            occurrences.setdefault(name, []).append(cpp_path)

    duplicates = {
        key: [str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in paths]
        for key, paths in occurrences.items()
        if len(paths) >= 2
    }
    assert not duplicates, (
        "{} has parameter keys declared in multiple files: {}"
        .format(package_dir.name, duplicates)
    )
