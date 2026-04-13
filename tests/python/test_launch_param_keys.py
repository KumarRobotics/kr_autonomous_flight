"""Launch dict-parameter keys vs ``declare_parameter()`` cross-reference.

For every ``Node(package=P, executable=E, parameters=[{...}])`` in a launch
file, verify every literal dict key is declared in package ``P``'s source
tree via either ``declare_parameter("key", ...)``, the repo's
``get_param_or(node, "key", ...)`` wrapper, or a ``declare_parameter_if_not_declared``
helper.  A missing declaration is a silent-no-op bug at runtime: ROS2 ignores
un-declared parameters passed through launch and the node sees only its C++
defaults.

External packages (``package=`` not in the managed list) and non-dict
parameter entries (e.g. ``parameters=[yaml_path]``) are skipped.

Mirrors Section P of ``tests/static/check_ros2_port.sh``.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Set, Tuple

import pytest

from _packages import (
    MANAGED_PACKAGE_DIRS,
    MANAGED_PKG_NAMES,
    PKG_NAME_TO_DIR,
    REPO_ROOT,
)


_CPP_SUFFIXES = {".cpp", ".cc", ".h", ".hpp"}
_PY_SUFFIXES = {".py"}
_CPP_LINE_COMMENT_RE = re.compile(r"//.*")


# In the flattened launch source, find every ``Node(...)`` block whose ``(``
# is not preceded by a word character (excludes ComposableNode /
# LifecycleNode).  We stop at the first ``)``, which is good enough for the
# repo's launch style.
_NODE_BLOCK_RE = re.compile(r"(?:^|[^A-Za-z0-9_])Node\s*\(([^)]{0,2000})\)")

# Dict-key extraction inside a ``parameters=[...]`` list.  We need to match:
#   'key': value      "key": value
# and skip substitution values (which aren't followed by ``:``).
_DICT_KEY_RE = re.compile(r"['\"]([A-Za-z_][A-Za-z0-9_.]*)['\"]\s*:")

_PACKAGE_RE = re.compile(r"package\s*=\s*['\"]([^'\"]+)['\"]")
_EXECUTABLE_RE = re.compile(r"executable\s*=\s*['\"]([^'\"]+)['\"]")
_PARAMETERS_RE = re.compile(r"parameters\s*=\s*\[([^\]]*)\]")


# In source, find every ``declare_parameter("key"...)`` (with optional
# template argument), ``get_param_or(node, "key", ...)``, or
# ``declare_parameter_if_not_declared(node, "key", ...)``.
_DECLARE_CPP_RE = re.compile(
    r'declare_parameter\s*(?:<[^>]*>)?\s*\(\s*"([^"]+)"',
)
_GET_PARAM_OR_RE = re.compile(
    r'\bget_param_or\s*\(\s*[A-Za-z_][A-Za-z0-9_>.]*\s*,\s*"([^"]+)"',
)
_DECL_IF_NOT_RE = re.compile(
    r'\bdeclare_parameter_if_not_declared\s*\(\s*[A-Za-z_][A-Za-z0-9_>.]*\s*,\s*"([^"]+)"',
)
_DECLARE_PY_RE = re.compile(
    r'declare_parameter\s*\(\s*[\'"]([^\'"]+)[\'"]',
)


def _strip_cpp_comments(text: str) -> str:
    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _strip_py_comments(text: str) -> str:
    # Very loose: strip the trailing ``# ...`` from each line outside quotes.
    # We don't try to be clever about string literals; in practice
    # ``declare_parameter("#foo", ...)`` doesn't exist in this repo.
    lines = []
    for line in text.splitlines():
        in_s = False
        in_d = False
        out = []
        for ch in line:
            if ch == "#" and not in_s and not in_d:
                break
            if ch == "'" and not in_d:
                in_s = not in_s
            elif ch == '"' and not in_s:
                in_d = not in_d
            out.append(ch)
        lines.append("".join(out))
    return "\n".join(lines)


def _collect_declared_keys(pkg_dir: Path) -> Set[str]:
    keys: Set[str] = set()
    for src in pkg_dir.rglob("*"):
        if not src.is_file():
            continue
        suffix = src.suffix.lower()
        try:
            text = src.read_text(errors="replace")
        except OSError:
            continue
        if suffix in _CPP_SUFFIXES:
            stripped = _strip_cpp_comments(text)
            keys.update(_DECLARE_CPP_RE.findall(stripped))
            keys.update(_GET_PARAM_OR_RE.findall(stripped))
            keys.update(_DECL_IF_NOT_RE.findall(stripped))
        elif suffix in _PY_SUFFIXES:
            stripped = _strip_py_comments(text)
            keys.update(_DECLARE_PY_RE.findall(stripped))
    return keys


def _launch_files() -> List[Path]:
    out: List[Path] = []
    for sub in ("autonomy_core", "autonomy_real", "autonomy_sim"):
        base = REPO_ROOT / sub
        if not base.is_dir():
            continue
        out.extend(sorted(base.rglob("*.launch.py")))
    return out


_LAUNCH_FILES = _launch_files()
_LAUNCH_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _LAUNCH_FILES
]


_DECLARED_CACHE: Dict[str, Set[str]] = {}


def _declared_keys_for(pkg_name: str) -> Set[str]:
    if pkg_name in _DECLARED_CACHE:
        return _DECLARED_CACHE[pkg_name]
    pkg_dir = PKG_NAME_TO_DIR.get(pkg_name)
    if pkg_dir is None or not pkg_dir.is_dir():
        _DECLARED_CACHE[pkg_name] = set()
    else:
        _DECLARED_CACHE[pkg_name] = _collect_declared_keys(pkg_dir)
    return _DECLARED_CACHE[pkg_name]


@pytest.mark.parametrize("launch_file", _LAUNCH_FILES, ids=_LAUNCH_IDS)
def test_launch_param_keys(launch_file: Path) -> None:
    """Every dict key in ``parameters=[{...}]`` is declared in the target pkg."""

    try:
        raw = launch_file.read_text(errors="replace")
    except OSError:
        pytest.skip("could not read {}".format(launch_file))
        return

    flat = raw.replace("\n", " ")
    problems: List[str] = []
    nodes_seen = 0

    for match in _NODE_BLOCK_RE.finditer(flat):
        block = match.group(1)
        pkg_m = _PACKAGE_RE.search(block)
        if not pkg_m:
            continue
        pkg = pkg_m.group(1)
        if pkg not in MANAGED_PKG_NAMES:
            continue
        nodes_seen += 1
        exe_m = _EXECUTABLE_RE.search(block)
        exe = exe_m.group(1) if exe_m else "<unknown>"
        plist_m = _PARAMETERS_RE.search(block)
        if not plist_m:
            continue
        plist = plist_m.group(1)
        keys = set(_DICT_KEY_RE.findall(plist))
        if not keys:
            continue
        declared = _declared_keys_for(pkg)
        for key in sorted(keys):
            if key.startswith("/") or key.endswith((".yaml", ".yml")):
                continue
            if key not in declared:
                problems.append(
                    "Node(package={pkg!r}, executable={exe!r}) parameters=[{{'{k}': ...}}]"
                    " but no declare_parameter(\"{k}\", ...) found in {pkg} source".format(
                        pkg=pkg, exe=exe, k=key,
                    )
                )

    if problems:
        pytest.fail(
            "{0}: {1} undeclared launch parameter key(s) ({2} Node calls checked)\n  ".format(
                launch_file.relative_to(REPO_ROOT), len(problems), nodes_seen,
            )
            + "\n  ".join(problems)
        )
