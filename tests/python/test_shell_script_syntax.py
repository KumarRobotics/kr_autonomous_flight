"""Shell script ``bash -n`` syntax sweep.

For every ``.sh`` / ``.bash`` under ``autonomy_core``, ``autonomy_real``,
``autonomy_sim``, and ``tests``, shell out to ``bash -n`` to confirm it
parses. A syntactically-invalid shell script would fail at first run and is
a trivial bug to catch statically.

If ``bash`` is not available on the host (common on Windows CI without
WSL), the test is skipped rather than failed -- the goal is a best-effort
static sanity check, not a mandatory gate on every platform.

Mirrors Section Q of ``tests/static/check_ros2_port.sh``.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import List

import pytest

from _packages import REPO_ROOT


def _collect_shell_scripts() -> List[Path]:
    out: List[Path] = []
    for sub in ("autonomy_core", "autonomy_real", "autonomy_sim", "tests"):
        base = REPO_ROOT / sub
        if not base.is_dir():
            continue
        for ext in ("*.sh", "*.bash"):
            out.extend(base.rglob(ext))
    return sorted(out)


_SHELL_SCRIPTS = _collect_shell_scripts()
_SHELL_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _SHELL_SCRIPTS
]


_BASH = shutil.which("bash")


@pytest.mark.parametrize("shell_script", _SHELL_SCRIPTS, ids=_SHELL_IDS)
def test_bash_n_parses(shell_script: Path) -> None:
    """``bash -n`` must accept every shipped shell script."""

    if _BASH is None:
        pytest.skip("bash not available on PATH; shell syntax check skipped")
        return
    try:
        result = subprocess.run(
            [_BASH, "-n", str(shell_script)],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (subprocess.TimeoutExpired, OSError) as exc:
        pytest.fail("bash -n failed to execute on {0}: {1}".format(shell_script, exc))
        return
    if result.returncode != 0:
        pytest.fail(
            "bash -n reported a syntax error in {0}:\n{1}".format(
                shell_script, result.stderr.strip() or "<no stderr>"
            )
        )
