"""Static checks on every ``.action`` file in the repo.

ROS2 action definitions are stricter than ROS1's: ``Header`` must be
explicitly namespaced as ``std_msgs/Header``, raw ``time``/``duration``
types are gone (use ``builtin_interfaces/Time``/``Duration``), and field
names must be snake_case. A surprising amount of the pain during a
ROS1->ROS2 port is hit at interface generation time, so catching these
early is worth the 30-line test.

For each ``.action`` file we assert:

1. Exactly two ``---`` separator lines (three sections: goal, result,
   feedback).
2. No line in any section starts with bare ``Header `` -- it must be
   ``std_msgs/Header``.
3. No line starts with bare ``time `` or ``duration `` -- must be
   ``builtin_interfaces/Time`` / ``builtin_interfaces/Duration``.
4. Every field name (first whitespace-separated token after the type)
   matches the ROS2 ``^[a-z][a-z0-9_]*$`` snake_case regex.

Lines that are empty or are comments (``#``) are ignored.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import List

import pytest

from _packages import REPO_ROOT


_SNAKE_CASE_RE = re.compile(r"^[a-z][a-z0-9_]*$")

# Anything starting with one of these type tokens (case-sensitive) is
# considered an error. Use ``\b`` to avoid matching prefixes like
# ``Headerless``.
_BANNED_BARE_HEADER_RE = re.compile(r"^Header\b")
_BANNED_BARE_TIME_RE = re.compile(r"^time\b")
_BANNED_BARE_DURATION_RE = re.compile(r"^duration\b")


def _collect_action_files() -> List[Path]:
    return sorted(REPO_ROOT.rglob("*.action"))


_ACTION_FILES = _collect_action_files()
_ACTION_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _ACTION_FILES
]


def _strip_field_line(line: str) -> str:
    """Return the field line with the trailing inline comment removed."""

    comment_idx = line.find("#")
    if comment_idx >= 0:
        line = line[:comment_idx]
    return line.rstrip()


@pytest.mark.parametrize("action_path", _ACTION_FILES, ids=_ACTION_IDS)
def test_action_has_three_sections(action_path: Path) -> None:
    """Goal/Result/Feedback sections must be separated by exactly two ``---`` lines."""

    text = action_path.read_text(errors="replace")
    separator_lines = [ln for ln in text.splitlines() if ln.strip() == "---"]
    assert len(separator_lines) == 2, (
        "{} has {} '---' separators; want exactly 2 (goal/result/feedback)"
        .format(action_path, len(separator_lines))
    )


@pytest.mark.parametrize("action_path", _ACTION_FILES, ids=_ACTION_IDS)
def test_action_has_no_bare_header(action_path: Path) -> None:
    """``Header`` must be namespaced as ``std_msgs/Header``."""

    text = action_path.read_text(errors="replace")
    offenders: List[str] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = _strip_field_line(raw).lstrip()
        if not line or line.startswith("#") or line == "---":
            continue
        if _BANNED_BARE_HEADER_RE.match(line):
            offenders.append("line {}: {!r}".format(lineno, raw))
    assert not offenders, (
        "{} has bare Header (must be std_msgs/Header): {}"
        .format(action_path, offenders)
    )


@pytest.mark.parametrize("action_path", _ACTION_FILES, ids=_ACTION_IDS)
def test_action_has_no_bare_time_or_duration(action_path: Path) -> None:
    """Raw ``time`` / ``duration`` are ROS1; use builtin_interfaces/*."""

    text = action_path.read_text(errors="replace")
    offenders: List[str] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = _strip_field_line(raw).lstrip()
        if not line or line.startswith("#") or line == "---":
            continue
        if _BANNED_BARE_TIME_RE.match(line) or _BANNED_BARE_DURATION_RE.match(line):
            offenders.append("line {}: {!r}".format(lineno, raw))
    assert not offenders, (
        "{} has bare time/duration (must be builtin_interfaces/*): {}"
        .format(action_path, offenders)
    )


@pytest.mark.parametrize("action_path", _ACTION_FILES, ids=_ACTION_IDS)
def test_action_field_names_are_snake_case(action_path: Path) -> None:
    """ROS2 enforces snake_case; catch mixedCase and PascalCase fields.

    Constants (``int32 MY_CONST = 42``) are allowed to be UPPER_SNAKE.
    We decide whether a line is a constant-declaration by looking for an
    ``=`` anywhere in the line (because the ``=`` can be separated from
    the name by any amount of whitespace).
    """

    text = action_path.read_text(errors="replace")
    offenders: List[str] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = _strip_field_line(raw).lstrip()
        if not line or line.startswith("#") or line == "---":
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        type_token, name_token = parts[0], parts[1]
        # Constant declaration (``<type> <NAME> = <value>`` OR
        # ``<type> <NAME>=<value>``): names may be UPPER_SNAKE.
        if "=" in line:
            # Strip any trailing ``=value`` glued to the name token.
            name_token = name_token.split("=", 1)[0]
            if not name_token or re.match(r"^[A-Z][A-Z0-9_]*$", name_token):
                continue
        if not _SNAKE_CASE_RE.match(name_token):
            offenders.append(
                "line {}: type {!r}, field {!r}".format(lineno, type_token, name_token)
            )
    assert not offenders, (
        "{} has non-snake_case field names: {}".format(action_path, offenders)
    )
