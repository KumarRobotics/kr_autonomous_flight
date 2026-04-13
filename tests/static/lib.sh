#!/usr/bin/env bash
# lib.sh - shared helpers for the kr_autonomous_flight ROS2 port static test suite
#
# Provides:
#   - ANSI color codes (with NO_COLOR + non-tty fallback)
#   - PASS/FAIL/SKIPPED counters
#   - `section`, `pass`, `fail`, `skip`, `print_summary` helpers
#   - `strip_cxx_comments`, `strip_py_comments` awk helpers
#   - `list_cxx_files`, `list_py_files`, `list_launch_py_files`, `list_pkgs_with_cmakelists`
#
# Sourced by tests/static/check_ros2_port.sh. Not intended for direct execution.

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
if [[ -n "${NO_COLOR:-}" ]] || ! { [[ -t 1 ]] || tty -s 2>/dev/null; }; then
  C_RED=""
  C_GREEN=""
  C_YELLOW=""
  C_BLUE=""
  C_BOLD=""
  C_RESET=""
else
  C_RED=$'\033[31m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
  C_BLUE=$'\033[34m'
  C_BOLD=$'\033[1m'
  C_RESET=$'\033[0m'
fi

# ---------------------------------------------------------------------------
# Counters
# ---------------------------------------------------------------------------
PASSED=0
FAILED=0
SKIPPED=0
WARNED=0
TOTAL=0

# list of failing section labels, used for final summary
FAILED_SECTIONS=()

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
section() {
  local label="$1"
  printf "\n%s==> %s%s\n" "${C_BOLD}${C_BLUE}" "$label" "${C_RESET}"
}

pass() {
  local msg="$1"
  PASSED=$((PASSED + 1))
  TOTAL=$((TOTAL + 1))
  printf "  %s[PASS]%s %s\n" "${C_GREEN}" "${C_RESET}" "$msg"
}

fail() {
  local msg="$1"
  FAILED=$((FAILED + 1))
  TOTAL=$((TOTAL + 1))
  FAILED_SECTIONS+=("$msg")
  printf "  %s[FAIL]%s %s\n" "${C_RED}" "${C_RESET}" "$msg"
}

skip() {
  local msg="$1"
  SKIPPED=$((SKIPPED + 1))
  TOTAL=$((TOTAL + 1))
  printf "  %s[SKIP]%s %s\n" "${C_YELLOW}" "${C_RESET}" "$msg"
}

warn() {
  local msg="$1"
  WARNED=$((WARNED + 1))
  printf "  %s[WARN]%s %s\n" "${C_YELLOW}" "${C_RESET}" "$msg"
}

# Dump at most N lines from stdin, indented, only if VERBOSE=1.
# Usage: some_command | verbose_dump 10
verbose_dump() {
  local max="${1:-10}"
  if [[ "${VERBOSE:-0}" != "1" ]]; then
    # drain stdin so callers don't block
    cat >/dev/null
    return
  fi
  local count=0
  while IFS= read -r line; do
    count=$((count + 1))
    if [[ $count -le $max ]]; then
      printf "         %s\n" "$line"
    fi
  done
  if [[ $count -gt $max ]]; then
    printf "         ... (%d more)\n" $((count - max))
  fi
}

print_summary() {
  local color="$C_GREEN"
  [[ $FAILED -gt 0 ]] && color="$C_RED"
  printf "\n%s%s==========================================%s\n" "$C_BOLD" "$color" "$C_RESET"
  printf "%s%s%d checks, %d passed, %d failed, %d skipped%s\n" \
    "$C_BOLD" "$color" "$TOTAL" "$PASSED" "$FAILED" "$SKIPPED" "$C_RESET"
  if [[ $WARNED -gt 0 ]]; then
    printf "%s%d warnings (non-fatal)%s\n" "$C_YELLOW" "$WARNED" "$C_RESET"
  fi
  printf "%s%s==========================================%s\n" "$C_BOLD" "$color" "$C_RESET"
}

# ---------------------------------------------------------------------------
# Comment strippers (awk streaming; single-line // and # comments only)
# ---------------------------------------------------------------------------
# Strip C++ single-line comments (`//...` to end of line). Handles a quoted-string
# heuristic: if a `//` is inside a `"..."` we leave it alone. Not a full C++
# lexer, but sufficient for grepping active code.
strip_cxx_comments() {
  awk '
  {
    out = ""
    i = 1
    in_str = 0
    n = length($0)
    while (i <= n) {
      c = substr($0, i, 1)
      c2 = (i < n) ? substr($0, i, 2) : ""
      if (in_str) {
        out = out c
        if (c == "\\" && i < n) {
          out = out substr($0, i+1, 1)
          i += 2
          continue
        }
        if (c == "\"") in_str = 0
        i++
        continue
      }
      if (c == "\"") { in_str = 1; out = out c; i++; continue }
      if (c2 == "//") break
      out = out c
      i++
    }
    print out
  }'
}

# Strip Python single-line comments (`#...` to end of line). Ignores `#` inside
# `"..."` or `'...'` strings.
strip_py_comments() {
  awk '
  BEGIN { SQ = sprintf("%c", 39) }
  {
    out = ""
    i = 1
    q = 0
    n = length($0)
    while (i <= n) {
      c = substr($0, i, 1)
      if (q != 0) {
        out = out c
        if (c == "\\" && i < n) {
          out = out substr($0, i+1, 1)
          i += 2
          continue
        }
        if ((q == 1 && c == "\"") || (q == 2 && c == SQ)) q = 0
        i++
        continue
      }
      if (c == "\"") { q = 1; out = out c; i++; continue }
      if (c == SQ) { q = 2; out = out c; i++; continue }
      if (c == "#") break
      out = out c
      i++
    }
    print out
  }'
}

# ---------------------------------------------------------------------------
# File listers
# ---------------------------------------------------------------------------
# Produce a newline-separated list of C/C++ source files in the in-tree packages.
list_cxx_files() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.hpp' \) \
    2>/dev/null | LC_ALL=C sort
}

list_py_files() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f -name '*.py' 2>/dev/null | LC_ALL=C sort
}

list_launch_py_files() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f -name '*.launch.py' 2>/dev/null | LC_ALL=C sort
}

list_action_files() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f -name '*.action' 2>/dev/null | LC_ALL=C sort
}

list_package_xmls() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f -name 'package.xml' 2>/dev/null | LC_ALL=C sort
}

list_cmakelists() {
  local root="$1"
  find "$root/autonomy_core" "$root/autonomy_real" "$root/autonomy_sim" \
    -type f -name 'CMakeLists.txt' 2>/dev/null | LC_ALL=C sort
}

list_shell_scripts() {
  local root="$1"
  find "$root" -type f \( -name '*.sh' -o -name '*.bash' \) \
    -not -path "*/tests/*" 2>/dev/null | LC_ALL=C sort
}

list_dockerfiles() {
  local root="$1"
  find "$root" -type f -name 'Dockerfile*' -not -path "*/tests/*" 2>/dev/null | LC_ALL=C sort
}

list_ci_workflows() {
  local root="$1"
  find "$root/.github" -type f \( -name '*.yaml' -o -name '*.yml' \) 2>/dev/null | LC_ALL=C sort
}
