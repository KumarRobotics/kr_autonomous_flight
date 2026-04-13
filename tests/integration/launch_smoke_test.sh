#!/usr/bin/env bash
# launch_smoke_test.sh - parse-only smoke test for every *.launch.py in the repo.
#
# For each launch file under autonomy_core/, autonomy_real/, autonomy_sim/ we run
#
#     ros2 launch --print-description <abs_path>
#
# with a per-file timeout. `--print-description` resolves the LaunchDescription
# without actually starting any nodes, so this is a pure parse/import check. It
# works even against an unbuilt workspace: all we need is `ros2` on PATH. The
# test flags any file that:
#
#   - raises an import error or other Python exception (`FAIL`)
#   - hangs past $LAUNCH_TIMEOUT seconds (`TIMEOUT`, typically a blocking call at
#     import time or a misbehaving `OpaqueFunction`)
#   - completes cleanly (`OK`)
#
# If `ros2` or `timeout` (GNU coreutils) is missing the script skips with exit
# code 77 - the autotools "skipped" convention - so CI runners without a ROS2
# environment treat it as skipped rather than failed.
#
# Usage:
#   tests/integration/launch_smoke_test.sh [repo_root]
#
# Environment:
#   LAUNCH_TIMEOUT    Per-file timeout in seconds (default: 20)
#   VERBOSE=1         Dump the full captured output on failure (default: last 5
#                     lines only)
#   NO_COLOR=1        Disable ANSI color output
#
# Exit codes:
#   0   all launches parsed cleanly
#   1   at least one FAIL or TIMEOUT
#   77  ros2 / timeout missing - skipped
#
# Design mirrors SlideSLAM's tests/integration/launch_smoke_test.sh, adapted to
# the kr_autonomous_flight layout.

set -uo pipefail

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

LAUNCH_TIMEOUT="${LAUNCH_TIMEOUT:-20}"

# ---------------------------------------------------------------------------
# Colors (tty + NO_COLOR aware)
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
# Temp file cleanup
# ---------------------------------------------------------------------------
TMPFILE=""
cleanup() {
  if [[ -n "$TMPFILE" && -e "$TMPFILE" ]]; then
    rm -f "$TMPFILE"
  fi
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Prerequisite check - skip with exit 77 if tooling is missing
# ---------------------------------------------------------------------------
missing=()
if ! command -v ros2 >/dev/null 2>&1; then
  missing+=("ros2")
fi
if ! command -v timeout >/dev/null 2>&1; then
  missing+=("timeout")
fi
if (( ${#missing[@]} > 0 )); then
  printf "%s[SKIP]%s launch_smoke_test: missing prerequisite(s): %s\n" \
    "$C_YELLOW" "$C_RESET" "${missing[*]}"
  printf "       source /opt/ros/jazzy/setup.bash and re-run, or install GNU coreutils.\n"
  exit 77
fi

# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------
printf "%s==> launch_smoke_test%s\n" "${C_BOLD}${C_BLUE}" "${C_RESET}"
printf "    repo root : %s\n" "$REPO_ROOT"
printf "    timeout   : %ss per file\n" "$LAUNCH_TIMEOUT"
printf "\n"

# ---------------------------------------------------------------------------
# Walk every *.launch.py under the three in-tree roots.
# Skip anything under a tests/ or .git/ directory.
# Sort so output is deterministic regardless of filesystem order.
# ---------------------------------------------------------------------------
declare -a SEARCH_ROOTS=()
for sub in autonomy_core autonomy_real autonomy_sim; do
  if [[ -d "$REPO_ROOT/$sub" ]]; then
    SEARCH_ROOTS+=("$REPO_ROOT/$sub")
  fi
done

if (( ${#SEARCH_ROOTS[@]} == 0 )); then
  printf "%s[FAIL]%s no autonomy_core/ autonomy_real/ autonomy_sim/ subtrees under %s\n" \
    "$C_RED" "$C_RESET" "$REPO_ROOT"
  exit 1
fi

TOTAL=0
OK_COUNT=0
FAIL_COUNT=0
TIMEOUT_COUNT=0
FAILED_FILES=()

# Use a while-read loop (not find -exec) so we can keep per-file counters.
while IFS= read -r -d '' launch_file; do
  TOTAL=$((TOTAL + 1))
  rel="${launch_file#$REPO_ROOT/}"

  TMPFILE="$(mktemp)"

  timeout --signal=TERM "$LAUNCH_TIMEOUT" \
    ros2 launch --print-description "$launch_file" \
    >"$TMPFILE" 2>&1
  rc=$?

  case $rc in
    0)
      status="OK"
      ;;
    124)
      # GNU timeout returns 124 when the command was killed by SIGTERM due to
      # the time limit.
      status="TIMEOUT"
      ;;
    *)
      status="FAIL"
      ;;
  esac

  case "$status" in
    OK)
      OK_COUNT=$((OK_COUNT + 1))
      printf "  %s[OK]%s      %s\n" "$C_GREEN" "$C_RESET" "$rel"
      ;;
    TIMEOUT)
      TIMEOUT_COUNT=$((TIMEOUT_COUNT + 1))
      FAILED_FILES+=("$rel (timeout)")
      printf "  %s[TIMEOUT]%s %s  (exceeded %ss)\n" \
        "$C_YELLOW" "$C_RESET" "$rel" "$LAUNCH_TIMEOUT"
      if [[ "${VERBOSE:-0}" == "1" ]]; then
        sed 's/^/      /' "$TMPFILE"
      else
        tail -n 5 "$TMPFILE" | sed 's/^/      /'
      fi
      ;;
    FAIL)
      FAIL_COUNT=$((FAIL_COUNT + 1))
      FAILED_FILES+=("$rel (rc=$rc)")
      printf "  %s[FAIL]%s    %s  (rc=%d)\n" \
        "$C_RED" "$C_RESET" "$rel" "$rc"
      if [[ "${VERBOSE:-0}" == "1" ]]; then
        sed 's/^/      /' "$TMPFILE"
      else
        tail -n 5 "$TMPFILE" | sed 's/^/      /'
      fi
      ;;
  esac

  rm -f "$TMPFILE"
  TMPFILE=""
done < <(
  find "${SEARCH_ROOTS[@]}" \
    -type f -name '*.launch.py' \
    -not -path '*/tests/*' \
    -not -path '*/.git/*' \
    -print0 2>/dev/null | LC_ALL=C sort -z
)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
printf "\n"
if (( TOTAL == 0 )); then
  printf "%s[FAIL]%s no *.launch.py files found under %s\n" \
    "$C_RED" "$C_RESET" "${SEARCH_ROOTS[*]}"
  exit 1
fi

summary_color="$C_GREEN"
if (( FAIL_COUNT > 0 || TIMEOUT_COUNT > 0 )); then
  summary_color="$C_RED"
fi

printf "%s%s==========================================%s\n" \
  "$C_BOLD" "$summary_color" "$C_RESET"
printf "%s%s%d launches, %d OK, %d failed, %d timeout%s\n" \
  "$C_BOLD" "$summary_color" \
  "$TOTAL" "$OK_COUNT" "$FAIL_COUNT" "$TIMEOUT_COUNT" "$C_RESET"
printf "%s%s==========================================%s\n" \
  "$C_BOLD" "$summary_color" "$C_RESET"

if (( ${#FAILED_FILES[@]} > 0 )); then
  printf "\n%sFailing launches:%s\n" "$C_BOLD" "$C_RESET"
  for f in "${FAILED_FILES[@]}"; do
    printf "  - %s\n" "$f"
  done
fi

if (( FAIL_COUNT > 0 || TIMEOUT_COUNT > 0 )); then
  exit 1
fi
exit 0
