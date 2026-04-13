#!/usr/bin/env bash
# check_ros2_port.sh - Static test suite for the kr_autonomous_flight ROS1-to-ROS2
# port. Implements sections A-N from the port-checklist plus four regression
# guards. Safe to run on CI or locally.
#
# Usage:
#   tests/static/check_ros2_port.sh [repo-root]
#   tests/static/check_ros2_port.sh --verbose
#
# Environment variables:
#   NO_COLOR=1        disable ANSI colors
#   VERBOSE=1         dump first 10 hits per failing check
#
# Exit 0 on success, 1 on any failures.

set -uo pipefail

# -----------------------------------------------------------------------------
# Arg parsing
# -----------------------------------------------------------------------------
VERBOSE=0
REPO_ROOT_ARG=""

for arg in "$@"; do
  case "$arg" in
    --verbose|-v) VERBOSE=1 ;;
    --help|-h)
      cat <<EOF
check_ros2_port.sh [repo-root] [--verbose]

Runs the kr_autonomous_flight ROS1-to-ROS2 port static-analysis suite.

Options:
  repo-root      path to repo (default: parent of parent of this script)
  --verbose, -v  print first 10 matching lines per failing check
  --help, -h     this text

Environment:
  NO_COLOR=1     disable colored output
  VERBOSE=1      same as --verbose
EOF
      exit 0
      ;;
    *)
      REPO_ROOT_ARG="$arg"
      ;;
  esac
done
export VERBOSE

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ -n "$REPO_ROOT_ARG" ]]; then
  REPO_ROOT="$(cd "$REPO_ROOT_ARG" && pwd)"
else
  REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi

# shellcheck source=./lib.sh
source "$SCRIPT_DIR/lib.sh"

printf "%s[kr_autonomous_flight ROS2 port static test]%s\n" "$C_BOLD" "$C_RESET"
printf "  repo  : %s\n" "$REPO_ROOT"
printf "  script: %s\n" "$SCRIPT_DIR"
printf "  verbose: %s\n" "$VERBOSE"

# -----------------------------------------------------------------------------
# Managed-package list (22 in-tree packages)
# -----------------------------------------------------------------------------
MANAGED_PKGS=(
  client_launch
  rqt_quadrotor_safety
  control_launch
  estimation_launch
  fla_ukf
  mavros_interface
  px4_interface_launch
  action_planner
  coverage_utils
  jps3d
  mapper
  map_plan_launch
  motion_primitive_library
  traj_opt_ros
  action_trackers
  state_machine
  state_machine_launch
  real_experiment_launch
  gazebo_utils
  dcist_utils
  fake_lidar
  fake_sloam
)

is_managed_pkg() {
  local needle="$1"
  for p in "${MANAGED_PKGS[@]}"; do
    [[ "$p" == "$needle" ]] && return 0
  done
  return 1
}

# -----------------------------------------------------------------------------
# Section A: ROS1 C++ idioms in active source
# -----------------------------------------------------------------------------
check_a_cxx_ros1_idioms() {
  section "A. ROS1 C++ idioms in active source"
  local files
  files="$(list_cxx_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no C/C++ source files found"
    return
  fi

  # (regex, label) pairs encoded as two parallel arrays
  local -a patterns=(
    '#include[[:space:]]*<ros/ros\.h>'
    '#include[[:space:]]*<ros/package\.h>'
    '#include[[:space:]]*<tf/'
    '#include[[:space:]]*<tf2[^>]+\.h>'
    '#include[[:space:]]*<nodelet/'
    '#include[[:space:]]*<pluginlib/class_list_macros\.h>'
    '#include[[:space:]]*<actionlib/server/simple_action_server\.h>'
    '#include[[:space:]]*<actionlib/client/simple_action_client\.h>'
    '#include[[:space:]]*<sensor_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<geometry_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<nav_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<std_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<visualization_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<kr_mav_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<kr_planning_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<kr_tracker_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<planning_ros_msgs/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<action_planner/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<state_machine/[A-Z][A-Za-z0-9]+\.h>'
    '#include[[:space:]]*<action_trackers/[A-Z][A-Za-z0-9]+\.h>'
    '\bros::NodeHandle\b'
    '\bros::Publisher\b'
    '\bros::Subscriber\b'
    '\bros::Time::now\(\)'
    '\bros::Duration\b'
    '\bros::Rate\b'
    '\bros::init\('
    '\bros::spin\('
    '\bros::spinOnce\('
    '\bros::AsyncSpinner\('
    '\bros::ok\(\)'
    '\bROS_(INFO|WARN|ERROR|DEBUG|FATAL)[A-Z_]*\('
    '\bnodelet::Nodelet\b'
    'PLUGINLIB_EXPORT_CLASS\([^,]+,[[:space:]]*nodelet::'
    'actionlib::Simple(Action)?(Server|Client)'
  )
  local -a labels=(
    '#include <ros/ros.h>'
    '#include <ros/package.h>'
    '#include <tf/... (non-tf2)'
    '#include <tf2_*/*.h> (should be .hpp in Jazzy)'
    '#include <nodelet/...>'
    '#include <pluginlib/class_list_macros.h> (use .hpp)'
    '#include <actionlib/server/simple_action_server.h>'
    '#include <actionlib/client/simple_action_client.h>'
    'old-style sensor_msgs/*.h'
    'old-style geometry_msgs/*.h'
    'old-style nav_msgs/*.h'
    'old-style std_msgs/*.h'
    'old-style visualization_msgs/*.h'
    'old-style kr_mav_msgs/*.h'
    'old-style kr_planning_msgs/*.h'
    'old-style kr_tracker_msgs/*.h'
    'old-style planning_ros_msgs/*.h'
    'old-style action_planner/*.h'
    'old-style state_machine/*.h'
    'old-style action_trackers/*.h'
    'ros::NodeHandle'
    'ros::Publisher'
    'ros::Subscriber'
    'ros::Time::now()'
    'ros::Duration'
    'ros::Rate'
    'ros::init('
    'ros::spin('
    'ros::spinOnce('
    'ros::AsyncSpinner('
    'ros::ok()'
    'ROS_INFO/WARN/ERROR/DEBUG/FATAL macros'
    'nodelet::Nodelet'
    'PLUGINLIB_EXPORT_CLASS(..., nodelet::...)'
    'actionlib::Simple{Action}{Server|Client}'
  )

  # Pre-strip comments once per file into a cache so we don't re-run awk
  # for every pattern.
  local cache_dir
  cache_dir="$(mktemp -d)"
  local idx=0
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    idx=$((idx + 1))
    local cf="$cache_dir/$idx"
    strip_cxx_comments <"$f" >"$cf"
    printf "%s\n" "$f" >>"$cache_dir/names"
  done <<<"$files"

  local nidx=${#patterns[@]}
  local i
  for ((i = 0; i < nidx; i++)); do
    local pat="${patterns[$i]}"
    local lbl="${labels[$i]}"
    local tmp
    tmp="$(mktemp)"
    local j=0
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      j=$((j + 1))
      local cf="$cache_dir/$j"
      [[ -f "$cf" ]] || continue
      grep -nE "$pat" "$cf" 2>/dev/null | \
        awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
    done <"$cache_dir/names"

    # Explicit exemption: kr_trackers_manager/Tracker.h external header.
    grep -v 'kr_trackers_manager/Tracker\.h' "$tmp" >"${tmp}.filt" 2>/dev/null || true
    if [[ -f "${tmp}.filt" ]]; then
      mv "${tmp}.filt" "$tmp"
    fi

    if [[ -s "$tmp" ]]; then
      fail "C++ idiom found: $lbl"
      verbose_dump 10 <"$tmp"
    else
      pass "C++ idiom absent: $lbl"
    fi
    rm -f "$tmp"
  done
  rm -rf "$cache_dir"
}

# -----------------------------------------------------------------------------
# Section B: ROS1 Python idioms
# -----------------------------------------------------------------------------
check_b_py_ros1_idioms() {
  section "B. ROS1 Python idioms"
  local files
  files="$(list_py_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no Python files found"
    return
  fi

  local -a patterns=(
    '^[[:space:]]*import[[:space:]]+rospy\b'
    '^[[:space:]]*from[[:space:]]+rospy\b'
    '\brospy\.(Publisher|Subscriber|init_node|spin|Rate|loginfo|logwarn|logerr|logdebug|logfatal|get_param|has_param|set_param|Time|Duration|is_shutdown|on_shutdown|wait_for_message|wait_for_service|ServiceProxy|Service)\b'
    '^[[:space:]]*import[[:space:]]+tf[[:space:]]*$'
    '\bros_numpy\b'
    '\brospkg\.(RosPack|get_path)'
    '\bactionlib\.(SimpleActionClient|SimpleActionServer)'
  )
  local -a labels=(
    'import rospy'
    'from rospy ...'
    'rospy.* API call'
    'import tf (non-tf2)'
    'ros_numpy usage'
    'rospkg.RosPack / rospkg.get_path'
    'actionlib.Simple{Action}{Server|Client}'
  )

  local nidx=${#patterns[@]}
  local i
  for ((i = 0; i < nidx; i++)); do
    local pat="${patterns[$i]}"
    local lbl="${labels[$i]}"
    local tmp
    tmp="$(mktemp)"
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      strip_py_comments <"$f" | grep -nE "$pat" 2>/dev/null | \
        awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
    done <<<"$files"

    if [[ -s "$tmp" ]]; then
      fail "Python idiom found: $lbl"
      verbose_dump 10 <"$tmp"
    else
      pass "Python idiom absent: $lbl"
    fi
    rm -f "$tmp"
  done

  # Special: `from tf ...` but not `from tf2_ros` or `from tf_transformations`
  local tmp
  tmp="$(mktemp)"
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    strip_py_comments <"$f" \
      | grep -nE '^[[:space:]]*from[[:space:]]+tf([[:space:]]|\.)' \
      | grep -vE '^[[:space:]]*from[[:space:]]+tf2_ros|tf_transformations' \
      | awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
  done <<<"$files"
  if [[ -s "$tmp" ]]; then
    fail "Python idiom found: from tf (non-tf2)"
    verbose_dump 10 <"$tmp"
  else
    pass "Python idiom absent: from tf (non-tf2)"
  fi
  rm -f "$tmp"
}

# -----------------------------------------------------------------------------
# Section C: package.xml format 3 + ament
# -----------------------------------------------------------------------------
check_c_package_xml() {
  section "C. package.xml (format 3 + ament)"
  local files
  files="$(list_package_xmls "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no package.xml files found"
    return
  fi

  local tmp_fail
  tmp_fail="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    local problems=""

    if ! grep -qE '<package[[:space:]]+format="3"' "$f"; then
      problems+=$'\n    missing <package format="3">'
    fi
    if ! grep -qE '<buildtool_depend>[[:space:]]*ament_(cmake|python)[[:space:]]*</buildtool_depend>' "$f"; then
      problems+=$'\n    missing <buildtool_depend>ament_cmake|ament_python</buildtool_depend>'
    fi
    if grep -qE '<buildtool_depend>[[:space:]]*catkin[[:space:]]*</buildtool_depend>' "$f"; then
      problems+=$'\n    contains <buildtool_depend>catkin</buildtool_depend>'
    fi
    if ! grep -qE '<build_type>[[:space:]]*ament_(cmake|python)[[:space:]]*</build_type>' "$f"; then
      problems+=$'\n    missing <build_type>ament_cmake|ament_python</build_type>'
    fi
    if grep -qE '<depend>[[:space:]]*message_(generation|runtime)[[:space:]]*</depend>' "$f"; then
      problems+=$'\n    contains <depend>message_generation|message_runtime</depend>'
    fi

    if [[ -n "$problems" ]]; then
      printf "%s:%s\n" "$f" "$problems" >>"$tmp_fail"
    fi
  done <<<"$files"

  if [[ -s "$tmp_fail" ]]; then
    local count
    count="$(awk -F: '/:/ { c++ } END { print c+0 }' "$tmp_fail")"
    fail "package.xml format issues in $count file(s)"
    verbose_dump 20 <"$tmp_fail"
  else
    pass "all package.xml files are ROS2/ament format 3"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section D: CMakeLists.txt catkin leftovers + ament_package
# -----------------------------------------------------------------------------
check_d_cmakelists() {
  section "D. CMakeLists.txt (no catkin, has ament_package)"
  local files
  files="$(list_cmakelists "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no CMakeLists.txt files found"
    return
  fi

  local -a forbidden_patterns=(
    'find_package\([[:space:]]*catkin'
    'catkin_package\('
    '\$\{catkin_INCLUDE_DIRS\}'
    '\$\{catkin_LIBRARIES\}'
    'add_message_files\('
    'add_service_files\('
    'add_action_files\('
    'generate_messages\('
  )
  local -a forbidden_labels=(
    'find_package(catkin ...)'
    'catkin_package(...)'
    '${catkin_INCLUDE_DIRS}'
    '${catkin_LIBRARIES}'
    'add_message_files('
    'add_service_files('
    'add_action_files('
    'generate_messages('
  )

  local tmp_fail
  tmp_fail="$(mktemp)"
  local missing_ament=""

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    if ! grep -q 'ament_package()' "$f"; then
      missing_ament+="$f"$'\n'
    fi
    local i
    for ((i = 0; i < ${#forbidden_patterns[@]}; i++)); do
      local pat="${forbidden_patterns[$i]}"
      local lbl="${forbidden_labels[$i]}"
      if grep -nE "$pat" "$f" >/dev/null 2>&1; then
        grep -nE "$pat" "$f" | awk -v file="$f" -v lbl="$lbl" \
          '{ print file ": " lbl " -> " $0 }' >>"$tmp_fail"
      fi
    done
  done <<<"$files"

  if [[ -n "$missing_ament" ]]; then
    fail "CMakeLists missing ament_package()"
    printf "%s" "$missing_ament" | verbose_dump 10
  else
    pass "all CMakeLists.txt files contain ament_package()"
  fi

  if [[ -s "$tmp_fail" ]]; then
    fail "CMakeLists catkin leftovers found"
    verbose_dump 10 <"$tmp_fail"
  else
    pass "no catkin leftovers in CMakeLists.txt"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section E: Launch files
# -----------------------------------------------------------------------------
check_e_launch() {
  section "E. Launch files (.launch gone, .launch.py well-formed)"
  local xml_launches
  xml_launches="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
    -type f -name '*.launch' 2>/dev/null | LC_ALL=C sort || true)"

  if [[ -n "$xml_launches" ]]; then
    fail "found XML .launch files (should be .launch.py)"
    printf "%s\n" "$xml_launches" | verbose_dump 10
  else
    pass "no XML .launch files remaining under autonomy_*"
  fi

  local files
  files="$(list_launch_py_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no .launch.py files found"
    return
  fi

  local tmp_fail
  tmp_fail="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    local problems=""
    if ! grep -qE 'from[[:space:]]+launch[[:space:]]+import[[:space:]]+.*LaunchDescription' "$f"; then
      problems+="missing 'from launch import ... LaunchDescription'; "
    fi
    if ! grep -qE 'def[[:space:]]+generate_launch_description' "$f"; then
      problems+="missing generate_launch_description(); "
    fi
    if grep -qE '^[[:space:]]*<launch' "$f"; then
      problems+="contains literal <launch> XML opening tag; "
    fi
    if [[ -n "$problems" ]]; then
      printf "%s: %s\n" "$f" "$problems" >>"$tmp_fail"
    fi
  done <<<"$files"

  if [[ -s "$tmp_fail" ]]; then
    local c
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    fail ".launch.py structural issues in $c file(s)"
    verbose_dump 20 <"$tmp_fail"
  else
    pass "all .launch.py files have ROS2 structure"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section F: .action interface files
# -----------------------------------------------------------------------------
check_f_action_files() {
  section "F. .action interface files (rosidl-compatible)"
  local files
  files="$(list_action_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no .action files found"
    return
  fi

  local tmp_fail
  tmp_fail="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    local problems=""

    # bare Header at start of line
    if grep -nE '^Header[[:space:]]+' "$f" >/dev/null 2>&1; then
      problems+=$'\n    bare "Header" (use std_msgs/Header):'
      local bh
      bh="$(grep -nE '^Header[[:space:]]+' "$f" | head -5)"
      problems+=$'\n      '"$bh"
    fi
    # bare time
    if grep -nE '^time[[:space:]]+' "$f" >/dev/null 2>&1; then
      problems+=$'\n    bare "time" (use builtin_interfaces/Time):'
      local bt
      bt="$(grep -nE '^time[[:space:]]+' "$f" | head -5)"
      problems+=$'\n      '"$bt"
    fi
    # bare duration
    if grep -nE '^duration[[:space:]]+' "$f" >/dev/null 2>&1; then
      problems+=$'\n    bare "duration" (use builtin_interfaces/Duration):'
      local bd
      bd="$(grep -nE '^duration[[:space:]]+' "$f" | head -5)"
      problems+=$'\n      '"$bd"
    fi
    # camelCase field name after a type
    # Field syntax: <Type> <name>. <Type> may contain /. <name> must be
    # lower_snake_case (no uppercase letters).
    local cc
    cc="$(awk '
      /^[[:space:]]*#/ { next }
      /^[[:space:]]*$/ { next }
      /^[[:space:]]*---/ { next }
      {
        # strip trailing comment
        line = $0
        sub(/#.*/, "", line)
        # ignore constant declarations of form `<type> <NAME> = <value>`
        # which conventionally use UPPER_CASE and are ROSIDL-legal.
        if (line ~ /=/) next
        # try to match: [type] [name]
        n = split(line, parts, /[[:space:]]+/)
        type=""; name=""
        for (i=1; i<=n; i++) {
          if (parts[i] == "") continue
          if (type == "") { type = parts[i]; continue }
          name = parts[i]; break
        }
        if (name == "") next
        # name must be lower_snake_case: no uppercase letters
        if (name ~ /[A-Z]/) {
          print NR ": " line
        }
      }
    ' "$f")"
    if [[ -n "$cc" ]]; then
      problems+=$'\n    camelCase field name(s):'
      while IFS= read -r ln; do
        problems+=$'\n      '"$ln"
      done <<<"$cc"
    fi

    # Must contain two `---` separator lines
    local sep_count=0
    sep_count="$(awk '/^---/ { c++ } END { print c+0 }' "$f")"
    if [[ "$sep_count" -ne 2 ]]; then
      problems+=$'\n    expected exactly 2 "---" separator lines, found '"$sep_count"
    fi

    if [[ -n "$problems" ]]; then
      fail "$(basename "$f")$problems"
      printf "%s:%s\n" "$f" "$problems" >>"$tmp_fail"
    else
      pass "$(basename "$f")"
    fi
  done <<<"$files"

  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section G: install(PROGRAMS ...) targets exist on disk
# -----------------------------------------------------------------------------
check_g_install_programs() {
  section "G. install(PROGRAMS ...) files exist on disk"
  local files
  files="$(list_cmakelists "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no CMakeLists.txt files found"
    return
  fi

  local tmp_missing
  tmp_missing="$(mktemp)"
  local tmp_found
  tmp_found="$(mktemp)"

  while IFS= read -r cmake; do
    [[ -z "$cmake" ]] && continue
    local pkg_dir
    pkg_dir="$(dirname "$cmake")"
    awk '
      /install[[:space:]]*\([[:space:]]*PROGRAMS/ { in_install=1; sub(/^.*install[[:space:]]*\([[:space:]]*PROGRAMS/, ""); }
      in_install >= 1 {
        line = $0
        # Remove DESTINATION and after
        sub(/DESTINATION.*$/, "", line)
        sub(/\).*$/, "", line)
        # Split on whitespace
        n = split(line, toks, /[[:space:]]+/)
        for (i=1; i<=n; i++) {
          t = toks[i]
          gsub(/[[:space:]]+/, "", t)
          gsub(/\(/, "", t)
          gsub(/\)/, "", t)
          if (t == "" || t == "PROGRAMS") continue
          if (t == "DESTINATION") { in_install=0; continue }
          print t
        }
        if ($0 ~ /\)/) in_install=0
        if ($0 ~ /DESTINATION/) in_install=0
      }
    ' "$cmake" | while IFS= read -r target; do
      [[ -z "$target" ]] && continue
      # Strip CMake variables like ${PROJECT_NAME}
      if [[ "$target" == *"\${"* ]]; then
        # We cannot resolve variables statically - skip
        continue
      fi
      local full
      if [[ "$target" == /* ]]; then
        full="$target"
      else
        full="$pkg_dir/$target"
      fi
      if [[ -e "$full" ]]; then
        printf "%s\n" "$full" >>"$tmp_found"
      else
        printf "%s: missing %s\n" "$cmake" "$target" >>"$tmp_missing"
      fi
    done
  done <<<"$files"

  if [[ -s "$tmp_missing" ]]; then
    local c
    c="$(wc -l <"$tmp_missing" | tr -d ' ')"
    fail "install(PROGRAMS ...) references $c missing file(s)"
    verbose_dump 10 <"$tmp_missing"
  else
    local c
    c="$(awk 'END { print NR }' "$tmp_found" 2>/dev/null)"
    [[ -z "$c" ]] && c=0
    pass "all install(PROGRAMS ...) targets exist ($c checked)"
  fi
  rm -f "$tmp_missing" "$tmp_found"
}

# -----------------------------------------------------------------------------
# Section H: nodelet_plugins.xml (plural, ROS1 name) must not exist
# -----------------------------------------------------------------------------
check_h_nodelet_plugins() {
  section "H. nodelet_plugins.xml (plural) removed"
  local found
  found="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
    -type f -name 'nodelet_plugins.xml' 2>/dev/null || true)"
  if [[ -n "$found" ]]; then
    fail "legacy nodelet_plugins.xml files found"
    printf "%s\n" "$found" | verbose_dump 10
  else
    pass "no legacy nodelet_plugins.xml files"
  fi
}

# -----------------------------------------------------------------------------
# Section I: no *.launch (XML) anywhere in repo except tests/ and tools/
# -----------------------------------------------------------------------------
check_i_xml_launch_anywhere() {
  section "I. No XML .launch files anywhere in repo"
  local found
  found="$(find "$REPO_ROOT" -type f -name '*.launch' \
    -not -path "*/tests/*" -not -path "*/tools/*" 2>/dev/null || true)"
  if [[ -n "$found" ]]; then
    fail "stray XML .launch files found"
    printf "%s\n" "$found" | verbose_dump 10
  else
    pass "no stray XML .launch files"
  fi
}

# -----------------------------------------------------------------------------
# Section J: Launch-argument consistency (within-file)
# -----------------------------------------------------------------------------
check_j_launch_arg_consistency() {
  section "J. Launch-argument consistency within each .launch.py"
  local files
  files="$(list_launch_py_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no .launch.py files found"
    return
  fi

  local tmp_fail
  tmp_fail="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    # Portable (non-gawk) approach: slurp the file with tr '\n' ' ' to
    # collapse multi-line DeclareLaunchArgument/LaunchConfiguration calls
    # onto a single logical line, then grep out every occurrence.
    local decl_names use_names missing
    decl_names="$(python_style_extract_decls "$f" | LC_ALL=C sort -u)"
    use_names="$(python_style_extract_uses "$f" | LC_ALL=C sort -u)"

    missing=""
    if [[ -n "$use_names" ]]; then
      while IFS= read -r name; do
        [[ -z "$name" ]] && continue
        case "$name" in
          log_level|launch_prefix|use_sim_time|node_log_level|output_log_level)
            continue ;;
        esac
        if ! printf "%s\n" "$decl_names" | grep -qxF "$name"; then
          missing+="$name"$'\n'
        fi
      done <<<"$use_names"
    fi

    if [[ -n "$missing" ]]; then
      local first
      first="$(printf "%s" "$missing" | head -5 | tr '\n' ',' | sed 's/,$//')"
      printf "%s: undeclared LaunchConfiguration(): %s\n" "$f" "$first" >>"$tmp_fail"
    fi
  done <<<"$files"

  if [[ -s "$tmp_fail" ]]; then
    local c
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    fail "launch-arg consistency issues in $c file(s)"
    verbose_dump 20 <"$tmp_fail"
  else
    pass "all LaunchConfiguration() names are declared in their own file"
  fi
  rm -f "$tmp_fail"
}

# Helper fallback: extract decl names via python-free method using grep+sed.
python_style_extract_decls() {
  local f="$1"
  # slurp then strip newlines to make multi-line calls grep-able
  tr '\n' ' ' <"$f" \
    | grep -oE 'DeclareLaunchArgument[[:space:]]*\([[:space:]]*["'\''][^"'\'']+["'\'']' \
    | sed -E 's/.*["'\'']([^"'\'']+)["'\'']/\1/'
}

python_style_extract_uses() {
  local f="$1"
  tr '\n' ' ' <"$f" \
    | grep -oE 'LaunchConfiguration[[:space:]]*\([[:space:]]*["'\''][^"'\'']+["'\'']' \
    | sed -E 's/.*["'\'']([^"'\'']+)["'\'']/\1/'
}

# -----------------------------------------------------------------------------
# Section K: Hardcoded user-specific absolute paths
# -----------------------------------------------------------------------------
check_k_hardcoded_paths() {
  section "K. Hardcoded user-specific absolute paths"
  local files
  files="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
    -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.py' -o -name '*.cc' \) \
    2>/dev/null | LC_ALL=C sort)"
  if [[ -z "$files" ]]; then
    skip "no source files found"
    return
  fi

  local -a patterns=(
    '/home/[a-z0-9_]+/'
    '/root/'
    '/opt/slideslam_docker_ws'
    '/opt/bags/'
  )
  local -a labels=(
    '/home/<user>/...'
    '/root/...'
    '/opt/slideslam_docker_ws'
    '/opt/bags/'
  )

  local i
  for ((i = 0; i < ${#patterns[@]}; i++)); do
    local pat="${patterns[$i]}"
    local lbl="${labels[$i]}"
    local tmp
    tmp="$(mktemp)"
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      if [[ "$f" == *.py ]]; then
        strip_py_comments <"$f" | grep -nE "$pat" | awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
      else
        strip_cxx_comments <"$f" | grep -nE "$pat" | awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
      fi
    done <<<"$files"

    if [[ -s "$tmp" ]]; then
      fail "hardcoded path $lbl"
      verbose_dump 10 <"$tmp"
    else
      pass "no hardcoded $lbl"
    fi
    rm -f "$tmp"
  done
}

# -----------------------------------------------------------------------------
# Section L: Node(package=..., executable=...) resolves in managed packages
# -----------------------------------------------------------------------------
check_l_node_executables() {
  section "L. Node(executable=...) resolves in managed packages"

  local launch_files
  launch_files="$(list_launch_py_files "$REPO_ROOT")"
  if [[ -z "$launch_files" ]]; then
    skip "no launch files found"
    return
  fi

  local cmakes
  cmakes="$(list_cmakelists "$REPO_ROOT")"

  # Build map package_name -> CMakeLists path
  declare -A PKG_CMAKE
  while IFS= read -r cm; do
    [[ -z "$cm" ]] && continue
    local proj
    proj="$(grep -m1 -oE 'project\([[:space:]]*[A-Za-z0-9_]+' "$cm" | sed -E 's/project\([[:space:]]*//')"
    if [[ -n "$proj" ]]; then
      PKG_CMAKE["$proj"]="$cm"
    fi
  done <<<"$cmakes"

  # Helper: given pkg name + exec name, return 0 if it resolves
  resolves_in_pkg() {
    local pkg="$1"
    local exe="$2"
    local cm="${PKG_CMAKE[$pkg]:-}"
    [[ -z "$cm" ]] && return 1
    local pkg_dir
    pkg_dir="$(dirname "$cm")"

    # Strip .py suffix for the C++ executable check
    local exe_no_py="${exe%.py}"

    # Check add_executable(<exe> ...) — matches bare name or ${PROJECT_NAME}.
    if grep -qE "add_executable\([[:space:]]*${exe_no_py}([[:space:]]|$)" "$cm"; then
      return 0
    fi
    # Check install(PROGRAMS .../<exe>) using state-machine awk.
    if awk -v needle="$exe" '
      /install[[:space:]]*\([[:space:]]*PROGRAMS/ { in_install=1 }
      in_install >= 1 {
        line = $0
        sub(/DESTINATION.*$/, "", line)
        n = split(line, toks, /[[:space:]]+/)
        for (i=1; i<=n; i++) {
          t = toks[i]
          gsub(/\(/, "", t)
          gsub(/\)/, "", t)
          if (t == "" || t == "install" || t == "PROGRAMS") continue
          b = t
          sub(/^.*\//, "", b)
          if (b == needle) { print "HIT"; exit }
          b2 = b
          sub(/\.py$/, "", b2)
          if (b2 == needle) { print "HIT"; exit }
          if (b == needle ".py") { print "HIT"; exit }
        }
        if ($0 ~ /\)/) in_install=0
      }
    ' "$cm" | grep -q HIT; then
      return 0
    fi

    # install(DIRECTORY scripts/ ...) — python scripts dropped into lib/<pkg>.
    # Check for the script file on disk under the package's scripts/ dir.
    if grep -qE 'install[[:space:]]*\([[:space:]]*DIRECTORY[[:space:]]+scripts' "$cm"; then
      if [[ -e "$pkg_dir/scripts/$exe" ]] || [[ -e "$pkg_dir/scripts/${exe}.py" ]] \
         || [[ -e "$pkg_dir/scripts/${exe_no_py}" ]] || [[ -e "$pkg_dir/scripts/${exe_no_py}.py" ]]; then
        return 0
      fi
    fi
    return 1
  }

  local tmp_fail
  tmp_fail="$(mktemp)"

  while IFS= read -r lf; do
    [[ -z "$lf" ]] && continue
    # Portable (non-gawk) approach: slurp and split on `Node(` tokens.
    # We flatten the file with tr '\n' ' ', then grep out every non-greedy
    # `Node(...)` up to the next `)`. For each block, extract the
    # package='...' and executable='...' arguments.
    # Use word boundary to avoid matching ComposableNode/LifecycleNode.
    local nodes=""
    local joined
    joined="$(tr '\n' ' ' <"$lf")"
    local blocks
    blocks="$(printf "%s" "$joined" | grep -oE "(^|[^a-zA-Z0-9_])Node[[:space:]]*\([^)]*\)" 2>/dev/null || true)"
    if [[ -n "$blocks" ]]; then
      while IFS= read -r blk; do
        [[ -z "$blk" ]] && continue
        local pkg exe
        pkg="$(printf "%s" "$blk" | grep -oE "package[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")"
        exe="$(printf "%s" "$blk" | grep -oE "executable[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")"
        if [[ -n "$pkg" && -n "$exe" ]]; then
          nodes+="$pkg"$'\t'"$exe"$'\n'
        fi
      done <<<"$blocks"
    fi

    while IFS=$'\t' read -r pkg exe; do
      [[ -z "$pkg" || -z "$exe" ]] && continue
      if ! is_managed_pkg "$pkg"; then
        continue
      fi
      if ! resolves_in_pkg "$pkg" "$exe"; then
        printf "%s: Node(package='%s', executable='%s') not found in %s CMakeLists\n" \
          "$lf" "$pkg" "$exe" "$pkg" >>"$tmp_fail"
      fi
    done <<<"$nodes"
  done <<<"$launch_files"

  if [[ -s "$tmp_fail" ]]; then
    local c
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    fail "Node executable unresolved in $c entr(y/ies)"
    verbose_dump 20 <"$tmp_fail"
  else
    pass "all Node(executable=...) targets resolve in managed packages"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section M: Duplicate raw declare_parameter across files
# -----------------------------------------------------------------------------
check_m_duplicate_declare_parameter() {
  section "M. Duplicate raw declare_parameter() across files within a package"

  local files
  files="$(list_cxx_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no C/C++ files found"
    return
  fi

  local tmp_all
  tmp_all="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    # Find package dir = nearest parent containing package.xml
    local d
    d="$(dirname "$f")"
    local pkg_dir=""
    while [[ "$d" != "/" && "$d" != "." ]]; do
      if [[ -f "$d/package.xml" ]]; then
        pkg_dir="$d"
        break
      fi
      d="$(dirname "$d")"
    done
    [[ -z "$pkg_dir" ]] && continue
    local pkg_name
    pkg_name="$(basename "$pkg_dir")"

    # Grep `declare_parameter("key"...)` NOT preceded by `_declare_or_get<`.
    # Portable (non-gawk) approach: strip comments, skip wrapper lines,
    # then use grep -oE to pull every raw declare_parameter call and
    # extract the quoted first arg via sed.
    # We prepend a leading space to each line so the word-boundary check
    # works at column 0 too.
    strip_cxx_comments <"$f" \
      | grep -vE '_declare_or_get[[:space:]]*<' \
      | sed -E 's/^/ /' \
      | grep -oE '[^_a-zA-Z0-9]declare_parameter[[:space:]]*\([[:space:]]*"[^"]+"' \
      | sed -E 's/.*"([^"]+)".*/\1/' \
      | awk -v pkg="$pkg_name" -v file="$f" '{ print pkg "\t" $0 "\t" file }' \
      >>"$tmp_all" || true
  done <<<"$files"

  # Aggregate: for each (pkg, key), if referenced in >= 2 distinct files, flag it.
  local tmp_dup
  tmp_dup="$(mktemp)"

  LC_ALL=C sort -u "$tmp_all" | awk -F'\t' '
    {
      key = $1 "\t" $2
      count[key]++
      files[key] = files[key] "\n      " $3
    }
    END {
      for (k in count) {
        if (count[k] >= 2) {
          print k " (" count[k] " files):" files[k]
        }
      }
    }
  ' >"$tmp_dup"

  if [[ -s "$tmp_dup" ]]; then
    local c
    c="$(awk '/\(/ { c++ } END { print c+0 }' "$tmp_dup")"
    fail "$c duplicate raw declare_parameter key(s) across files in same package"
    verbose_dump 30 <"$tmp_dup"
  else
    pass "no duplicate raw declare_parameter() keys within any package"
  fi
  rm -f "$tmp_all" "$tmp_dup"
}

# -----------------------------------------------------------------------------
# Section N: Dockerfiles + CI + shell scripts ROS1 leftovers
# -----------------------------------------------------------------------------
check_n_dockerfiles_ci_shell() {
  section "N. Dockerfiles / CI / shell scripts — no ROS1 leftovers"

  # N.1 - Dockerfiles
  local dockers
  dockers="$(list_dockerfiles "$REPO_ROOT")"
  if [[ -z "$dockers" ]]; then
    skip "no Dockerfiles found"
  else
    local tmp
    tmp="$(mktemp)"
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      grep -nEi 'noetic|ros:noetic|ubuntu:20\.04' "$f" 2>/dev/null | \
        awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
    done <<<"$dockers"
    if [[ -s "$tmp" ]]; then
      fail "Dockerfiles contain ROS1 leftovers"
      verbose_dump 15 <"$tmp"
    else
      pass "no Dockerfiles reference noetic / ubuntu:20.04"
    fi
    rm -f "$tmp"
  fi

  # N.2 - CI workflows
  local ci
  ci="$(list_ci_workflows "$REPO_ROOT")"
  if [[ -z "$ci" ]]; then
    skip "no CI workflows found"
  else
    local tmp
    tmp="$(mktemp)"
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      grep -nEi 'noetic|ubuntu-20\.04' "$f" 2>/dev/null | \
        awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
    done <<<"$ci"
    if [[ -s "$tmp" ]]; then
      fail "CI workflows contain ROS1 leftovers"
      verbose_dump 15 <"$tmp"
    else
      pass "no CI workflow references noetic / ubuntu-20.04"
    fi
    rm -f "$tmp"
  fi

  # N.3 - Shell scripts (regression guard #1: must walk ALL *.sh / *.bash)
  local shells
  shells="$(list_shell_scripts "$REPO_ROOT")"
  if [[ -z "$shells" ]]; then
    skip "no shell scripts found"
    return
  fi

  # We strip comments (# to end-of-line, outside of quotes) and
  # skip matches inside simple `echo "..."` strings by a quick grep for the
  # echo prefix. Not perfect but good enough.
  local -a patterns=(
    '\broslaunch\b'
    '\brosrun\b'
    '\brosbag[[:space:]]+(play|record)\b'
    '\brostopic\b'
    '\brosnode\b'
    '\brosservice\b'
    '\brosparam\b'
    '\broscore\b'
    '\broscd\b'
    '\bcatkin[[:space:]]+build\b'
    '\bcatkin_make\b'
    'devel/setup\.bash'
    'source[[:space:]]+/opt/ros/noetic'
  )
  local -a labels=(
    'roslaunch'
    'rosrun'
    'rosbag play|record'
    'rostopic'
    'rosnode'
    'rosservice'
    'rosparam'
    'roscore'
    'roscd'
    'catkin build'
    'catkin_make'
    'devel/setup.bash'
    'source /opt/ros/noetic'
  )

  local i
  for ((i = 0; i < ${#patterns[@]}; i++)); do
    local pat="${patterns[$i]}"
    local lbl="${labels[$i]}"
    local tmp
    tmp="$(mktemp)"
    while IFS= read -r f; do
      [[ -z "$f" ]] && continue
      # Skip lines that are pure comments (ws + #). This is conservative —
      # we do NOT try to strip inline # comments because shell # comments
      # are hard to tell apart from #!/hashbang, ${var#...} parameter
      # expansions, and quoted #. False positives on inline # comments are
      # rare in practice for the tokens we care about.
      awk '/^[[:space:]]*#/ { next } { print }' "$f" \
        | grep -nE "$pat" 2>/dev/null \
        | awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
    done <<<"$shells"

    if [[ -s "$tmp" ]]; then
      fail "shell script ROS1 leftover: $lbl"
      verbose_dump 10 <"$tmp"
    else
      pass "no $lbl in shell scripts"
    fi
    rm -f "$tmp"
  done
}

# -----------------------------------------------------------------------------
# Regression guards (sub-checks)
# -----------------------------------------------------------------------------
check_rg1_dynamic_reconfigure_cfg() {
  section "RG1. Dynamic reconfigure .cfg files (WARN)"
  local cfgs
  cfgs="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
    -type f -path '*/cfg/*.cfg' 2>/dev/null | LC_ALL=C sort || true)"
  if [[ -n "$cfgs" ]]; then
    local cfg_count
    cfg_count="$(printf '%s\n' "$cfgs" | awk 'NF { c++ } END { print c+0 }')"
    warn "$cfg_count .cfg files present (ROS2 has no dynamic_reconfigure drop-in; migrate to parameter callbacks)"
    printf "%s\n" "$cfgs" | verbose_dump 20
    pass "dynamic_reconfigure .cfg sub-check (warn only)"
  else
    pass "no .cfg files in cfg/ directories"
  fi
}

check_rg2_package_readme() {
  section "RG2. Package-level README.md references to roslaunch/rosrun/rosbag (WARN)"
  local readmes
  readmes="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
    -type f -name 'README.md' 2>/dev/null | LC_ALL=C sort || true)"
  if [[ -z "$readmes" ]]; then
    skip "no package-level README.md files"
    return
  fi
  local tmp
  tmp="$(mktemp)"
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    grep -nE '\b(roslaunch|rosrun|rosbag)\b' "$f" 2>/dev/null | \
      awk -v file="$f" '{ print file ":" $0 }' >>"$tmp" || true
  done <<<"$readmes"

  if [[ -s "$tmp" ]]; then
    warn "package README.md files still reference ROS1 CLIs"
    verbose_dump 10 <"$tmp"
    pass "package README sub-check (warn only)"
  else
    pass "no ROS1 CLI references in package README files"
  fi
  rm -f "$tmp"
}

# -----------------------------------------------------------------------------
# Run everything
# -----------------------------------------------------------------------------
main() {
  check_a_cxx_ros1_idioms
  check_b_py_ros1_idioms
  check_c_package_xml
  check_d_cmakelists
  check_e_launch
  check_f_action_files
  check_g_install_programs
  check_h_nodelet_plugins
  check_i_xml_launch_anywhere
  check_j_launch_arg_consistency
  check_k_hardcoded_paths
  check_l_node_executables
  check_m_duplicate_declare_parameter
  check_n_dockerfiles_ci_shell
  check_rg1_dynamic_reconfigure_cfg
  check_rg2_package_readme

  print_summary
  [[ $FAILED -gt 0 ]] && exit 1 || exit 0
}

main "$@"
