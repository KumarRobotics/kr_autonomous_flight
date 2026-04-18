#!/usr/bin/env bash
# check_ros2_port.sh - Static test suite for the kr_autonomous_flight ROS1-to-ROS2
# port. Implements sections A-Q from the port-checklist plus two regression
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

  # Helper: for ament_python packages (no CMakeLists in the tree but a
  # setup.py with console_scripts entry_points), resolve <exe> against the
  # package's console_scripts list.
  resolves_in_ament_python_pkg() {
    local pkg="$1"
    local exe="$2"
    # Find a setup.py under autonomy_*/*/<pkg>/ or autonomy_*/*/*/<pkg>/ that
    # declares the package name. Match by the package name appearing in a
    # `name='<pkg>'` line near the top of the file.
    local setup_py
    setup_py="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
                  -maxdepth 5 -type f -name setup.py 2>/dev/null \
                | while IFS= read -r sp; do
                    if grep -qE "name[[:space:]]*=[[:space:]]*['\"]${pkg}['\"]" "$sp"; then
                      echo "$sp"
                      break
                    fi
                  done)"
    [[ -z "$setup_py" ]] && return 1
    # Match an entry_points console_scripts entry of the form
    #   '<exe> = something:main'
    # Tolerate leading whitespace and surrounding quotes (single or double).
    local exe_no_py="${exe%.py}"
    if grep -qE "['\"](${exe}|${exe_no_py})[[:space:]]*=[[:space:]]*[A-Za-z0-9_.]+:" "$setup_py"; then
      return 0
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
        # Fall back to ament_python setup.py console_scripts resolution for
        # pure-Python packages that ship no CMakeLists.txt.
        if ! resolves_in_ament_python_pkg "$pkg" "$exe"; then
          printf "%s: Node(package='%s', executable='%s') not found in %s CMakeLists or setup.py console_scripts\n" \
            "$lf" "$pkg" "$exe" "$pkg" >>"$tmp_fail"
        fi
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
# Section O: cross-package #include <pkg/...> vs package.xml <depend>pkg</depend>
# -----------------------------------------------------------------------------
# For every C++ source file under autonomy_*, extract `#include <pkg/...>` /
# `#include "pkg/..."` top-level package names, find the owning package.xml,
# and verify every non-exempt `pkg` appears in the package.xml's <depend>-like
# tags. Self-includes, system headers, standard-library, and certain known
# third-party libs that are pulled via find_package() are skipped.
check_o_include_depends() {
  section "O. #include <pkg/...> cross-referenced against package.xml <depend>"
  local files
  files="$(list_cxx_files "$REPO_ROOT")"
  if [[ -z "$files" ]]; then
    skip "no C/C++ source files found"
    return
  fi

  # Exemption set: the regex below already requires a `/` after the pkg name
  # so single-token C++ stdlib headers (`<string>`, `<vector>`, ...) are never
  # extracted. The list below covers multi-token system headers (`<pcl/...>`)
  # that are resolved via find_package() + target_link_libraries instead of
  # <depend>, plus a few known external include-prefix paths whose owning ROS
  # package name is DIFFERENT from the top-level include directory
  # (e.g. `plan_manage/` ships inside a bigger `opt_planner` package).
  local exempt_list=$'\nEigen\neigen3\nunsupported\nboost\ngtsam\npcl\nopencv2\nOpenCV\nfmt\nyaml-cpp\nglog\ntbb\ngtest\nbenchmark\nnetinet\narpa\nlinux\nfcntl\nunistd\npthread\nsys\nrcutils\nplan_manage\ntraj_utils\n'

  # Cache: pkg_name -> absolute package.xml path; pkg_name -> newline-joined depends.
  local -A PKGXML
  local -A PKGDEPS

  # Seed cache from every managed package.xml in-tree.
  local xmls
  xmls="$(list_package_xmls "$REPO_ROOT")"
  while IFS= read -r xml; do
    [[ -z "$xml" ]] && continue
    local name
    name="$(package_xml_name "$xml")"
    [[ -z "$name" ]] && continue
    PKGXML["$name"]="$xml"
    PKGDEPS["$name"]=$'\n'"$(package_xml_depends "$xml")"$'\n'
  done <<<"$xmls"

  # Build include-path top-dir -> owning pkg name map. For each managed
  # package we scan its `include/*` subdirs and record the top-level name.
  # Example: motion_primitive_library exports headers under mpl_collision/,
  # mpl_basis/, mpl_planner/ -- so `mpl_collision` maps to
  # `motion_primitive_library`. This avoids false positives on include paths
  # that legitimately resolve to a package with a different name.
  local -A INCL2PKG
  local pxml
  while IFS= read -r pxml; do
    [[ -z "$pxml" ]] && continue
    local pname
    pname="$(package_xml_name "$pxml")"
    [[ -z "$pname" ]] && continue
    local pdir
    pdir="$(dirname "$pxml")"
    [[ -d "$pdir/include" ]] || continue
    local sub
    while IFS= read -r sub; do
      [[ -z "$sub" ]] && continue
      local b
      b="$(basename "$sub")"
      INCL2PKG["$b"]="$pname"
    done < <(find "$pdir/include" -mindepth 1 -maxdepth 1 -type d 2>/dev/null)
  done <<<"$xmls"

  local tmp_fail
  tmp_fail="$(mktemp)"
  local tmp_hits
  tmp_hits="$(mktemp)"

  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    # Find owning package.xml
    local owning_xml
    owning_xml="$(find_owning_package_xml "$f")"
    [[ -z "$owning_xml" ]] && continue
    local owning_name
    owning_name="$(package_xml_name "$owning_xml")"
    [[ -z "$owning_name" ]] && continue

    # Carve-out: only check files belonging to one of the 22 managed packages.
    if ! is_managed_pkg "$owning_name"; then
      continue
    fi

    local deps="${PKGDEPS[$owning_name]:-}"

    # Extract top-level pkg names from #include lines (both <...> and "...").
    # Require a slash after pkg to skip C++ stdlib (`<string>`, `<vector>`, ...).
    local pkgs
    pkgs="$(strip_cxx_comments <"$f" \
            | grep -oE '^[[:space:]]*#include[[:space:]]*[<"][A-Za-z_][A-Za-z0-9_]*/' \
            | sed -E 's/.*[<"]([A-Za-z_][A-Za-z0-9_]*)\/.*/\1/' \
            | LC_ALL=C sort -u)"
    [[ -z "$pkgs" ]] && continue
    while IFS= read -r pkg; do
      [[ -z "$pkg" ]] && continue
      printf "%s\n" "$pkg" >>"$tmp_hits"
      # Skip exempt pkgs (system libs, stdlib categories).
      if [[ "$exempt_list" == *$'\n'"$pkg"$'\n'* ]]; then
        continue
      fi
      # Skip self-include.
      if [[ "$pkg" == "$owning_name" ]]; then
        continue
      fi
      # Resolve include-path top-dir to package name when we know an
      # in-tree package exports headers under that subdirectory name.
      local resolved_pkg="$pkg"
      if [[ -n "${INCL2PKG[$pkg]:-}" ]]; then
        resolved_pkg="${INCL2PKG[$pkg]}"
      fi
      # Skip self-include via resolved name too.
      if [[ "$resolved_pkg" == "$owning_name" ]]; then
        continue
      fi
      # If declared in deps (either the raw include top-dir or the resolved
      # package name), pass.
      if printf '%s' "$deps" | grep -qxF "$pkg"; then
        continue
      fi
      if [[ "$resolved_pkg" != "$pkg" ]] && printf '%s' "$deps" | grep -qxF "$resolved_pkg"; then
        continue
      fi
      if [[ "$resolved_pkg" != "$pkg" ]]; then
        printf "%s: #include <%s/...> (-> pkg '%s') but '%s' not in %s\n" \
          "$f" "$pkg" "$resolved_pkg" "$resolved_pkg" "$owning_xml" >>"$tmp_fail"
      else
        printf "%s: #include <%s/...> but '%s' not in %s\n" \
          "$f" "$pkg" "$pkg" "$owning_xml" >>"$tmp_fail"
      fi
    done <<<"$pkgs"
  done <<<"$files"

  local unique_hits
  unique_hits="$(LC_ALL=C sort -u "$tmp_hits" 2>/dev/null | awk 'NF' | wc -l | tr -d ' ')"

  if [[ -s "$tmp_fail" ]]; then
    local c
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    # Collapse identical (file, pkg) lines.
    LC_ALL=C sort -u "$tmp_fail" -o "$tmp_fail"
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    fail "$c missing <depend> entr(y/ies) across $unique_hits distinct include top-level names"
    verbose_dump 30 <"$tmp_fail"
  else
    pass "all cross-package #includes have a matching <depend> ($unique_hits distinct top-level names)"
  fi
  rm -f "$tmp_fail" "$tmp_hits"
}

# -----------------------------------------------------------------------------
# Section P: launch dict-parameter keys cross-referenced against declare_parameter
# -----------------------------------------------------------------------------
# For every Node(package=P, executable=E, parameters=[{...}]) call in a launch
# file, verify every dict key in parameters is declared somewhere in P's
# source tree as declare_parameter("key"...) or via _declare_or_get<T>(node, "key"...).
check_p_launch_param_keys() {
  section "P. Launch dict-parameter keys declared in target package source"
  local launch_files
  launch_files="$(list_launch_py_files "$REPO_ROOT")"
  if [[ -z "$launch_files" ]]; then
    skip "no launch files found"
    return
  fi

  # Build pkg_name -> package.xml path map once.
  local -A PKG_XML_BY_NAME
  local xmls_all
  xmls_all="$(list_package_xmls "$REPO_ROOT")"
  while IFS= read -r xml; do
    [[ -z "$xml" ]] && continue
    local nm
    nm="$(package_xml_name "$xml")"
    [[ -z "$nm" ]] && continue
    PKG_XML_BY_NAME["$nm"]="$xml"
  done <<<"$xmls_all"

  # Build pkg_name -> declared keys set (newline-separated; leading/trailing LF)
  local -A PKG_DECLARED
  local pkg
  for pkg in "${MANAGED_PKGS[@]}"; do
    local pkg_xml="${PKG_XML_BY_NAME[$pkg]:-}"
    [[ -z "$pkg_xml" ]] && continue
    local pkg_dir
    pkg_dir="$(dirname "$pkg_xml")"

    local tmp_keys
    tmp_keys="$(mktemp)"
    # Walk source and script files. For C++ we flatten with `tr` so that
    # multi-line `declare_parameter<T>(\n "key", ...)` calls are caught.
    find "$pkg_dir" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.hpp' -o -name '*.py' \) 2>/dev/null \
      | while IFS= read -r sf; do
          [[ -z "$sf" ]] && continue
          if [[ "$sf" == *.py ]]; then
            strip_py_comments <"$sf" \
              | tr '\n' ' ' \
              | grep -oE 'declare_parameter[[:space:]]*\([[:space:]]*["'\''][^"'\'']+["'\'']' \
              | sed -E 's/.*["'\'']([^"'\'']+)["'\''].*/\1/' >>"$tmp_keys" || true
          else
            flat="$(strip_cxx_comments <"$sf" | tr '\n' ' ')"
            printf '%s\n' "$flat" \
              | grep -oE 'declare_parameter[[:space:]]*(<[^>]*>)?[[:space:]]*\([[:space:]]*"[^"]+"' \
              | sed -E 's/.*"([^"]+)".*/\1/' >>"$tmp_keys" || true
            # Also capture _declare_or_get<T>(node, "key", default) wrapper form.
            printf '%s\n' "$flat" \
              | grep -oE '_declare_or_get[[:space:]]*<[^>]*>[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' \
              | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >>"$tmp_keys" || true
            # Also capture the repo's ad-hoc wrappers:
            #   get_param_or(node, "key", var, default)
            #   declare_parameter_if_not_declared(node, "key", ...)
            printf '%s\n' "$flat" \
              | grep -oE '\bget_param_or[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' \
              | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >>"$tmp_keys" || true
            printf '%s\n' "$flat" \
              | grep -oE '\bdeclare_parameter_if_not_declared[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' \
              | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >>"$tmp_keys" || true
          fi
        done
    local joined
    joined=$'\n'"$(LC_ALL=C sort -u "$tmp_keys" 2>/dev/null | awk 'NF')"$'\n'
    PKG_DECLARED["$pkg"]="$joined"
    rm -f "$tmp_keys"
  done

  local tmp_fail
  tmp_fail="$(mktemp)"
  local nodes_analyzed=0

  while IFS= read -r lf; do
    [[ -z "$lf" ]] && continue
    # Slurp the file and find Node(...) blocks up to the matching ')'.
    # We use a coarse `Node(...)` token match by slurping then greedy-matching
    # up to a reasonable bound, same idiom as section L. We accept the first
    # top-level ')' via grep -oE '[^)]*\)'.
    local joined
    joined="$(tr '\n' ' ' <"$lf")"
    # Extract each Node( ... ) block. Node must not be preceded by a word char
    # (avoids ComposableNode / LifecycleNode).
    local blocks
    blocks="$(printf "%s" "$joined" \
      | grep -oE "(^|[^A-Za-z0-9_])Node[[:space:]]*\([^)]{0,2000}\)" 2>/dev/null || true)"
    [[ -z "$blocks" ]] && continue
    while IFS= read -r blk; do
      [[ -z "$blk" ]] && continue
      local pkg exe
      pkg="$(printf "%s" "$blk" | grep -oE "package[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" \
              | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")"
      exe="$(printf "%s" "$blk" | grep -oE "executable[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" \
              | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")"
      [[ -z "$pkg" ]] && continue
      if ! is_managed_pkg "$pkg"; then
        continue
      fi
      nodes_analyzed=$((nodes_analyzed + 1))
      # Extract parameters=[...] list. Take up to the first ']' or end.
      local plist
      plist="$(printf "%s" "$blk" \
                | grep -oE "parameters[[:space:]]*=[[:space:]]*\[[^]]*\]" \
                | head -1 || true)"
      [[ -z "$plist" ]] && continue
      # Pull every quoted string that's followed by a ':' (dict key). Handle
      # both single- and double-quoted forms.
      local keys
      keys="$(printf "%s" "$plist" \
                | grep -oE "['\"][A-Za-z_][A-Za-z0-9_.]*['\"][[:space:]]*:" \
                | sed -E "s/['\"]([^'\"]+)['\"].*/\1/" \
                | LC_ALL=C sort -u)"
      [[ -z "$keys" ]] && continue
      local declared="${PKG_DECLARED[$pkg]:-}"
      while IFS= read -r key; do
        [[ -z "$key" ]] && continue
        # Skip resource-path-like keys (shouldn't appear as dict keys, but be safe).
        case "$key" in
          /*|*.yaml|*.yml|*.launch.py) continue ;;
        esac
        if ! printf '%s' "$declared" | grep -qxF "$key"; then
          printf "%s: Node(package='%s', executable='%s') parameters=[{'%s': ...}] but no declare_parameter(\"%s\", ...) found in %s source tree\n" \
            "$lf" "$pkg" "$exe" "$key" "$key" "$pkg" >>"$tmp_fail"
        fi
      done <<<"$keys"
    done <<<"$blocks"
  done <<<"$launch_files"

  if [[ -s "$tmp_fail" ]]; then
    local c
    LC_ALL=C sort -u "$tmp_fail" -o "$tmp_fail"
    c="$(wc -l <"$tmp_fail" | tr -d ' ')"
    fail "$c undeclared launch parameter key(s) across $nodes_analyzed Node(...) call(s)"
    verbose_dump 30 <"$tmp_fail"
  else
    pass "all launch dict-parameter keys are declared in their target package ($nodes_analyzed Node calls analyzed)"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section Q: Shell script syntax sweep
# -----------------------------------------------------------------------------
# Run `bash -n` on every .sh/.bash file under autonomy_* and tests/. Any
# syntactically-invalid script fails the check. If `bash -n` cannot run
# (restricted sandbox), the section reports SKIP.
check_q_shell_syntax() {
  section "Q. Shell script syntax (bash -n)"
  local scripts
  scripts="$(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" "$REPO_ROOT/tests" \
    -type f \( -name '*.sh' -o -name '*.bash' \) 2>/dev/null | LC_ALL=C sort || true)"
  if [[ -z "$scripts" ]]; then
    skip "no shell scripts found"
    return
  fi

  # Capability probe: try bash -n on a trivially-correct tiny script.
  local probe
  probe="$(mktemp)"
  printf '#!/usr/bin/env bash\nexit 0\n' >"$probe"
  if ! bash -n "$probe" 2>/dev/null; then
    rm -f "$probe"
    skip "bash -n not usable in this environment"
    return
  fi
  rm -f "$probe"

  local tmp_fail
  tmp_fail="$(mktemp)"
  local count=0
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    count=$((count + 1))
    if ! bash -n "$f" 2>>"$tmp_fail"; then
      printf "  [syntax error in] %s\n" "$f" >>"$tmp_fail"
    fi
  done <<<"$scripts"

  if [[ -s "$tmp_fail" ]]; then
    local c
    c="$(grep -c '' "$tmp_fail" 2>/dev/null || echo 0)"
    fail "bash -n reported errors in shell scripts"
    verbose_dump 30 <"$tmp_fail"
  else
    pass "all $count shell script(s) parse cleanly with bash -n"
  fi
  rm -f "$tmp_fail"
}

# -----------------------------------------------------------------------------
# Section R: Cross-package PathJoinSubstitution target resolution
# -----------------------------------------------------------------------------
# For every *.launch.py, every
#   PathJoinSubstitution([FindPackageShare('<pkg>'), 'seg1', 'seg2', ...])
# where <pkg> is one of the 22 managed packages in this repo must resolve
# to a file that exists in <pkg>'s source tree. This catches launch files
# whose config / rviz / sub-launch path references went stale during the
# port (e.g. a config renamed on master but still referenced under the
# old name in a launch file, or an upstream reference that simply never
# pointed at a real file).
#
# Carve-outs:
#   * External packages (FindPackageShare('X') where X is not in the
#     22-package managed map) are skipped — we cannot introspect their
#     share/ trees.
#   * Directory-only targets (no file extension in the last segment) are
#     skipped — those resolve at runtime by convention.
#   * msckf_calib.yaml (and msckf_calib_auto_generated.yaml) are carved out:
#     the upstream workflow expects users to run msckf_calib_gen to generate
#     these files before launch; the reference is "intentionally missing".
#   * mapper_3d.yaml and tracker_params_mp_3d.yaml are carved out: referenced
#     by polypixel_full_sim.launch.py in its IfCondition(use_3d) branch, but
#     neither file was ever shipped with map_plan_launch / control_launch on
#     master or on feature/integrate_lidar_3d_planner_default (pre-existing
#     upstream bug). Default path (use_3d=false) resolves fine; use_3d:=true
#     will fail at launch with a file-not-found error until someone ships
#     real 3D tuning configs.
check_r_pathjoinsub_resolve() {
  section "R. Cross-package PathJoinSubstitution targets resolve"
  #
  # For every *.launch.py, every cross-package file reference whose resolved
  # package is one of the 22 managed packages in this repo must point at a
  # file that exists in that package's source tree. Four reference forms
  # are handled:
  #   1. PathJoinSubstitution([FindPackageShare('X'), 'seg1', 'seg2', ...])
  #   2. PathJoinSubstitution([<var>, 'seg1', ...]) where earlier:
  #        <var> = get_package_share_directory('X')   OR
  #        <var> = FindPackageShare('X')
  #   3. os.path.join(get_package_share_directory('X'), 'seg1', ...)
  #   4. os.path.join(<var>, 'seg1', ...) with the same var bindings as (2).
  # External packages (where X is not in the managed map) and directory-only
  # targets (no '.' in the last segment) are silently skipped, as is the
  # msckf_calib.yaml carve-out.
  if ! command -v awk >/dev/null 2>&1; then
    skip "awk not available"
    return
  fi

  local pkgmap missing rows awkfile filelist
  pkgmap="$(mktemp)"
  missing="$(mktemp)"
  rows="$(mktemp)"
  awkfile="$(mktemp)"
  filelist="$(mktemp)"

  # Build pkg_name -> pkg_root map from every managed package.xml.
  local xml pname pdir
  while IFS= read -r xml; do
    [[ -z "$xml" ]] && continue
    pname="$(package_xml_name "$xml")"
    [[ -z "$pname" ]] && continue
    pdir="$(dirname "$xml")"
    printf "%s\t%s\n" "$pname" "$pdir" >>"$pkgmap"
  done < <(find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
            -type f -name 'package.xml' 2>/dev/null)

  # Known carve-outs:
  #   msckf_calib.yaml                : output of msckf_calib_gen (legacy name).
  #   msckf_calib_auto_generated.yaml : output of msckf_calib_gen (current default).
  #   mapper_3d.yaml,
  #   tracker_params_mp_3d.yaml       : referenced by polypixel_full_sim.launch.py
  #                                     in its IfCondition(use_3d) branch. Neither
  #                                     file was ever shipped with map_plan_launch
  #                                     / control_launch on master or on
  #                                     feature/integrate_lidar_3d_planner_default
  #                                     — pre-existing upstream bug. Launch with
  #                                     use_3d:=true fails with a file-not-found
  #                                     error until someone ships real 3D tuning.
  local carve=$'msckf_calib.yaml\nmsckf_calib_auto_generated.yaml\nmapper_3d.yaml\ntracker_params_mp_3d.yaml'

  # Write the awk extractor to a temp file (the program is long and quotes
  # both ' and ", so embedding it in a single-quoted bash string is ugly).
  cat >"$awkfile" <<'R_AWK_EOF'
# Input: one or more .launch.py files as arguments.
# Produces a pipe-separated row per resolved managed-package reference:
#   rel_path|pkg|seg1/seg2/...|absolute_target_path
# Skips references whose pkg is not in pkg_root (external packages),
# references whose last segment has no '.' (directory targets), and
# references whose basename appears in `carve`. Handles 4 reference
# forms documented in the bash caller.

BEGIN {
    if (pkgmap_file != "") {
        while ((getline line < pkgmap_file) > 0) {
            np = split(line, parts, "\t")
            if (np >= 2 && parts[1] != "") pkg_root[parts[1]] = parts[2]
        }
        close(pkgmap_file)
    }
    if (carve != "") {
        nc = split(carve, carr, /[\r\n]+/)
        for (cc = 1; cc <= nc; cc++) if (carr[cc] != "") carved[carr[cc]] = 1
    }
    buf = ""
    prevfile = ""
}

FNR == 1 && NR > 1 {
    if (prevfile != "") process(prevfile, buf)
    buf = ""
}

{ buf = buf $0 " "; prevfile = FILENAME }

END {
    if (buf != "" && prevfile != "") process(prevfile, buf)
}

function process(fname, flat,
                 s, nvars, i, k, R_s, R_l, chunk, name, pkg,
                 n, rem, p1, p2, startrel, taglen, start, j, depth, in_sq, in_dq, c,
                 span, span_begin, span_end, tmp, head, ident,
                 clean, segs, s2, qq, inner, base, rel, q, m, p)
{
    gsub(/[\n\r\t]+/, " ", flat)
    gsub(/  +/, " ", flat)

    # Pass 1: var bindings name = (FindPackageShare|get_package_share_directory)("X")
    delete varname
    delete varpkg
    nvars = 0
    s = flat
    while (match(s, /[A-Za-z_][A-Za-z0-9_]*[ ]*=[ ]*(FindPackageShare|get_package_share_directory)[ ]*\([ ]*["'\''][^"'\'']+["'\'']/)) {
        R_s = RSTART; R_l = RLENGTH
        chunk = substr(s, R_s, R_l)
        name = ""
        if (match(chunk, /^[A-Za-z_][A-Za-z0-9_]*/) > 0) name = substr(chunk, RSTART, RLENGTH)
        pkg = ""
        if (match(chunk, /["'\''][^"'\'']+["'\'']/) > 0) pkg = substr(chunk, RSTART + 1, RLENGTH - 2)
        if (name != "" && pkg != "") {
            nvars++
            varname[nvars] = name
            varpkg[nvars] = pkg
        }
        s = substr(s, R_s + R_l)
    }

    # Pass 2: for each PathJoinSubstitution(...) or os.path.join(...) span,
    # resolve pkg + segments and emit.
    n = length(flat)
    i = 1
    while (i <= n) {
        rem = substr(flat, i)
        p1 = index(rem, "PathJoinSubstitution(")
        p2 = index(rem, "os.path.join(")
        if (p1 == 0 && p2 == 0) break
        if (p1 == 0)              { startrel = p2; taglen = length("os.path.join(") }
        else if (p2 == 0)         { startrel = p1; taglen = length("PathJoinSubstitution(") }
        else if (p1 < p2)         { startrel = p1; taglen = length("PathJoinSubstitution(") }
        else                      { startrel = p2; taglen = length("os.path.join(") }

        start = i + startrel - 1
        j = start + taglen
        depth = 1; in_sq = 0; in_dq = 0
        while (j <= n && depth > 0) {
            c = substr(flat, j, 1)
            if (in_sq) {
                if (c == "\\" && j < n) { j += 2; continue }
                if (c == "'") in_sq = 0
                j++; continue
            }
            if (in_dq) {
                if (c == "\\" && j < n) { j += 2; continue }
                if (c == "\"") in_dq = 0
                j++; continue
            }
            if (c == "'")  { in_sq = 1; j++; continue }
            if (c == "\"") { in_dq = 1; j++; continue }
            if (c == "(")  { depth++; j++; continue }
            if (c == ")")  { depth--; j++; continue }
            j++
        }
        span_begin = start + taglen
        span_end = j - 2
        if (span_end < span_begin) { i = j; continue }
        span = substr(flat, span_begin, span_end - span_begin + 1)

        pkg = ""
        tmp = span
        if (match(tmp, /FindPackageShare[ ]*\([ ]*["'\''][^"'\'']+["'\'']/)) {
            chunk = substr(tmp, RSTART, RLENGTH)
            if (match(chunk, /["'\''][^"'\'']+["'\'']/) > 0) pkg = substr(chunk, RSTART + 1, RLENGTH - 2)
        }
        if (pkg == "" && match(tmp, /get_package_share_directory[ ]*\([ ]*["'\''][^"'\'']+["'\'']/)) {
            chunk = substr(tmp, RSTART, RLENGTH)
            if (match(chunk, /["'\''][^"'\'']+["'\'']/) > 0) pkg = substr(chunk, RSTART + 1, RLENGTH - 2)
        }
        if (pkg == "") {
            head = span
            sub(/^[ ]+/, "", head)
            sub(/^\[[ ]*/, "", head)
            if (match(head, /^[A-Za-z_][A-Za-z0-9_]*/) > 0) {
                ident = substr(head, RSTART, RLENGTH)
                for (k = 1; k <= nvars; k++) if (varname[k] == ident) { pkg = varpkg[k]; break }
            }
        }

        if (pkg != "" && (pkg in pkg_root)) {
            clean = span
            gsub(/FindPackageShare[ ]*\([ ]*["'\''][^"'\'']+["'\''][ ]*\)/, " ", clean)
            gsub(/get_package_share_directory[ ]*\([ ]*["'\''][^"'\'']+["'\''][ ]*\)/, " ", clean)
            segs = ""
            s2 = clean
            while (match(s2, /["'\''][^"'\'']*["'\'']/)) {
                R_s = RSTART; R_l = RLENGTH
                qq = substr(s2, R_s, R_l)
                inner = substr(qq, 2, R_l - 2)
                if (inner != "") {
                    if (segs == "") segs = inner
                    else segs = segs "/" inner
                }
                s2 = substr(s2, R_s + R_l)
            }
            if (segs != "") {
                base = segs
                if (index(base, "/") > 0) {
                    q = 0
                    for (m = length(base); m >= 1; m--) if (substr(base, m, 1) == "/") { q = m; break }
                    if (q > 0) base = substr(base, q + 1)
                }
                # skip directory-only and carved-out basenames
                if (index(base, ".") == 0) { i = j; continue }
                if (base in carved)        { i = j; continue }
                rel = fname
                if (repo_root != "") {
                    p = index(rel, repo_root)
                    if (p == 1) rel = substr(rel, length(repo_root) + 2)
                }
                print rel "|" pkg "|" segs "|" pkg_root[pkg] "/" segs
            }
        }
        i = j
    }
}
R_AWK_EOF

  # Single awk invocation processes every launch file in the managed roots.
  find "$REPO_ROOT/autonomy_core" "$REPO_ROOT/autonomy_real" "$REPO_ROOT/autonomy_sim" \
      -type f -name '*.launch.py' 2>/dev/null >"$filelist"
  if [[ -s "$filelist" ]]; then
    # shellcheck disable=SC2046
    awk -v pkgmap_file="$pkgmap" -v carve="$carve" -v repo_root="$REPO_ROOT" \
        -f "$awkfile" $(cat "$filelist") >"$rows" 2>/dev/null
  fi

  local checked=0 rel pkg path absfile
  while IFS='|' read -r rel pkg path absfile; do
    [[ -z "$rel" ]] && continue
    checked=$((checked + 1))
    if [[ ! -e "$absfile" ]]; then
      printf "%s|%s|%s\n" "$rel" "$pkg" "$path" >>"$missing"
    fi
  done <"$rows"

  if [[ -s "$missing" ]]; then
    local miss_count
    miss_count="$(awk 'END { print NR }' "$missing")"
    fail "$miss_count of $checked cross-package path-reference target(s) missing in managed packages"
    awk -F'|' '{ printf "         %s: pkg=%s -> %s (not found)\n", $1, $2, $3 }' "$missing" \
      | verbose_dump 20
  else
    pass "all $checked cross-package path-reference target(s) resolve"
  fi

  rm -f "$pkgmap" "$missing" "$rows" "$awkfile" "$filelist"
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
  check_o_include_depends
  check_p_launch_param_keys
  check_q_shell_syntax
  check_r_pathjoinsub_resolve
  check_rg1_dynamic_reconfigure_cfg
  check_rg2_package_readme

  print_summary
  [[ $FAILED -gt 0 ]] && exit 1 || exit 0
}

main "$@"
