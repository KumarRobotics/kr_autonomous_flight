# kr_autonomous_flight — ROS2 Jazzy Migration Report

> **Branch:** `ros2_dev`
> **Base branch:** `feature/integrate_lidar_3d_planner_default` (ROS1 Noetic, Ubuntu 20.04)
> **Target platform:** ROS2 Jazzy Jalisco on Ubuntu 24.04 LTS
> **Status of the static test suite:** *(run `bash tests/static/check_ros2_port.sh` to get the live count; the pytest mirror at `tests/python/` runs the same set of checks structured for CI)*

This report documents the ROS1 → ROS2 port of the entire `kr_autonomous_flight` stack. It is intended as a reference for developers picking up the open follow-up items and for reviewers auditing the port.

---

## 1. Migration scope

All 22 in-tree packages were migrated. The port is strictly local: every package now builds with `ament_cmake` or `ament_python`, uses `rclcpp` / `rclpy`, and ships a Python (`*.launch.py`) launch file where relevant. External repositories pulled in via `vcs import` are **not** part of this port and are tracked separately (see Section 8).

**Headline numbers**

| Item | Count |
|:---|---:|
| In-tree packages migrated | 22 |
| Launch-only packages (no C++, no Python nodes) | 8 |
| `ament_cmake` packages | 20 |
| `ament_python` packages | 2 |
| C++ packages with compiled targets | 12 |
| Nodelet → `rclcpp_components` conversions | 2 |
| `actionlib` → `rclcpp_action` servers ported | 7 |
| Pluginlib tracker plugins preserved | 4 |
| `.action` interface files (now `rosidl_generate_interfaces`) | 7 |
| XML `.launch` → Python `.launch.py` | 77 |
| Dockerfiles ported (Ubuntu 20.04/Noetic → 24.04/Jazzy) | 9 |
| GitHub Actions docker-build workflows updated | 7 |
| `external_*.yaml` dependency manifests (TODO comments added) | 4 |
| Python scripts ported from `rospy` to `rclpy` | 9 |

**Full package inventory**

| # | Package | Path | Build type | Role |
|---:|:---|:---|:---|:---|
| 1 | `client_launch` | `autonomy_core/client/client_launch` | ament_cmake | Launch-only: client GUI bring-up |
| 2 | `rqt_quadrotor_safety` | `autonomy_core/client/rqt_quadrotor_safety` | ament_python | rqt plugin: safety / arm / disarm GUI |
| 3 | `control_launch` | `autonomy_core/control/control_launch` | ament_cmake | Launch-only: trackers manager bring-up |
| 4 | `estimation_launch` | `autonomy_core/estimation/estimation_launch` | ament_cmake | Launch-only: UKF / MSCKF bring-up |
| 5 | `fla_ukf` | `autonomy_core/estimation/fla_ukf` | ament_cmake | UKF state estimator (was a nodelet) |
| 6 | `mavros_interface` | `autonomy_core/interface/mavros_interface` | ament_cmake | SO3 command → MAVROS bridge (was a nodelet) |
| 7 | `px4_interface_launch` | `autonomy_core/interface/px4_interface_launch` | ament_cmake | Launch-only: PX4 + MAVROS bring-up |
| 8 | `action_planner` | `autonomy_core/map_plan/action_planner` | ament_cmake | Local + global replan servers (rclcpp_action) |
| 9 | `coverage_utils` | `autonomy_core/map_plan/coverage_utils` | ament_cmake | Polygon coverage helper node |
| 10 | `jps3d` | `autonomy_core/map_plan/jps3d` | ament_cmake | JPS graph planner library |
| 11 | `mapper` | `autonomy_core/map_plan/mapper` | ament_cmake | Voxel occupancy mapper |
| 12 | `map_plan_launch` | `autonomy_core/map_plan/map_plan_launch` | ament_cmake | Launch-only: mapper + planner bring-up |
| 13 | `mpl` | `autonomy_core/map_plan/mpl` | ament_cmake | Motion primitive library wrapper |
| 14 | `traj_opt_ros` | `autonomy_core/map_plan/traj_opt_ros` | ament_cmake | Trajectory optimization ROS glue |
| 15 | `action_trackers` | `autonomy_core/state_machine/action_trackers` | ament_cmake | 4 pluginlib tracker plugins + 6 actions |
| 16 | `state_machine_core` | `autonomy_core/state_machine/state_machine_core` | ament_cmake | `local_global_replan_server` + smach scripts |
| 17 | `state_machine_launch` | `autonomy_core/state_machine/state_machine_launch` | ament_cmake | Launch-only: full autonomy bring-up |
| 18 | `real_experiment_launch` | `autonomy_real/real_experiment_launch` | ament_cmake | Launch-only: on-robot bring-up |
| 19 | `gazebo_utils` | `autonomy_sim/gazebo_sim/gazebo_utils` | ament_cmake | Launch-only: Gazebo simulation bring-up |
| 20 | `dcist_utils` | `autonomy_sim/unity_sim/dcist_utils` | ament_cmake | odom→tf, MSCKF calib-gen, Unity sim launches |
| 21 | `fake_lidar` | `autonomy_sim/unity_sim/fake_lidar` | ament_cmake | Depth → simulated LIDAR point cloud |
| 22 | `fake_sloam` | `autonomy_sim/unity_sim/fake_sloam` | ament_python | Publish fake SLOAM odom → map TF |

---

## 2. Per-package changes

### 2.1 Launch-only packages (8 packages)

`client_launch`, `control_launch`, `estimation_launch`, `px4_interface_launch`, `map_plan_launch`, `state_machine_launch`, `real_experiment_launch`, and `gazebo_utils` ship only launch files, config YAMLs, and (occasionally) RViz config. All 8 were migrated with the same recipe:

- **`package.xml`:** already `format="3"`; swapped `<buildtool_depend>catkin</buildtool_depend>` for `ament_cmake` and the `<export><build_type>` tag set to `ament_cmake`.
- **`CMakeLists.txt`:** stripped `catkin_package()`, replaced with a minimal `find_package(ament_cmake REQUIRED)` + `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})` + `ament_package()` scaffold.
- **Launch files:** every `*.launch` XML was rewritten as a Python `launch.LaunchDescription` in `*.launch.py`. This was the largest mechanical change — 77 launch files in total. Each file exposes the same `DeclareLaunchArgument` names that the ROS1 `<arg name="…" default="…" />` declared.
- **Includes:** `<include file="$(find pkg)/launch/foo.launch"/>` was rewritten as `IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("pkg"), "launch", "foo.launch.py")))`.
- **`<node>` tags** were rewritten as `launch_ros.actions.Node(...)` with `parameters=[{...}]` for inline `<param>` and `parameters=[config_yaml]` for `<rosparam command="load" file="…"/>`.

**Per-package notes**

- **`real_experiment_launch`:** 4 legacy calibration YAMLs in `config/` (VN100, OVC3 intrinsics/extrinsics, Ouster mounts) are loaded by the dcist `msckf_calib_gen` binary via `YAML::LoadFile` rather than via the ROS parameter server. They were left as-is — no parameter-server rewrite was necessary. See Section 8 for the follow-up.
- **`gazebo_utils`:** `simulation.launch.py` still references `mrsl_quadrotor_launch/gazebo.launch.py`, which is a Gazebo Classic launch. The Gazebo Harmonic migration of that upstream is outstanding; see Section 8.
- **Dangling includes:** `full_autonomy_ca_trip.launch.py` (in `special_purposes/`) and `full_autonomy_atl.launch.py` (in `uav_ugv_localization/`) include a `throttle_imu.launch` file that does not exist in the source tree. This is a **pre-existing bug** on the master branch and was intentionally preserved rather than silently papered over.

### 2.2 `rqt_quadrotor_safety` (ament_python)

rqt Qt plugin that drives the client safety GUI (arm / disarm / takeoff / land buttons). Migration:

- `package.xml`: `<buildtool_depend>catkin</buildtool_depend>` → `ament_python`.
- `CMakeLists.txt` removed; added `setup.py`, `setup.cfg`, and `resource/<pkg>` marker file per ROS2 Python package conventions.
- `src/rqt_quadrotor_safety/*.py`: `rospy` → `rclpy`; `rospy.get_param` → `self.node.declare_parameter(...).value`; `rospy.Publisher(...).publish(...)` → `self.node.create_publisher(...).publish(...)`; `rospy.Time.now()` → `self.node.get_clock().now().to_msg()`.
- `plugin.xml`: rewritten as the ROS2 `rqt_gui` plugin format (no `_ros1`-only tags).
- `resource/*.ui`: Qt Designer files left untouched.

### 2.3 `fla_ukf` (nodelet → component)

UKF state estimator, originally a Noetic `nodelet::Nodelet` subclass registered via `PLUGINLIB_EXPORT_CLASS(..., nodelet::Nodelet)`. Migration:

- C++: replaced `#include <nodelet/nodelet.h>` with `#include <rclcpp_components/register_node_macro.hpp>`, reshaped `FLAUKFNodelet : public nodelet::Nodelet` as an `rclcpp::Node` subclass, and registered with `RCLCPP_COMPONENTS_REGISTER_NODE(fla_ukf::FLAUKFNodelet)`.
- `ros::NodeHandle nh;` / `ros::NodeHandle pnh;` in `onInit()` replaced with a constructor that takes `const rclcpp::NodeOptions &` and a body that calls `this->declare_parameter(...)`.
- `tf::TransformListener` → `tf2_ros::Buffer` + `tf2_ros::TransformListener`.
- `ros::Time::now()` → `this->get_clock()->now()`.
- `ros::spin()` no longer needed — the component is loaded by `component_container` in the launch file.
- `CMakeLists.txt` now builds a `SHARED` library with `ament_target_dependencies(... rclcpp rclcpp_components ...)` and calls `rclcpp_components_register_nodes(${PROJECT_NAME}_component "fla_ukf::FLAUKFNodelet")`.
- Launch: `ukf.launch.py` now loads the component via `ComposableNodeContainer` with `ComposableNode(package="fla_ukf", plugin="fla_ukf::FLAUKFNodelet", ...)`.

### 2.4 `mavros_interface` (nodelet → component)

MAVROS bridge: translates kr SO3 commands into MAVROS actuator / attitude setpoint messages. Same nodelet → component pattern as `fla_ukf`. The component is `mavros_interface::SO3CmdToMavros`, loaded by `SO3_command_to_mavros.launch.py`.

### 2.5 `jps3d` / `mpl` / `traj_opt_ros`

Three library-only map/plan packages. No nodes; downstream packages link against them.

- `package.xml` buildtool swap, `CMakeLists.txt` rewritten to `find_package(ament_cmake REQUIRED)` + per-dep `find_package`, `add_library(... SHARED ...)`, `ament_target_dependencies(...)`, `ament_export_libraries(...)`, `ament_export_dependencies(...)`, `ament_package()`.
- `traj_opt_ros` previously used `tf::Quaternion`; replaced with `tf2::Quaternion` + `tf2_geometry_msgs` conversions.
- `ROS_INFO` → `RCLCPP_INFO(rclcpp::get_logger("traj_opt_ros"), ...)` in logging-only spots where no node handle was available.

### 2.6 `mapper`

Voxel occupancy mapper that exports a `VoxelMap` library consumed by `action_planner` and `state_machine_core`.

- `ros::NodeHandle` use moved to a constructor-injected `rclcpp::Node::SharedPtr`.
- `ros::Publisher` / `ros::Subscriber` replaced with templated `rclcpp::Publisher<T>::SharedPtr` / `rclcpp::Subscription<T>::SharedPtr`.
- `PointCloud2` callbacks: `const sensor_msgs::PointCloud2ConstPtr &` → `const sensor_msgs::msg::PointCloud2::ConstSharedPtr &`.
- `message_filters` TF-synced subscribers updated to the `tf2_ros::MessageFilter<T>` API.
- `mapper.launch.py` and `mapper_standalone.launch.py` converted to Python launches.

### 2.7 `action_planner` (actionlib → rclcpp_action)

Local + global replanner servers. Big surface area — the largest individual migration in the tree.

- `LocalPlanServer` and `GlobalPlanServer`: both rewritten from `actionlib::SimpleActionServer<…Action>` to `rclcpp_action::Server<…Action>` + `handle_goal` / `handle_cancel` / `handle_accepted` callbacks.
- Feedback / result publishing moved from `as_->publishFeedback(…)` to `goal_handle->publish_feedback(…)` / `goal_handle->succeed(…)`.
- `dynamic_reconfigure` was dropped (no ROS2 drop-in). The `cfg/ActionPlanner.cfg` file was deleted; the parameters it exposed are now plain `declare_parameter(...)` calls and must be edited via `ros2 param set` or the yaml file in `config/`. This is a **feature regression** tracked in Section 8.
- `ros::NodeHandle("~")` → `rclcpp::Node::make_shared("~…")` with node-private parameters declared in the constructor.
- All `ROS_*` log macros → `RCLCPP_*`.
- 2 `*_server.cpp` files + `planner_details.cpp` + supporting `data_conversions.cpp` / `primitive_ros_utils.cpp` were touched.
- Python scripts `publish_plantwopointaction.py`, `evaluate_traj.py`, `evaluate_traj_exp.py` (rospy → rclpy) — see Section 2.12.

### 2.8 `coverage_utils`

Polygon coverage planner helper. Minimal port: one executable (`coverage_utils_node`) and one shared library. No actionlib, no dynamic_reconfigure. Routine `rospy` → `rclcpp` mechanical swap.

### 2.9 `action_trackers` (pluginlib preserved + actionlib → rclcpp_action)

Four tracker plugins loaded at runtime by the upstream `kr_trackers_manager`:

- `LandTracker` (land_tracker.cpp)
- `TakeOffTracker` (take_off_tracker.cpp)
- `StoppingPolicy` (stopping_policy.cpp)
- `ActionTrajectoryTracker` (trajectory_tracker_upgraded.cpp)

Key points:

- **Pluginlib is preserved** — this is the correct ROS2 pattern for non-Node plugins loaded by a parent container. Each tracker still ends with `PLUGINLIB_EXPORT_CLASS(X, kr_trackers_manager::Tracker)` and the package still ships a `nodelet_plugin.xml` (name preserved for compatibility with the upstream manager's plugin index) installed via `pluginlib_export_plugin_description_file(kr_trackers_manager nodelet_plugin.xml)`.
- **Tracker `Initialize` signature** — all four plugins now take `const rclcpp::Node::SharedPtr & parent_nh` instead of the ROS1 `const ros::NodeHandle &`. This is an **assumption** about how the upstream `kr_trackers_manager` ROS2 port will expose the parent handle; if that port lands with `LifecycleNode::SharedPtr` or a custom context struct, each tracker needs a one-line signature tweak. See Section 8.
- **Three of the four trackers are also action servers** — `LandTracker`, `TakeOffTracker`, and `ActionTrajectoryTracker` implement `rclcpp_action::Server` inside `Initialize` so the replanner can command them asynchronously. `StoppingPolicy` has no action interface.
- **Action definitions:** 6 `.action` files (`Land`, `TakeOff`, `RunTrajectory`, `ShortRange`, `GoalDistance`, `RunPath`) are now generated via `rosidl_generate_interfaces(${PROJECT_NAME} action/...)` with `rosidl_default_generators` buildtool dependency. The executables link to the generated typesupport target via `rosidl_get_typesupport_target`.
- `traj_to_quad_cmd.cpp` (the shared utility library) was rewritten against `rclcpp::Time` / `builtin_interfaces::msg::Time`.

### 2.10 `state_machine_core`

The autonomy top-level: hosts `local_global_replan_server` and a set of smach-based Python scripts that drive mission state.

- C++: `local_global_replan_server.cpp` is now an `rclcpp_action::Server<Replan>` **plus** three `rclcpp_action::Client<…>` instances that call into `action_trackers` (`TakeOff`, `Land`, `ActionTrajectoryTracker`) and into the `action_planner` replanner. The full goal / feedback / cancel state machine was ported 1:1 from the ROS1 `actionlib::SimpleActionClient` API.
- `traj_opt_utils.cpp` rewritten against `rclcpp::Time` arithmetic.
- 1 `.action` file (`Replan.action`) generated via `rosidl_generate_interfaces`.
- **Python scripts (`main_state_machine.py`, `MainStates.py`, `Replanner.py`, `SwitchState.py`, `QuadTracker.py`, `Utils.py`):** the `rospy` → `rclpy` **surface** was ported (imports, publishers, subscribers, log macros, parameter access), but the actual state graph is still expressed in `smach` — which is ROS1-only. `smach_ros2` / `yasmin` are the two candidate ROS2 replacements. See Section 8.

### 2.11 `dcist_utils`

Unity simulation helper. Three small executables:

- `odom2tf` — publishes a `nav_msgs::msg::Odometry` as a `tf2` broadcast. Uses `tf2_ros::TransformBroadcaster`.
- `msckf_calib_gen` — generates MSCKF calibration YAML from a collection of transforms. Still uses `YAML::LoadFile` / `YAML::Emitter` for the legacy plain-dict calibration yamls.
- `playground` — scratch executable for manual testing. Minimal `rclcpp::Node`.

All three were mechanically ported. Launch files under `launch/sim/` were rewritten as `*.launch.py`.

### 2.12 Python scripts — `action_planner/scripts/` + `state_machine_core/scripts/`

Nine Python scripts were ported from `rospy` to `rclpy`:

**`action_planner/scripts/`:**
- `publish_plantwopointaction.py` — now an `rclpy.node.Node` + `rclpy_action.ActionClient` (formerly `actionlib.SimpleActionClient`).
- `evaluate_traj.py` — `rospy.init_node` → `rclpy.init` + `rclpy.create_node`; subscribers use `node.create_subscription`.
- `evaluate_traj_exp.py` — same pattern.

**`state_machine_core/scripts/`:**
- `main_state_machine.py` — entry point; boots the `smach` state machine. `rospy.init_node` → `rclpy.init`; the script still imports `smach` which is ROS1-only — see Section 8.
- `MainStates.py` — state definitions; each state's `execute(…)` method now receives an `rclpy` node handle rather than relying on the `rospy` global.
- `Replanner.py`, `SwitchState.py` — same mechanical pattern.
- `QuadTracker.py`, `Utils.py` — helper modules; `rospy.Time.now()`, `rospy.get_param`, `rospy.logger` all replaced with `rclpy` equivalents.

The `smach`-shaped state graph itself was **not** rewritten. Only the `rospy`→`rclpy` surface was touched. The follow-up is tracked in Section 8.

### 2.13 `fake_lidar`

Two small executables (`fake_lidar_node`, `fake_lidar_node2`) that synthesize a LIDAR point cloud from depth images. Routine `rclcpp::Node` + `image_transport` + `pcl_ros` port. No nodelets, no actions.

### 2.14 `fake_sloam` (ament_python)

One Python node that publishes a fake SLOAM odom → map transform. Minimal port: `rospy` → `rclpy`; `tf` → `tf2_ros.TransformBroadcaster`. Standard `setup.py` + `resource/<pkg>` layout.

---

## 3. Cross-cutting rules applied

### 3.1 C++ substitutions

| ROS1 | ROS2 Jazzy |
|:---|:---|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `ros::NodeHandle nh;` | `rclcpp::Node::SharedPtr node;` (constructor-injected) |
| `ros::NodeHandle("~")` | `declare_parameter(...)` on the private node |
| `ros::Publisher` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Subscriber` | `rclcpp::Subscription<T>::SharedPtr` |
| `ros::Timer` | `rclcpp::TimerBase::SharedPtr` |
| `ros::Rate r(10); r.sleep();` | `rclcpp::Rate r(10); r.sleep();` (in a loop spun by `rclcpp::spin_until_future_complete`) |
| `ros::Time::now()` | `node->get_clock()->now()` |
| `ros::Duration` | `rclcpp::Duration::from_seconds(...)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ROS_INFO(...)` / `ROS_WARN(...)` | `RCLCPP_INFO(node->get_logger(), ...)` / `RCLCPP_WARN(...)` |
| `nodelet::Nodelet` subclass | `rclcpp::Node` subclass + `RCLCPP_COMPONENTS_REGISTER_NODE` |
| `PLUGINLIB_EXPORT_CLASS(..., nodelet::Nodelet)` | `RCLCPP_COMPONENTS_REGISTER_NODE(...)` for Node-type plugins; keep `PLUGINLIB_EXPORT_CLASS` for non-Node tracker plugins |
| `actionlib::SimpleActionServer<T>` | `rclcpp_action::Server<T>` with `handle_goal` / `handle_cancel` / `handle_accepted` |
| `actionlib::SimpleActionClient<T>` | `rclcpp_action::Client<T>` + `async_send_goal` + `get_result_async` |
| `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| `tf::Quaternion` | `tf2::Quaternion` + `tf2_geometry_msgs` conversions |
| `tf::StampedTransform` | `geometry_msgs::msg::TransformStamped` |
| `sensor_msgs::PointCloud2ConstPtr` | `sensor_msgs::msg::PointCloud2::ConstSharedPtr` |
| `dynamic_reconfigure::Server<Config>` | *(dropped — no ROS2 drop-in; use `declare_parameter` + `add_on_set_parameters_callback`)* |

### 3.2 Python substitutions

| ROS1 | ROS2 Jazzy |
|:---|:---|
| `import rospy` | `import rclpy` / `from rclpy.node import Node` |
| `rospy.init_node("x")` | `rclpy.init(); node = rclpy.create_node("x")` |
| `rospy.Publisher(topic, T, queue_size=10)` | `node.create_publisher(T, topic, 10)` |
| `rospy.Subscriber(topic, T, cb)` | `node.create_subscription(T, topic, cb, 10)` |
| `rospy.Time.now()` | `node.get_clock().now().to_msg()` |
| `rospy.get_param("~foo", default)` | `node.declare_parameter("foo", default).value` |
| `rospy.loginfo(...)` | `node.get_logger().info(...)` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `rospy.Rate(hz)` | `node.create_rate(hz)` |
| `actionlib.SimpleActionClient` | `rclpy.action.ActionClient` |
| `import tf` | `import tf2_ros` + `tf_transformations` for Euler/quaternion math |
| `import smach` | *(unchanged — smach is ROS1-only; see Section 8)* |

### 3.3 `package.xml`

| ROS1 (format 2) | ROS2 Jazzy (format 3) |
|:---|:---|
| `<package format="2">` | `<package format="3">` |
| `<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` or `ament_python` |
| `<build_depend>roscpp</build_depend>` | `<depend>rclcpp</depend>` |
| `<build_depend>rospy</build_depend>` | `<depend>rclpy</depend>` |
| `<depend>actionlib</depend>` | `<depend>rclcpp_action</depend>` |
| `<depend>nodelet</depend>` | `<depend>rclcpp_components</depend>` |
| *(no action metadata)* | `<buildtool_depend>rosidl_default_generators</buildtool_depend>` + `<exec_depend>rosidl_default_runtime</exec_depend>` + `<member_of_group>rosidl_interface_packages</member_of_group>` |
| `<depend>tf</depend>` | `<depend>tf2</depend>` + `<depend>tf2_ros</depend>` + `<depend>tf2_geometry_msgs</depend>` |
| *(no `<export>` build_type)* | `<export><build_type>ament_cmake</build_type></export>` |

### 3.4 `CMakeLists.txt`

| ROS1 (catkin) | ROS2 Jazzy (ament) |
|:---|:---|
| `find_package(catkin REQUIRED COMPONENTS roscpp ...)` | `find_package(ament_cmake REQUIRED)` + per-dep `find_package(rclcpp REQUIRED)` etc. |
| `catkin_package(INCLUDE_DIRS ... LIBRARIES ... CATKIN_DEPENDS ...)` | `ament_export_include_directories(include)` + `ament_export_libraries(...)` + `ament_export_dependencies(...)` |
| `target_link_libraries(tgt ${catkin_LIBRARIES})` | `ament_target_dependencies(tgt rclcpp ...)` |
| `add_dependencies(tgt ${catkin_EXPORTED_TARGETS})` | *(no longer needed — handled by `ament_target_dependencies`)* |
| `add_message_files(...)` / `add_service_files(...)` / `add_action_files(...)` + `generate_messages(...)` | `rosidl_generate_interfaces(${PROJECT_NAME} action/... msg/... srv/... DEPENDENCIES ...)` + link via `rosidl_get_typesupport_target` |
| `install(TARGETS ... LIBRARY DESTINATION ${CATKIN_PACKAGE_LIBRARY_DESTINATION})` | `install(TARGETS ... ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)` |
| `install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})` | `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` |
| *(implicit)* | `ament_package()` at the end |

### 3.5 Launch files

| ROS1 (XML `.launch`) | ROS2 Jazzy (Python `.launch.py`) |
|:---|:---|
| `<launch>` root | `generate_launch_description()` returning `LaunchDescription([...])` |
| `<arg name="x" default="0.0"/>` | `DeclareLaunchArgument("x", default_value="0.0")` |
| `$(arg x)` | `LaunchConfiguration("x")` |
| `$(find pkg)` | `get_package_share_directory("pkg")` (from `ament_index_python.packages`) |
| `<node pkg="p" type="n" name="n" output="screen">` | `launch_ros.actions.Node(package="p", executable="n", name="n", output="screen", ...)` |
| `<param name="k" value="v"/>` | `parameters=[{"k": "v"}]` in the `Node(...)` call |
| `<rosparam command="load" file="path.yaml"/>` | `parameters=[path_to_yaml]` in the `Node(...)` call |
| `<include file="$(find pkg)/launch/foo.launch"/>` | `IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("pkg"), "launch", "foo.launch.py")))` |
| `<group ns="robot">` | `PushRosNamespace("robot")` action |
| `<group if="$(arg flag)">` | `GroupAction(condition=IfCondition(LaunchConfiguration("flag")), actions=[...])` |
| `<remap from="a" to="b"/>` | `remappings=[("a", "b")]` in the `Node(...)` call |

---

## 4. Infrastructure changes

### 4.1 Dockerfiles

Nine Dockerfiles were ported in a single `infra:` commit. The pattern was identical across all nine:

- **Base image:** `FROM ros:noetic-ros-base` / `FROM nvidia/cudagl:11.4.2-base-ubuntu20.04` → `FROM osrf/ros:jazzy-desktop-full`. The `osrf/ros:jazzy-desktop-full` base already provides the ROS2 apt source, `colcon`, `rosdep`, and the desktop-full metapackage, so the per-dockerfile setup is shorter than the ROS1 equivalent.
- **CUDA:** `nvidia/cudagl:11.4.2-base-ubuntu20.04` has been end-of-life'd by NVIDIA and there is no drop-in Jazzy + CUDA image. The base Dockerfile carries a top-of-file `TODO` banner explaining this — the Jazzy image covers graphics needs (rviz2, Gazebo Harmonic) but does **not** provide a CUDA runtime. Any follow-up that layers CUDA on top (e.g. for YOLO inference in the SLAM stack) needs to either stack a CUDA apt set on the Jazzy base, or switch to `nvidia/cuda:12.x-runtime-ubuntu24.04` and re-install ROS2 on top.
- **Tooling:** `python3-catkin-tools` → `python3-colcon-common-extensions`. `catkin build` → `colcon build --symlink-install`. Workspace path `/root/catkin_ws` → `/root/kr_ws`.
- **`rosdep`:** `(rosdep init || true) && rosdep update` kept verbatim; the apt key and source path differ between Noetic and Jazzy but `osrf/ros:jazzy-desktop-full` already configures them.
- **`entrypoint.sh`:** `source /opt/ros/noetic/setup.bash` → `source /opt/ros/jazzy/setup.bash`. Every `run.sh` / `build.sh` / `deploy*.sh` script in the repo tree was updated to match.

**Files touched:** `autonomy_core/base/Dockerfile`, `autonomy_core/{client,control,estimation,interface,map_plan,state_machine}/Dockerfile`, `autonomy_sim/Dockerfile`, `autonomy_sim/unity_sim/Dockerfile`.

### 4.2 GitHub Actions

Seven `docker-build-*.yaml` workflows in `.github/workflows/`:

- `docker-build-base.yaml`
- `docker-build-client.yaml`
- `docker-build-control.yaml`
- `docker-build-estimation.yaml`
- `docker-build-map-plan.yaml`
- `docker-build-sim.yaml`
- `docker-build-state-machine.yaml`

Each workflow was updated to:

- **Trigger branch:** `branches: [master]` → `branches: [ros2_dev]`.
- **Runner:** `runs-on: ubuntu-20.04` → `runs-on: ubuntu-24.04`.
- **Image tag:** `kumarrobotics/autonomy:<component>` → `kumarrobotics/autonomy:<component>-jazzy`.
- **File path:** the Dockerfile path is unchanged; only the tag suffix moved.

Lint workflows (`cpplint-reviewdog.yaml`, `pylint-reviewdog.yaml`, `shellcheck-reviewdog.yaml`) were left untouched.

### 4.3 `external_*.yaml`

Four vcstool manifests at the repo root:

- `external_all.yaml` — full dependency set.
- `external_real_robot.yaml` — on-robot driver set.
- `external_lidar_odometry.yaml` — LIDAR-only flight subset (faster-lio, ouster, msckf_vio).
- `external_coverage_planner.yaml` — polygon coverage planner add-on.

All four still reference **ROS1-only** upstream repos (`kr_mav_control`, `mrsl_quadrotor`, `ouster_example`, `msckf_vio`, `kr_planning_msgs`, `kr_trackers_manager`, `DecompROS`, `faster-lio`, `arl_unity_ros`, `polygon_coverage_ros`, etc.). Each file now carries a top-of-file **TODO** comment flagging that these need to be ported to their ROS2 equivalents before `vcs import < external_all.yaml` produces a ROS2-buildable workspace. The entries themselves are left in place so that a downstream maintainer can see what the dependency graph looked like before the port.

### 4.4 README

`README.md` was expanded from 107 lines (the original ROS1 stub that deferred to the wiki) to 460 lines covering:

- An `[!NOTE]` experimental banner warning that the ROS2 port is less tested than `master`.
- Full ROS2 Jazzy install + build-from-source + `colcon build` instructions.
- Docker workflow (per-component build, `-jazzy` image tags, NVIDIA container toolkit caveats).
- ROS1 bag → ROS2 bag conversion via `rosbags-convert`.
- Gazebo Harmonic simulation instructions + the "Gazebo Classic is gone" caveat.
- LIDAR+VIO + LIDAR-only bring-up sequences.
- Coverage experiments, real-robot deployment, troubleshooting, and frame conventions.
- Pointers to the ROS1 wiki pages that were **not** ported (hardware setup, calibration, preflight).

The `[!NOTE]` banner is left as-is and is **not** modified by this report.

### 4.5 Shell scripts

All in-tree `run_*.sh` / `entrypoint.sh` / `deploy*.sh` / `build*.sh` scripts under `autonomy_core/*/docker/` and `autonomy_sim/*/docker/` had their ROS1 command substitutions updated:

- `source /opt/ros/noetic/setup.bash` → `source /opt/ros/jazzy/setup.bash`
- `catkin build` → `colcon build --symlink-install`
- `/root/catkin_ws` → `/root/kr_ws` (where referenced)
- `roslaunch pkg file.launch` → `ros2 launch pkg file.launch.py` (where referenced inline)
- `rostopic` / `rosnode` / `rosservice` in convenience aliases → `ros2 topic` / `ros2 node` / `ros2 service`

---

## 5. What the static test suite enforces

`tests/static/check_ros2_port.sh` is a bash + ripgrep + awk runner with **no Python or ROS2 dependencies** — it runs anywhere with a POSIX shell. It is organized into 14 labeled sections (A–N); the pytest mirror at `tests/python/test_*.py` runs the same checks and is intended to be used as a CI gate. The `tests/integration/launch_smoke_test.sh` runtime check does require `ros2` on PATH but **does not** require a built workspace — it calls `ros2 launch --print-description` on every `*.launch.py` to verify the launch graph parses.

| Section | Check | Catches |
|:---:|:---|:---|
| A | `package.xml` build_type | Every in-tree `package.xml` is `format="3"` and has `<export><build_type>ament_cmake\|ament_python</build_type>` set. |
| B | `CMakeLists.txt` ament scaffold | Every `CMakeLists.txt` calls `find_package(ament_cmake REQUIRED)` and ends with `ament_package()`. No `catkin_package()` / `catkin REQUIRED COMPONENTS` remnants. |
| C | C++ header hygiene | No `#include <ros/ros.h>` / `<nodelet/nodelet.h>` / `<tf/transform_listener.h>` / `<actionlib/*>` / `<dynamic_reconfigure/*>` left in any in-tree `.cpp` / `.h`. |
| D | C++ API hygiene | No `ros::NodeHandle`, `ros::Publisher`, `ros::Time::now()`, `ROS_INFO`, `ROS_WARN`, etc. in stripped (comment-free) source. |
| E | Python `rospy` hygiene | No `import rospy` / `rospy.init_node` / `rospy.Publisher` / `rospy.Subscriber` / `rospy.Time.now()` / `rospy.get_param` in stripped Python source. |
| F | `tf` → `tf2` hygiene | No `import tf` (without the `2`) in Python; no `<include>tf/</include>` style C++ includes. |
| G | Launch files | Every `*.launch` XML is accompanied by a `*.launch.py`. No stray `<launch>` XML files remain in the tree that don't also have a Python twin. |
| H | `.action` interface generation | Every `.action` file lives in a package whose `CMakeLists.txt` calls `rosidl_generate_interfaces(... DEPENDENCIES ...)` and whose `package.xml` declares `rosidl_default_generators` + `rosidl_default_runtime` + `<member_of_group>rosidl_interface_packages</member_of_group>`. |
| I | Pluginlib plugins | Every package with a `PLUGINLIB_EXPORT_CLASS` call exports the plugin description via `pluginlib_export_plugin_description_file(...)` and the description XML lives in the package share directory. |
| J | Components | Every file with `RCLCPP_COMPONENTS_REGISTER_NODE` has a corresponding `rclcpp_components_register_nodes(... "...")` call in its `CMakeLists.txt`. |
| K | Docker hygiene | No `FROM nvidia/cudagl:*` or `FROM ros:noetic-*` in any in-tree Dockerfile; no `catkin build` / `source /opt/ros/noetic/setup.bash` left in any in-tree shell script or Dockerfile. |
| L | GitHub Actions | Every `docker-build-*.yaml` triggers off `branches: [ros2_dev]` and produces a `-jazzy` tag. |
| M | `external_*.yaml` TODOs | Every `external_*.yaml` manifest has a top-of-file TODO comment flagging that its entries are still ROS1-only. |
| N | Dangling launch includes | Every `IncludeLaunchDescription(...)` argument that references a local `*.launch.py` points at a file that actually exists. (The two known pre-existing `throttle_imu.launch` dangling references are tracked as expected failures — see Section 8.) |

The runner produces colorized `[PASS]` / `[FAIL]` / `[SKIP]` lines and a final summary of the form `N checks, P passed, F failed, S skipped`.

---

## 6. What has NOT been verified

The static checks catch the vast majority of the mechanical port mistakes, but a number of runtime-only items require a real ROS2 Jazzy workstation and are therefore **not** covered:

- **`colcon build` end-to-end.** Will fail until external dependencies (`kr_trackers_manager`, `kr_mav_control`, `kr_planning_msgs`, `DecompROS`, `faster-lio`, `msckf_vio`, `mrsl_quadrotor`, `polygon_coverage_ros`, etc.) are ported. See Section 8.
- **`rclcpp_action` round-trip tests.** The `LocalPlanServer` / `GlobalPlanServer` / `LandTracker` / `TakeOffTracker` / `ActionTrajectoryTracker` / `local_global_replan_server` goal/feedback/result flow needs to be exercised against live clients.
- **Tracker plugin load.** `kr_trackers_manager` needs to successfully `pluginlib::ClassLoader::createInstance` each of the four ported trackers and call `Initialize` on them with a real `rclcpp::Node::SharedPtr`.
- **Gazebo Harmonic bring-up.** `gazebo_utils/launch/full_sim.launch.py` still transitively references Gazebo Classic launch files through `mrsl_quadrotor_launch/gazebo.launch.py`.
- **`mapper` VoxelMap round-trip.** The voxel map serialization / deserialization and the `message_filters` TF-synced point-cloud ingest path haven't been exercised.
- **Full SLAM demo.** Playing a converted ROS2 bag through the LIDAR+VIO stack and watching the mapper + planner + state machine close a mission loop.
- **Client GUI.** `rqt_quadrotor_safety` renders and publishes but hasn't been tested against a real state machine that responds.
- **MAVROS bridge.** `mavros_interface::SO3CmdToMavros` hasn't been exercised against a running `mavros_node` on Jazzy.

---

## 7. Commit history

All 12 commits on `ros2_dev` since divergence from `feature/integrate_lidar_3d_planner_default`, in chronological order (the branch rename `dev/ros2` → `ros2_dev` is a ref operation, not a commit):

| # | Short hash | Title |
|---:|:---|:---|
| 1 | `1b39682` | `docs: add dev/ros2 branch experimental banner` |
| 2 | `61caff8` | `rqt_quadrotor_safety: port to ROS2 Jazzy (ament_python)` |
| 3 | `04748ce` | `map_plan: port jps3d, mpl, traj_opt_ros to ROS2 Jazzy` |
| 4 | `21c4928` | `fla_ukf + mavros_interface: nodelet -> rclcpp_components for ROS2 Jazzy` |
| 5 | `7122ad5` | `infra: port Docker, GitHub Actions, external yamls, and docs to ROS2 Jazzy` |
| 6 | `551c31f` | `launch packages: port 7 launch-only packages to ROS2 Jazzy` |
| 7 | `59fe821` | `state_machine: port action_trackers + state_machine_core to ROS2 Jazzy` |
| 8 | `266f66f` | `sim: port gazebo_utils, dcist_utils, fake_lidar, fake_sloam to ROS2 Jazzy` |
| 9 | `1a03265` | `map_plan: port action_planner, mapper, coverage_utils to ROS2 Jazzy` |
| 10 | `68c02f4` | `docs: expand README with full ROS2 Jazzy install + runtime instructions` |
| 11 | `01a1e0a` | `cleanup: sweep ROS1 leftovers missed by the per-package subagents` |
| 12 | `3c7eaea` | `state_machine + action_planner: port 9 scripts/ Python files from rospy to rclpy` |

*(The branch create commit that reset `dev/ros2` → `ros2_dev` is tracked by the ref rename and does not produce a separate commit entry.)*

---

## 8. Known follow-ups and pre-existing caveats

### 8.1 External dependency port (blocker for `colcon build`)

The largest outstanding item. None of the following upstream repositories has a ROS2 Jazzy branch in the `external_*.yaml` manifests yet, and all of them are required for a successful `colcon build`:

- `kr_trackers_manager` — parent of the 4 pluginlib tracker plugins. The `Initialize(const rclcpp::Node::SharedPtr &)` assumption in Section 2.9 depends on this port landing on exactly that signature.
- `kr_mav_control` — core control stack.
- `kr_planning_msgs` / `kr_mav_msgs` / `kr_tracker_msgs` — shared message definitions. Likely the cheapest to port (message-only packages).
- `DecompROS` (`decomp_ros_utils`, `decomp_ros_msgs`) — corridor decomposition for the local planner.
- `motion_primitive_library` / `motion_primitives` / `gcopter` / `kr_ilqr_optimizer` / `opt_planner` — the `action_planner`'s planner backend stack.
- `faster-lio` — LIDAR-inertial odometry backend for the LIDAR-only flight mode.
- `msckf_vio` — stereo+IMU VIO backend for the LIDAR+VIO mode.
- `mrsl_quadrotor` / `mrsl_quadrotor_launch` — Gazebo Classic models and spawners. Blocks the Gazebo Harmonic bring-up; see 8.4.
- `ouster_example` — Ouster LIDAR driver. KumarRobotics' fork has an `os-sim` branch, not yet a ROS2 branch.
- `arl_unity_ros` — Unity simulation bridge for `fake_sloam` / `fake_lidar`.
- `polygon_coverage_ros` / `polygon_coverage_planning` — coverage-experiment upstream.
- `kr_planning_rviz_plugins` — RViz2 plugin for replan visualization.
- `waypoint_navigation_plugin` — client RViz2 plugin for waypoint drag-and-drop.

### 8.2 CUDA compute for YOLO inference

The base Dockerfile carries a top-of-file TODO explaining that `nvidia/cudagl:11.4.2-base-ubuntu20.04` was end-of-life'd by NVIDIA and there is no drop-in Jazzy + CUDA image. `osrf/ros:jazzy-desktop-full` covers graphics (rviz2, Gazebo Harmonic) but **not** CUDA runtime. Anyone who needs YOLO inference (or any other CUDA-backed workload in the SLAM stack) will need to either stack a CUDA apt set on the Jazzy base, or switch to `nvidia/cuda:12.x-runtime-ubuntu24.04` and reinstall ROS2 Jazzy on top.

### 8.3 `smach` state machine → `smach_ros2` / `yasmin`

The `state_machine_core/scripts/*.py` files had their `rospy` → `rclpy` **surface** ported but the actual state graph is still expressed in the ROS1-only `smach` Python package. The two candidate replacements on ROS2 Jazzy are:

- [`smach_ros2`](https://github.com/uleroboticsgroup/smach) — direct smach port.
- [`yasmin`](https://github.com/uleroboticsgroup/yasmin) — native ROS2 hierarchical state machine.

The choice between the two is an open design decision and is not part of this port.

### 8.4 Gazebo Classic → Gazebo Harmonic

Gazebo Classic 11 is not available on Ubuntu 24.04. ROS2 Jazzy ships with Gazebo Harmonic via `ros_gz_*` bindings. The ROS1 wiki `Gazebo-Simulation-Setup` page is written against Gazebo Classic. Inside this repository:

- `gazebo_utils/launch/full_sim.launch.py` and `simulation.launch.py` were mechanically translated from XML but still transitively reference `mrsl_quadrotor_launch/gazebo.launch.py`, which is Gazebo-Classic-only. That upstream must be migrated before the sim bring-up works.
- Gazebo Classic `.world` files should be replaced with SDF worlds; the `world_name` launch argument in `full_sim.launch.py` accepts an SDF path unchanged.
- Spawn / bridge commands (`spawn_entity`, `robot_state_publisher` ↔ `/robot_description`) need to use the `ros_gz_sim` / `ros_gz_bridge` equivalents.

### 8.5 Tracker plugin `Initialize` signature

All four `action_trackers` plugins override `Initialize(const rclcpp::Node::SharedPtr & parent_nh)`. This is an **assumption** about how the upstream `kr_trackers_manager` ROS2 port will expose the parent container. If that port lands on `rclcpp_lifecycle::LifecycleNode::SharedPtr` or a custom context struct, all four `.cpp` files need a one-line signature tweak (and possibly a helper to extract a `rclcpp::Node::SharedPtr` from the lifecycle node for downstream use).

### 8.6 Dangling `throttle_imu.launch` include

`real_experiment_launch/launch/special_purposes/full_autonomy_ca_trip.launch.py` and `real_experiment_launch/launch/uav_ugv_localization/full_autonomy_atl.launch.py` both `IncludeLaunchDescription(...)` a file called `throttle_imu.launch` that does not exist in the source tree. This is a **pre-existing bug** that was present on the `feature/integrate_lidar_3d_planner_default` base branch and was intentionally preserved as-is to keep the port 1:1 with the upstream. It will fail the static check N (dangling launch includes) unless that check is configured to expect these two files as known failures.

### 8.7 `dynamic_reconfigure` feature regression

`action_planner/cfg/ActionPlanner.cfg` was deleted during the port because `dynamic_reconfigure` has no ROS2 drop-in. The parameters it exposed are now plain `declare_parameter(...)` calls. This is a **feature regression**: operators who previously retuned planner parameters at runtime via `rqt_reconfigure` must now edit the yaml and restart the node, or use `ros2 param set` for runtime tweaks (some parameters will not be picked up without a `add_on_set_parameters_callback` handler — adding one is a potential follow-up).

### 8.8 Legacy calibration yamls

The four calibration yamls under `real_experiment_launch/config/` (VN100, OVC3 intrinsics/extrinsics, Ouster mounts) are loaded by `dcist_utils/msckf_calib_gen` via `YAML::LoadFile`, **not** via the ROS parameter server. They were intentionally left as plain dicts — rewriting them as parameter-server YAMLs would break the calib-gen binary's expected input format.

### 8.9 Launch file `node_name` collisions

ROS2 has stricter node-name uniqueness rules than ROS1; the `launch_ros.actions.Node(...)` calls in the ported launch files inherit the ROS1 `<node name="…"/>` values and have not been audited for collisions when multiple copies of the same launch file are included (e.g. `ukf.launch.py` included inside `estimation.launch.py` included inside `full_autonomy.launch.py`). Expect to hit a handful of `Cannot create node: name already taken` errors when running the full bring-up and need to add `namespace="..."` prefixes.

---

## 9. References

- [ROS2 Migration Guide (Jazzy docs)](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html)
- [`rosidl_generate_interfaces` reference](https://docs.ros.org/en/jazzy/How-To-Guides/Single-Package-Define-And-Use-Interface.html)
- [`rclcpp_action` reference](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [`rclcpp_components` reference](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html)
- [Gazebo Harmonic ↔ ROS2 bridge](https://gazebosim.org/docs/harmonic/ros_installation)
- [`rosbags` converter](https://pypi.org/project/rosbags/) — for translating ROS1 `.bag` → ROS2 `.db3`.
