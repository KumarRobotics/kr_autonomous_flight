# Copyright (c) 2016, Mike Watterson and Dinesh Thankur

import os
from math import log10, floor

from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi

try:
    # PyQt4
    from python_qt_binding.QtGui import QWidget
except ImportError:
    # PyQt5
    from python_qt_binding.QtWidgets import QWidget

from qt_gui.plugin import Plugin

import std_msgs.msg
import kr_planning_msgs.msg as MHL
from nav_msgs.msg import Odometry
import kr_mav_msgs.msg as QM
import numpy as np

try:
    from tf_transformations import euler_from_quaternion
except ImportError:  # pragma: no cover - fallback for systems without tf_transformations
    from transforms3d.euler import quat2euler

    def euler_from_quaternion(q):
        # transforms3d uses (w, x, y, z) order; tf uses (x, y, z, w)
        return quat2euler((q[3], q[0], q[1], q[2]))


def round_str(x, sig=3):
    return str(round(x, sig - int(floor(log10(x))) - 1))


class QuadrotorSafety(Plugin):

    # slider_factor = 1000.0

    def __init__(self, context):
        super(QuadrotorSafety, self).__init__(context)
        self.setObjectName("QuadrotorSafety")

        # rqt_gui_py provides an rclpy Node via context.node in ROS2
        self._node = context.node

        self.odom_count = 0
        self.command_count = 0

        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_quadrotor_safety"),
            "resource",
            "QuadrotorSafety.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("QuadrotorSafetyUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} ({context.serial_number()})"
            )
        context.add_widget(self._widget)

        # ROS2 topics
        self.odom_sub = self._node.create_subscription(
            Odometry, "odom", self.odom_cb, 10
        )
        self.command_sub = self._node.create_subscription(
            QM.PositionCommand, "position_cmd", self.command_cb, 10
        )
        self.state_trigger = self._node.create_publisher(
            MHL.StateTransition, "state_trigger", 1
        )
        self.replan_state_trigger = self._node.create_publisher(
            std_msgs.msg.String, "replan_state_trigger", 1
        )
        self.motors_pub = self._node.create_publisher(
            std_msgs.msg.Bool, "motors", 1
        )

        # normal operations
        self._widget.motors_on_push_button.pressed.connect(self._on_motors_on_pressed_empty)
        self._widget.motors_off_push_button.pressed.connect(self._on_motors_off_pressed_empty)
        self._widget.skip_next_waypoint_push_button.pressed.connect(self._on_skip_next_waypoint_pressed_empty)
        self._widget.reset_mission_push_button.pressed.connect(self._on_reset_mission_pressed_empty)
        self._widget.takeoff_push_button.pressed.connect(self._on_takeoff_pressed_empty)
        self._widget.landhere_push_button.pressed.connect(self._on_landhere_pressed_empty)
        # self._widget.short_push_button.pressed.connect(self.short_range_pressed)
        self._widget.execute_waypoints_push_button.pressed.connect(self.execute_waypoints_pressed)

    def odom_cb(self, msg):
        if self.odom_count % 10 == 0:
            x = np.round(msg.pose.pose.position.x, 4)
            y = np.round(msg.pose.pose.position.y, 4)
            z = np.round(msg.pose.pose.position.z, 4)

            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            euler = euler_from_quaternion(quaternion)
            yaw = np.round(euler[2], 4)
            self._widget.odom_x_data.setText(str(x))
            self._widget.odom_y_data.setText(str(y))
            self._widget.odom_z_data.setText(str(z))
            self._widget.odom_yaw_data.setText(str(yaw))
            self.odom_count = 0
        self.odom_count += 1

    def command_cb(self, msg):
        if self.command_count % 10 == 0:
            pos_x = np.round(msg.position.x, 4)
            pos_y = np.round(msg.position.y, 4)
            pos_z = np.round(msg.position.z, 4)
            vel_x = np.round(msg.velocity.x, 4)
            vel_y = np.round(msg.velocity.y, 4)
            vel_z = np.round(msg.velocity.z, 4)
            acc_x = np.round(msg.acceleration.x, 4)
            acc_y = np.round(msg.acceleration.y, 4)
            acc_z = np.round(msg.acceleration.z, 4)
            yaw = np.round(msg.yaw, 4)
            yaw_dot = np.round(msg.yaw_dot, 4)

            self._widget.command_pos_x_data.setText(str(pos_x))
            self._widget.command_pos_y_data.setText(str(pos_y))
            self._widget.command_pos_z_data.setText(str(pos_z))
            self._widget.command_vel_x_data.setText(str(vel_x))
            self._widget.command_vel_y_data.setText(str(vel_y))
            self._widget.command_vel_z_data.setText(str(vel_z))
            self._widget.command_acc_x_data.setText(str(acc_x))
            self._widget.command_acc_y_data.setText(str(acc_y))
            self._widget.command_acc_z_data.setText(str(acc_z))
            # self._widget.command_jerk_x_data.setText(str(jerk_x))
            # self._widget.command_jerk_y_data.setText(str(jerk_y))
            # self._widget.command_jerk_z_data.setText(str(jerk_z))
            self._widget.command_yaw_data.setText(str(yaw))
            self._widget.command_yaw_dot_data.setText(str(yaw_dot))

            self.command_count = 0
        self.command_count += 1

    def publish_string(self, string):
        # this will be subscribed by state machine
        msg = MHL.StateTransition()
        msg.transition.data = string
        self.state_trigger.publish(msg)

    def publish_replan_string(self, string):
        # this will be subscribed by replanner
        msg = std_msgs.msg.String()
        msg.data = string
        self.replan_state_trigger.publish(msg)

    def _on_motors_on_pressed_empty(self):
        self.publish_string("motors_on")

    def _on_motors_off_pressed_empty(self):
        self.publish_string("motors_off")

    def _on_reset_mission_pressed_empty(self):
        self.publish_replan_string("reset_mission")

    def _on_skip_next_waypoint_pressed_empty(self):
        self.publish_replan_string("skip_next_waypoint")

    def _on_takeoff_pressed_empty(self):
        self.publish_string("takeoff")

    def _on_landhere_pressed_empty(self):
        self.publish_string("land_here")

    #    short_range_pressed(self):
    #     self.publish_string("short_range")

    def execute_waypoints_pressed(self):
        self.publish_string("waypoints")

    def shutdown_plugin(self):
        # Destroy publishers/subscriptions so rclpy releases them cleanly.
        if hasattr(self, "_node") and self._node is not None:
            for handle in (
                getattr(self, "odom_sub", None),
                getattr(self, "command_sub", None),
            ):
                if handle is not None:
                    try:
                        self._node.destroy_subscription(handle)
                    except Exception:
                        pass
            for handle in (
                getattr(self, "state_trigger", None),
                getattr(self, "replan_state_trigger", None),
                getattr(self, "motors_pub", None),
            ):
                if handle is not None:
                    try:
                        self._node.destroy_publisher(handle)
                    except Exception:
                        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
