#!/usr/bin/env python3
# title           :fake_sloam_node.py
# description     :Publishes a fake SLOAM tree point cloud / markers based on
#                  a ground truth tree position file and the current robot
#                  odometry.
# author          :Xu Liu
# date            :2020-09-09
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FakeSloamNode(Node):
    def __init__(self):
        super().__init__("fake_sloam_node")

        self.declare_parameter(
            "input_file",
            "/home/sam/LRS-SLAM/simulator_data_process/treeposition_nn_radius_5.txt",
        )
        self.declare_parameter("perception_range", 20.0)
        self.declare_parameter("publish_interval", 1.0)
        self.declare_parameter("robot_frame_id", "quadrotor")
        self.declare_parameter(
            "ground_truth_robot_frame_id", "ground_truth/quadrotor/quadrotor"
        )
        self.declare_parameter("odom_in_world_frame", True)

        input_fname = self.get_parameter("input_file").value
        self.perception_range = float(self.get_parameter("perception_range").value)
        self.publish_interval = float(self.get_parameter("publish_interval").value)
        self.robot_frame_id = self.get_parameter("robot_frame_id").value
        self.ground_truth_robot_frame_id = self.get_parameter(
            "ground_truth_robot_frame_id"
        ).value
        self.odom_in_world_frame = bool(
            self.get_parameter("odom_in_world_frame").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tree_pub = self.create_publisher(
            PointCloud2, "/quadrotor/trees", 1
        )
        self.tree_vis_pub = self.create_publisher(
            MarkerArray, "/tree_observations_with_perfect_odom", 100
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/unity_command/ground_truth/quadrotor/odom",
            self.odom_cb,
            10,
        )

        raw_data = np.loadtxt(input_fname, delimiter=",")
        # first line is stats info
        self.tree_data = raw_data[1:, :]

        self.prev_time = self.get_clock().now()
        self.t_world_map = []
        self.quat_world_map = []
        self.H_world_map = np.eye(4)
        self.H_world_map_rot = np.eye(3)
        self.quat_world_robot = None

    def odom_cb(self, odom_msg):
        # If transform from world to map has not been initialized yet
        if (not self.odom_in_world_frame) and (len(self.t_world_map) == 0):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    "quadrotor/map", "world", rclpy.time.Time()
                )
                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                self.t_world_map = [t.x, t.y, t.z]
                self.quat_world_map = [q.x, q.y, q.z, q.w]
                r_world_map = R.from_quat(self.quat_world_map)
                self.H_world_map_rot = r_world_map.as_matrix()
                H_world_map_trans = np.array(self.t_world_map)
                # transformation matrix from world to robot
                self.H_world_map[:3, :3] = self.H_world_map_rot
                self.H_world_map[:3, 3] = H_world_map_trans
                self.H_world_map[3, 3] = 1
                self.get_logger().info("found tf from world to map")
            except TransformException:
                self.get_logger().warn("cannot find tf from world to map")
                return

        now = self.get_clock().now()
        if (now - self.prev_time).nanoseconds / 1e9 <= self.publish_interval:
            return
        self.prev_time = now

        robot_position_odom = odom_msg.pose.pose.position
        robot_orientation_odom = odom_msg.pose.pose.orientation
        r_robot_map = R.from_quat(
            [
                robot_orientation_odom.x,
                robot_orientation_odom.y,
                robot_orientation_odom.z,
                robot_orientation_odom.w,
            ]
        )
        H_robot_map_rot = r_robot_map.as_matrix()
        r_world_robot = R.from_matrix(H_robot_map_rot.T @ self.H_world_map_rot)
        self.quat_world_robot = r_world_robot.as_quat()
        H_robot_map_trans = np.array(
            [
                robot_position_odom.x,
                robot_position_odom.y,
                robot_position_odom.z,
            ]
        )
        H_map_robot = np.zeros((4, 4))
        # transformation matrix from map to robot
        H_map_robot[:3, :3] = H_robot_map_rot.T
        H_map_robot[:3, 3] = -(H_robot_map_rot.T).dot(H_robot_map_trans)
        H_map_robot[3, 3] = 1
        # transformation matrix from world to robot
        H_world_robot = H_map_robot @ self.H_world_map
        robot_position_world = -((H_world_robot[:3, :3]).T).dot(
            H_world_robot[:3, 3]
        )

        diff = self.tree_data - np.array(
            [robot_position_world[0], robot_position_world[1]]
        )
        diff_norm = np.linalg.norm(diff, axis=1)
        within_ran_idx = diff_norm < self.perception_range
        trees_world_xy = self.tree_data[within_ran_idx, :]
        trees_world_xyz = np.zeros((trees_world_xy.shape[0], 3))
        z_coord = 5
        z_coords = z_coord * np.ones(trees_world_xy.shape[0])
        trees_world_xyz[:, :2] = trees_world_xy
        trees_world_xyz[:, 2] = z_coords
        # convert to homogeneous representation
        trees_world_xyz_homo = np.ones((trees_world_xy.shape[0], 4))
        trees_world_xyz_homo[:, :3] = trees_world_xyz
        trees_robot_frame_homo = (H_world_robot @ trees_world_xyz_homo.T).T
        trees_robot_frame = trees_robot_frame_homo[:, :3]

        points = []
        for row in trees_robot_frame:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            intensity = 0.0
            points.append([x, y, z, intensity])

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        header = Header()
        header.stamp = odom_msg.header.stamp
        header.frame_id = self.robot_frame_id
        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.publish_tree_markers(trees_robot_frame, header)
        self.tree_pub.publish(pc2)
        self.get_logger().info(
            "tree points published, total number of trees are: "
            + str(trees_robot_frame.shape[0])
        )

    def publish_tree_markers(self, trees, header):
        tree_markers = MarkerArray()
        tree_markers.markers = []
        for idx in np.arange(trees.shape[0]):
            marker = Marker()
            marker.header.frame_id = self.ground_truth_robot_frame_id
            marker.ns = str(idx)
            marker.header.stamp = header.stamp
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(trees[idx, 0])
            marker.pose.position.y = float(trees[idx, 1])
            marker.pose.position.z = float(trees[idx, 2])

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 10.0

            if self.quat_world_robot is not None:
                marker.pose.orientation.x = float(self.quat_world_robot[0])
                marker.pose.orientation.y = float(self.quat_world_robot[1])
                marker.pose.orientation.z = float(self.quat_world_robot[2])
                marker.pose.orientation.w = float(self.quat_world_robot[3])
            else:
                marker.pose.orientation.w = 1.0

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            tree_markers.markers.append(marker)
        self.tree_vis_pub.publish(tree_markers)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSloamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
