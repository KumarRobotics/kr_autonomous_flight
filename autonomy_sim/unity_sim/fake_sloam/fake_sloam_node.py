#!/usr/bin/python
#title			:object_localization.py
#description	:subscribe to image & depth & object segmentation, publish marker representing the object's estimated 3D bounding box
#author			:Xu Liu
#date			:2020-09-09
#python_version	:

import tf
import rospy
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
from tf2_ros import TransformListener, Buffer
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class FakeSloamNode:
    def __init__(self, input_fname):

        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer)

        self.tree_pub = rospy.Publisher("/quadrotor/trees", PointCloud2, queue_size=1)
        self.odom_sub = rospy.Subscriber('/unity_command/ground_truth/quadrotor/odom', Odometry, self.odom_cb)
        self.odom_in_world_frame = True

        self.perception_range = 20
        raw_data = np.loadtxt(input_fname, delimiter=',')
        self.tree_data = raw_data[1:,:] # first line is stats info
        self.robot_frame_id = 'quadrotor'
        self.ground_truth_robot_frame_id = 'ground_truth/quadrotor/quadrotor'

        self.tree_vis_pub = rospy.Publisher("/tree_observations_with_perfect_odom", MarkerArray, queue_size=100)
        self.prev_time = rospy.get_time()
        self.publish_interval = 1.0 # seconds
        self.listener = tf.TransformListener()
        self.t_world_map = []
        self.quat_world_map = []
        self.H_world_map = np.eye(4)
        self.H_world_map_rot = np.eye(3)
        self.quat_world_robot = None

    def odom_cb(self, odom_msg):
        # if transform from world to map has not been initialized yet
        if (~(self.odom_in_world_frame)) & (len(self.t_world_map)==0):
            try:
                (self.t_world_map,self.quat_world_map) = self.listener.lookupTransform('quadrotor/map', 'world', odom_msg.header.stamp)
                r_world_map = R.from_quat(self.quat_world_map)
                self.H_world_map_rot = r_world_map.as_matrix()
                # r_inv = R.from_matrix(H_world_map_rot.T)
                # self.quat_robot2world = r_inv.as_quat()
                H_world_map_trans = np.array(self.t_world_map)
                # transformation matrix from world to robot
                self.H_world_map[:3, :3] = self.H_world_map_rot
                self.H_world_map[:3, 3] = H_world_map_trans
                self.H_world_map[3, 3] = 1
                print("found tf from world to map")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("cannot find tf from world to map")
                return
        if (rospy.get_time() - self.prev_time) <= self.publish_interval:
            return
        self.prev_time = rospy.get_time()

        robot_position_odom = odom_msg.pose.pose.position
        robot_orientation_odom = odom_msg.pose.pose.orientation
        r_robot_map = R.from_quat([robot_orientation_odom.x, robot_orientation_odom.y, robot_orientation_odom.z, robot_orientation_odom.w])
        H_robot_map_rot = r_robot_map.as_matrix()
        r_world_robot = R.from_matrix(H_robot_map_rot.T @ self.H_world_map_rot)
        self.quat_world_robot = r_world_robot.as_quat()
        H_robot_map_trans = np.array([robot_position_odom.x, robot_position_odom.y, robot_position_odom.z])
        H_map_robot = np.zeros((4,4))
        # transformation matrix from map to robot
        H_map_robot[:3, :3] = H_robot_map_rot.T
        H_map_robot[:3,3] = - (H_robot_map_rot.T).dot(H_robot_map_trans)
        H_map_robot[3,3] = 1
        # transformation matrix from world to robot
        H_world_robot = H_map_robot @ self.H_world_map
        robot_position_world = - ((H_world_robot[:3, :3]).T).dot(H_world_robot[:3,3])


        diff = self.tree_data - np.array([robot_position_world[0], robot_position_world[1]])
        diff_norm = np.linalg.norm(diff, axis = 1)
        within_ran_idx = diff_norm < self.perception_range
        trees_world_xy = self.tree_data[within_ran_idx, :]
        trees_world_xyz = np.zeros((trees_world_xy.shape[0], 3))
        z_coord = 5
        z_coords = z_coord * np.ones(trees_world_xy.shape[0])
        trees_world_xyz[:,:2] = trees_world_xy
        trees_world_xyz[:,2] = z_coords
        # convert to homogeneous representation
        trees_world_xyz_homo = np.ones((trees_world_xy.shape[0], 4))
        trees_world_xyz_homo[:,:3] = trees_world_xyz
        trees_robot_frame_homo = (H_world_robot @ trees_world_xyz_homo.T).T
        trees_robot_frame = trees_robot_frame_homo[:,:3]

        points = []
        for row in trees_robot_frame:
            x = row[0]
            y = row[1]
            z = row[2]
            intensity = 0.0
            pt = [x, y, z, intensity]
            points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  ]
        header = Header()
        header.stamp = odom_msg.header.stamp
        header.frame_id = self.robot_frame_id
        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.publish_tree_markers(trees_robot_frame, header)
        self.tree_pub.publish(pc2)
        print('tree points published, total number of trees are: ', trees_robot_frame.shape[0])

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
            marker.pose.position.x = trees[idx, 0]
            marker.pose.position.y = trees[idx, 1]
            marker.pose.position.z = trees[idx, 2]

            marker.scale.x = 1 # length
            marker.scale.y = 1 # diameter
            marker.scale.z = 10 # diameter

            marker.pose.orientation.x = self.quat_world_robot[0]
            marker.pose.orientation.y = self.quat_world_robot[1]
            marker.pose.orientation.z = self.quat_world_robot[2]
            marker.pose.orientation.w = self.quat_world_robot[3]

            marker.color.a = 1  # alpha = 1 means not transparent at all
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            tree_markers.markers.append(marker)
        self.tree_vis_pub.publish(tree_markers)


if __name__ == '__main__':
    rospy.init_node('fake_sloam_node', anonymous=False)
    input_fnamex = '/home/sam/LRS-SLAM/simulator_data_process/treeposition_nn_radius_5.txt'
    my_node = FakeSloamNode(input_fnamex)
    rospy.spin()
