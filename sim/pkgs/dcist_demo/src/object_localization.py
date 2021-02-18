#!/usr/bin/python
#title			:object_localization.py
#description	:subscribe to image & depth & object segmentation, publish marker representing the object's estimated 3D bounding box
#author			:Xu Liu
#date			:2020-09-09
#python_version	:

import rospy
import message_filters
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from arl_unity_ros.msg import ImageDetections

from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import fields_to_dtype, get_xyz_points, DUMMY_FIELD_PREFIX
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import TransformListener, Buffer

class ObjectLocalizationNode:
    def __init__(self):

        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer)

        self.last_depth_image = np.empty(0)
        self.last_semantic_seg = np.empty(0)
        self.last_depth_image_msg = None
        self.bridge = CvBridge()
        self.car_pix_values = np.array([255,0,0])
        self.car_pos = np.empty(0)
        # value from 0 to 1, the larger the more weight on current position
        self.pos_decay_alpha = 0.8 

        self.car_depth_image_pub = rospy.Publisher("/quadrotor/DepthCamera/car_depth_raw", Image, queue_size=1)
        self.car_box_pub = rospy.Publisher("/car_detection_marker", MarkerArray, queue_size=1)

        self.car_pc_sub = rospy.Subscriber("/quadrotor/rgbd/car_cloud", PointCloud2, self.car_pc_cb)

        self.depth_image_sub = message_filters.Subscriber("/quadrotor/DepthCamera/image_raw", Image)
        self.semantic_seg_sub = message_filters.Subscriber("/quadrotor/DepthCamera/semantic_image", Image)
        ts = message_filters.TimeSynchronizer([self.depth_image_sub, self.semantic_seg_sub], 10)
        ts.registerCallback(self.semantic_depth_cb)



    def semantic_depth_cb(self, depth_image_msg, semantic_seg_msg):
        # check if it is the first detection
        if self.car_pos.shape[0] == 0:
            print("Depth image and semantic image received")
        self.last_depth_image_msg = depth_image_msg
        depth_image_msg.encoding = "mono16"
        self.last_depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "mono16")
        self.last_semantic_seg = self.bridge.imgmsg_to_cv2(semantic_seg_msg, "rgb8")

        # remember, depth image unit is mm, value 0 represent invalid depth
        # refer here: https://www.ros.org/reps/rep-0118.html#:~:text=Depth%20images%20are%20published%20as,the%20standard%20camera%20driver%20API.
        car_only_depth_image = self.last_depth_image

        # find out index of pixels belonging to car class
        idx_r = self.last_semantic_seg[:,:,0] == self.car_pix_values[0]
        idx_g = self.last_semantic_seg[:,:,1] == self.car_pix_values[1]
        idx_b = self.last_semantic_seg[:,:,2] == self.car_pix_values[2]
        idx_all = idx_r & idx_g & idx_b

        # set non-car depth to be invalid
        car_only_depth_image[idx_all==False] = 0
        car_only_depth_image_msg = self.bridge.cv2_to_imgmsg(car_only_depth_image, encoding="16UC1")
        car_only_depth_image_msg.header = self.last_depth_image_msg.header
        self.car_depth_image_pub.publish(car_only_depth_image_msg)

    def car_pc_cb(self, car_cloud_msg):
        # This trasform step is slow, thus not used any more. Directly publish in "quadrotor/DepthCamera" frame (see marker.header.frame_id below)
        # last_transform = self.tf_buffer.lookup_transform("world", "quadrotor/DepthCamera", rospy.Time())
        # car_cloud_world = do_transform_cloud(car_cloud_msg, last_transform)
        # cloud_arr = cloud2array(car_cloud_world, squeeze=True)

        # parse the cloud into an array
        cloud_arr = cloud2array(car_cloud_msg, squeeze=True)
        # N*3 matrix recording X Y Z coordinates
        cloud_mat = get_xyz_points(cloud_arr)

        # TODO: consider the case of having more than one car in the scene
        # don't update when to few points
        if cloud_mat.shape[0] >=100:
            # get the max and min of each axis
            lower_percentile = 10
            upper_percentile = 90
            xs = cloud_mat[:,0]
            ys = cloud_mat[:,1]
            zs = cloud_mat[:,2]
            x_med = np.median(xs)
            y_med = np.median(ys)
            z_med = np.median(zs)
            print("Current car detection x_med, y_med, z_med are: ", x_med, y_med, z_med)
            current_pos = np.array([x_med, y_med, z_med])
            # compute exponential moving average of car position to make it smooth
            # check if it is the first detection
            if self.car_pos.shape[0] == 0:
                self.car_pos = current_pos
            else:
                self.car_pos = current_pos
                # decaying in local frame is not a good idea.
                # self.car_pos =  (1-self.pos_decay_alpha) * self.car_pos + self.pos_decay_alpha * current_pos

            self.publish_car_markers()


    def publish_car_markers(self):
        car_markers = MarkerArray()
        # TODO: consider the case of having more than one car in the scene
        # Publish car as a marker in rviz
        car_markers.markers = []
        marker = Marker()
        marker.header.frame_id = "quadrotor/DepthCamera"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "car"
        marker.id = 0   # TODO: consider the case of having more than one car in the scene, marker.id should be different for each car
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.car_pos[0]
        marker.pose.position.y = self.car_pos[1]
        marker.pose.position.z = self.car_pos[2]

        marker.scale.x = 4 # length
        marker.scale.y = 0.6 # diameter
        marker.scale.z = 0.6 # diameter

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = -1.0
        marker.pose.orientation.w = 1.0

        marker.color.a = 1  # alpha = 1 means not transparent at all
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0
        car_markers.markers.append(marker)
        self.car_box_pub.publish(car_markers)

def cloud2array(cloud_msg, squeeze=True):
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    # parse the cloud into an array
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

if __name__ == '__main__':
    rospy.init_node('object_localization_node', anonymous=False)
    my_node = ObjectLocalizationNode()
    rospy.spin()