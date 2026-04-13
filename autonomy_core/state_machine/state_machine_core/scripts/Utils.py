from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import geometry_msgs.msg as GM
import tf_transformations
import numpy


def pose_to_transform(msg):
    m = GM.TransformStamped()
    m.header = msg.header
    m.transform.rotation = msg.pose.orientation
    m.transform.translation.x = msg.pose.position.x
    m.transform.translation.y = msg.pose.position.y
    m.transform.translation.z = msg.pose.position.z
    return m


def odom_to_pose(msg):
    m = GM.PoseStamped()
    m.header = msg.header
    m.pose = msg.pose.pose
    return m


def transform_inverse(msg):
    # Inverts a TransformStamped by computing the 4x4 matrix inverse.
    # The previous implementation used tf.Transformer which is ROS1-only.
    transform = GM.TransformStamped()
    transform.header = msg.header
    trans = (
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z,
    )
    rot = (
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    )
    mat = tf_transformations.concatenate_matrices(
        tf_transformations.translation_matrix(trans),
        tf_transformations.quaternion_matrix(rot),
    )
    inv_mat = tf_transformations.inverse_matrix(mat)
    inv_trans = tf_transformations.translation_from_matrix(inv_mat)
    inv_rot = tf_transformations.quaternion_from_matrix(inv_mat)
    transform.transform.translation.x = inv_trans[0]
    transform.transform.translation.y = inv_trans[1]
    transform.transform.translation.z = inv_trans[2]
    transform.transform.rotation.x = inv_rot[0]
    transform.transform.rotation.y = inv_rot[1]
    transform.transform.rotation.z = inv_rot[2]
    transform.transform.rotation.w = inv_rot[3]
    return transform


def pose_err(p1, p2):  # assume quaternions are only rotation about z
    # err = numpy.power(p1.position.x - p2.position.x, 2.0) +
    # numpy.power(p1.position.y - p2.position.y, 2.0) +
    # numpy.power(p1.position.z - p2.position.z, 2.0) +
    # numpy.power(numpy.abs(p1.orientation.w) -
    # numpy.abs(p2.orientation.w), 2.0)
    err = numpy.power(p1.position.x - p2.position.x, 2.0) + numpy.power(
        p1.position.y - p2.position.y, 2.0
    )
    # err = numpy.power(p1.position.x - p2.position.x, 2.0) + numpy.power(p1.position.y - p2.position.y, 2.0)  + numpy.power(numpy.abs(p1.orientation.w) - numpy.abs(p2.orientation.w), 2.0)
    return numpy.sqrt(err)
