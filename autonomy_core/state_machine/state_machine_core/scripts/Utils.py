from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import geometry_msgs.msg as GM
import tf
import rospy
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
    transform = GM.TransformStamped()
    transform.header = msg.header
    t = tf.Transformer(True, rospy.Duration.from_sec(0.1))
    msg.header.frame_id = "a"
    msg.child_frame_id = "b"
    t.setTransform(msg)
    (trans, rot) = t.lookupTransform("b", "a", rospy.Time(0))
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]
    transform.transform.rotation.x = rot[0]
    transform.transform.rotation.y = rot[1]
    transform.transform.rotation.z = rot[2]
    transform.transform.rotation.w = rot[3]
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
