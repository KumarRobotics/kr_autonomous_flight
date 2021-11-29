from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy

import kr_mav_msgs.msg as QuadM
import nav_msgs.msg as NavM
import geometry_msgs.msg as GM
import std_msgs.msg as SM
import sensor_msgs.msg as SMS
import threading
import tf
import copy
import numpy as np
from Utils import *


class QuadTracker:
    def lidar_cb(self, msg):
        self.lock.acquire()
        self.rel_z = msg.range
        self.lock.release()

    def way_cb(self, msg):
        rospy.loginfo("Got waypoints")
        self.waypoints = msg

    def cmd_call_back(self, msg):
        self.lock.acquire()
        self.posC = GM.Pose()
        self.posC.position.x = msg.position.x
        self.posC.position.y = msg.position.y
        self.posC.position.z = msg.position.z
        self.posC.orientation.z = np.sin(msg.yaw / 2.0)
        self.posC.orientation.w = np.cos(msg.yaw / 2.0)
        self.v_des = np.sqrt(
            msg.velocity.x * msg.velocity.x
            + msg.velocity.y * msg.velocity.y
            + msg.velocity.z * msg.velocity.z
        )
        self.lock.release()

    def abort_call_back(self, msg):
        self.abort = True

    def odom_call_back(self, msg):
        self.lock.acquire()
        self.odom = msg
        self.pos = msg.pose.pose.position
        self.pose = msg.pose.pose
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.lock.release()

    def set_home_from_air(self):
        self.lock.acquire()
        self.homeP = copy.copy(self.pos)
        self.homeY = self.yaw
        self.ground = copy.copy(self.pos)
        self.ground.z += self.landing_height - self.takeoff_height
        rospy.logwarn("Setting landing height to %f m", self.ground.z)
        self.lock.release()

    def set_home_from_ground(self):
        self.lock.acquire()
        self.homeP = copy.copy(self.pos)
        self.ground = copy.copy(self.pos)
        self.ground.z += self.landing_height
        self.homeP.z += self.takeoff_height
        rospy.logwarn("Setting takeoff_height height to %f m", self.homeP.z)
        # self.homeP.y += 2.0
        self.homeY = self.yaw
        self.lock.release()

    def get_curr_pose(self):
        self.lock.acquire()
        # Make sure home is set
        # if self.homeP is None:
        #   self.lock.release()
        #   return None
        msg = copy.copy(self.pose)
        self.lock.release()

        return msg

    def get_curr_poseC(self):
        self.lock.acquire()
        msg = copy.deepcopy(self.posC)
        self.lock.release()

        return msg

    def get_curr_transform(self):
        return pose_to_transform(odom_to_pose(self.odom))

    def add_to_takeoff(self, dh):
        self.homeP.z += dh

    def __init__(self, abort_topic):
        self.lock = threading.Lock()
        self.homeP = None
        self.abort = False
        self.pos = None
        self.posC = None
        self.yaw = None
        self.ground = None
        # self.polaris_goal = None
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.5)
        # self.xml_file = rospy.get_param("~xml_file", "")
        self.landing_height = rospy.get_param("~landing_height", -5.05)
        self.sub = rospy.Subscriber("~odom", NavM.Odometry, self.odom_call_back, queue_size=1)
        self.subC = rospy.Subscriber(
            "position_cmd", QuadM.PositionCommand, self.cmd_call_back, queue_size=1
        )
        self.subA = rospy.Subscriber(abort_topic, SM.Empty, self.abort_call_back, queue_size=1)
        self.waypoints = None
        self.way_sub = rospy.Subscriber("waypoints", NM.Path, self.way_cb, queue_size=1)
        self.lidar_sub = rospy.Subscriber("~height", SMS.Range, self.lidar_cb, queue_size=10)
        self.avoid = True
        self.waypoints = None
        # self.trajectory = None
        self.pose_goal = None
        # replan rate, calling local planner every (1/replan_rate) seconds
        self.replan_rate = None
        self.replan_status = None
        self.reset_server = True
        self.v_des = 100
        self.et = False
        self.pose_goals = None
        self.listener = tf.TransformListener()
        self.rel_z = 1.0
        # max allowed trials to re-enter replanner
        self.max_replan_trials = None 
        # if False, start a new mission; otherwise, continue to finish waypoints in existing mission
        self.continue_mission = False
