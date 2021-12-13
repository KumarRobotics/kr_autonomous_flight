from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import smach
import smach_ros
import std_srvs.srv as srvs
import std_msgs.msg as msgs
import kr_tracker_msgs.srv as TM
import action_trackers.msg as AT
import mavros_msgs.msg as MR
import nav_msgs.msg as NM
import mavros_msgs.srv as MR
import mavros_msgs.msg as MRM
import geometry_msgs.msg as GM
import planning_ros_msgs.msg as PM
import copy
import numpy as np


# Waits for offbaord signal. then takes off
class WaitForOffboard(smach.State):
    def state_cb(self, msg):
        self.mode = msg.mode

    def __init__(self, quad_monitor, topic):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.mode = None
        self.mr_sub = rospy.Subscriber(topic, MRM.State, self.state_cb, queue_size=5)
        self.quad_monitor = quad_monitor
        self.timeout = 100

    def execute(self, userdata):
        self.timeout = 100
        r = rospy.Rate(10)
        while True:
            rospy.logwarn_throttle(1, "Waiting for offboard. Current mode is " + str(self.mode))
            if self.mode is None:
                self.timeout -= 1
            if self.timeout <= 0:
                rospy.logerr("Wait time reached, is px4 running and mapped correctly?")
                return "failed"
            if self.mode is not None and self.mode == "OFFBOARD":
                return "done"
            r.sleep()


class GetWaypoints(smach.State):
    """Gets waypoints and prepends quadrotors pose"""

    def __init__(self, quad_monitor):
        smach.State.__init__(self, outcomes=["succeeded", "multi", "failed"])
        self.quad_monitor = quad_monitor
        self.reset_pub = rospy.Publisher("reset", GM.PoseStamped, queue_size=10)

    def execute(self, userdata):
        if self.quad_monitor.waypoints is None or len(self.quad_monitor.waypoints.poses) == 0:
            rospy.logerr("Failed to get waypoints")
            return "failed"

        # record all waypoints
        self.quad_monitor.pose_goals = [it.pose for it in self.quad_monitor.waypoints.poses] # will be feed into goal.p_finals later in RePlan class       
        # record the last waypoint to check whether the robot has finished all waypoints when replanning
        self.quad_monitor.pose_goal = self.quad_monitor.waypoints.poses[-1].pose #  will be feed into goal.p_final later in RePlan class
        # this informs the replanner to starting a new mission
        self.quad_monitor.continue_mission = False # will be feed into goal.continue_mission later in RePlan class

        ps = GM.PoseStamped()
        ps.header = self.quad_monitor.waypoints.header
        ps.pose = self.quad_monitor.pose_goal
        self.reset_pub.publish(ps)

        if len(self.quad_monitor.waypoints.poses) > 1:
            rospy.loginfo("Multiple waypoints: {}".format(len(self.quad_monitor.waypoints.poses)))
            return "multi"

        return "succeeded"



class RetryWaypoints(smach.State):
    """Retry waypoints and prepends quadrotors pose"""

    def __init__(self, quad_monitor, max_planning_velocity, max_stopping_acceleration):
        smach.State.__init__(self, outcomes=["succeeded", "multi", "failed"])
        self.quad_monitor = quad_monitor
        self.reset_pub = rospy.Publisher("reset", GM.PoseStamped, queue_size=10)
        self.num_trials = 1
        self.max_trials = quad_monitor.max_replan_trials
        # time to wait for stopping policy to finish, should be large enough so that the robot fully stops
        est_stopping_time = max_planning_velocity / (max_stopping_acceleration * 0.3) # reduce the acceleration to consider worst-case scenarios
        self.wait_for_stopping = est_stopping_time
    def execute(self, userdata):
        # print self.quad_monitor.waypoints
        print("[state_machine:] waiting for stopping policy to finish, wait time is: ", self.wait_for_stopping, " seconds.")
        rospy.sleep(self.wait_for_stopping)
        print("\n")
        print("[state_machine:] retrying waypoints! Have tried ", self.num_trials, " times up till now. max_trials is set as ", self.max_trials)
        print("\n")
        
        if self.num_trials >= self.max_trials:
            print("Current number of trials >= max_trials, which is ", self.max_trials, " aborting the mission!")
            return "failed"
        else:
            self.num_trials += 1
       
        # this informs the replanner to continue to finish waypoints in existing mission instead of starting a new mission
        self.quad_monitor.continue_mission = True # will be feed into goal.continue_mission later in RePlan class
        ps = GM.PoseStamped()
        ps.header = self.quad_monitor.waypoints.header
        ps.pose = self.quad_monitor.pose_goal
        self.reset_pub.publish(ps)

        return "succeeded"

class WaitState(smach.State):
    """Publishes an LED status"""

    def __init__(self, time):
        smach.State.__init__(self, outcomes=["done"])
        self.time = time

    def execute(self, userdata):
        rospy.sleep(self.time)
        if self.preempt_requested():
            self.service_preempt()
        return "done"


class ArmDisarmMavros(smach.State):
    def __init__(self, service, value):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.service = service
        self.value = value
        # setup internal msgs

    def execute(self, userdata):
        # rospy.wait_for_service(self.service)

        try:
            arm = rospy.ServiceProxy(self.service, MR.CommandBool)
            arm(self.value)
        except Exception as e:
            rospy.logerr("Error thrown while trying to arm motors %s: %s" % (str(self.value), e))
            return "succeeded"
        return "succeeded"


class PublishBoolMsgState(smach.State):
    """PublishBoolMsgState published a std_msgs Bool on topic with data value"""

    def __init__(self, topic, value):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.pub = rospy.Publisher(topic, msgs.Bool, queue_size=1)
        # setup internal msgs
        msg = msgs.Bool()
        msg.data = value
        self.msg = msg

    def execute(self, userdata):
        try:
            self.pub.publish(self.msg)
        except Exception as e:
            rospy.logerr(
                "Error thrown while executing condition callback %s: %s" % (str(self._cond_cb), e)
            )
            return "failed"
        return "succeeded"


class TrackerTransition(smach_ros.ServiceState):
    def execute(self, userdata):

        rospy.wait_for_service(self.service_name)
        try:
            self.prox(self.req)
            return "succeeded"
        except Exception as e:
            rospy.logerr(
                "Error thrown while executing condition callback %s: %s" % (str(self.execute), e)
            )
            return "aborted"

    """Wraps service state with a tracker transition"""

    def __init__(self, service, tracker, quad_monitor, safety_critical=False):
        self.quad_monitor = quad_monitor
        self.safety_critical = safety_critical
        smach.State.__init__(self, outcomes=["succeeded", "aborted", "preempted"])
        self.req = TM.TransitionRequest()
        self.req.tracker = tracker
        self.service_name = rospy.names.get_namespace() + service
        self.prox = rospy.ServiceProxy(self.service_name, TM.Transition)


# Define States Which Need Extra Data (ex. home, goal, etc)
class TakingOff(smach_ros.SimpleActionState):
    """docstring for TakeOff"""

    def goal_cb(self, userdata, goal):
        # need to generate goal at run time
        goal.height = self.quad_tracker.takeoff_height
        return goal

    def __init__(self, action_topic, quad_tracker):
        self.quad_tracker = quad_tracker
        smach_ros.SimpleActionState.__init__(
            self, rospy.names.get_namespace() + action_topic, AT.TakeOffAction, goal_cb=self.goal_cb
        )


class SetHomeHere(smach.State):
    def __init__(self, quad_tracker):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.quad_tracker = quad_tracker

    def execute(self, userdata):
        try:
            self.quad_tracker.set_home_from_air()
        except Exception as e:
            rospy.logerr("Error thrown while executing set home here %s" % e)
            return "failed"
        return "succeeded"


class Landing(smach_ros.SimpleActionState):
    """docstring for Landing"""

    def goal_cb(self, userdata, goal):
        # need to generate goal at run time
        # goal.goal = self.quad_tracker.get_ground_msg()
        return goal

    def __init__(self, action_topic, quad_tracker):
        self.quad_tracker = quad_tracker
        smach_ros.SimpleActionState.__init__(
            self, rospy.names.get_namespace() + action_topic, AT.LandAction, goal_cb=self.goal_cb
        )
