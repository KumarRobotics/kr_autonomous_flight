# TODO: smach is ROS1-only. This file will not work under ROS2 until the smach
# ecosystem is ported (smach_ros2 or yasmin). The rospy -> rclpy translation
# below is best-effort; the smach state machine itself (including
# smach_ros.SimpleActionState and smach_ros.ServiceState) needs a separate
# migration.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time

import smach
import smach_ros
import std_msgs.msg as msgs
import kr_tracker_msgs.srv as TM
import mavros_msgs.srv as MR
import mavros_msgs.msg as MRM
import geometry_msgs.msg as GM
import action_trackers.msg as AT


def _node(quad_monitor):
    """Return the rclpy Node attached to the quad_monitor (set by main)."""
    return getattr(quad_monitor, "node", None)


# Waits for offbaord signal. then takes off
class WaitForOffboard(smach.State):
    def state_cb(self, msg):
        self.mode = msg.mode

    def __init__(self, quad_monitor, topic):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.mode = None
        self.quad_monitor = quad_monitor
        self.topic = topic
        self._sub = None
        node = _node(quad_monitor)
        if node is not None:
            self._sub = node.create_subscription(MRM.State, topic, self.state_cb, 5)
        self.timeout = 100

    def execute(self, _userdata):
        self.timeout = 100
        node = _node(self.quad_monitor)
        while True:
            if node is not None:
                node.get_logger().warn("Waiting for offboard. Current mode is " + str(self.mode),
                                       throttle_duration_sec=1.0)
            if self.mode is None:
                self.timeout -= 1
            if self.timeout <= 0:
                if node is not None:
                    node.get_logger().error("Wait time reached, is px4 running and mapped correctly?")
                return "failed"
            if self.mode is not None and self.mode == "OFFBOARD":
                return "done"
            time.sleep(0.1)


class GetWaypoints(smach.State):
    """Gets waypoints and prepends quadrotors pose"""

    def __init__(self, quad_monitor):
        smach.State.__init__(self, outcomes=["succeeded", "multi", "failed"])
        self.quad_monitor = quad_monitor
        node = _node(quad_monitor)
        if node is not None:
            self.reset_pub = node.create_publisher(GM.PoseStamped, "reset", 10)
        else:
            self.reset_pub = None

    def execute(self, _userdata):
        node = _node(self.quad_monitor)
        if self.quad_monitor.waypoints is None or len(self.quad_monitor.waypoints.poses) == 0:
            if node is not None:
                node.get_logger().error("Failed to get waypoints")
            return "failed"

        # record all waypoints
        # will be feed into goal.p_finals later in RePlan class
        self.quad_monitor.pose_goals = [it.pose for it in
                self.quad_monitor.waypoints.poses]
        # record the last waypoint to check whether the robot has finished all waypoints
        # when replanning
        #  will be feed into goal.p_final later in RePlan class
        self.quad_monitor.pose_goal = self.quad_monitor.waypoints.poses[-1].pose
        # this informs the replanner to starting a new mission
        self.quad_monitor.continue_mission = False # will be feed into goal.continue_mission later in RePlan class

        ps = GM.PoseStamped()
        ps.header = self.quad_monitor.waypoints.header
        ps.pose = self.quad_monitor.pose_goal
        if self.reset_pub is not None:
            self.reset_pub.publish(ps)

        if len(self.quad_monitor.waypoints.poses) > 1:
            if node is not None:
                node.get_logger().info(f"Multiple waypoints: {len(self.quad_monitor.waypoints.poses)}")
            return "multi"

        return "succeeded"



class RetryWaypoints(smach.State):
    """Retry waypoints and prepends quadrotors pose"""

    def __init__(self, quad_monitor):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.quad_monitor = quad_monitor
        node = _node(quad_monitor)
        if node is not None:
            self.reset_pub = node.create_publisher(GM.PoseStamped, "reset", 10)
        else:
            self.reset_pub = None
        self.num_trials = 1
        self.max_trials = quad_monitor.max_replan_trials

    def execute(self, _userdata):
        # print self.quad_monitor.waypoints
        print("[state_machine:] retrying waypoints! Have tried ", self.num_trials, " times up till now. max_trials is set as ", self.max_trials, "\n")

        if self.num_trials >= self.max_trials:
            print("[state_machine:] current number of trials >= max_trials, which is ", self.max_trials, " aborting the mission!")
            return "failed"
        self.num_trials += 1

        # this informs the replanner to continue to finish waypoints in existing mission instead of starting a new mission
        self.quad_monitor.continue_mission = True # will be feed into goal.continue_mission later in RePlan class
        ps = GM.PoseStamped()
        ps.header = self.quad_monitor.waypoints.header
        ps.pose = self.quad_monitor.pose_goal
        if self.reset_pub is not None:
            self.reset_pub.publish(ps)

        return "succeeded"

class WaitState(smach.State):
    """Publishes an LED status"""

    def __init__(self, time_sec):
        smach.State.__init__(self, outcomes=["done"])
        self.time = time_sec

    def execute(self, _userdata):
        time.sleep(self.time)
        if self.preempt_requested():
            self.service_preempt()
        return "done"


class ArmDisarmMavros(smach.State):
    def __init__(self, service, value):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.service = service
        self.value = value
        self.quad_monitor = None  # set externally if needed
        # setup internal msgs

    def execute(self, _userdata):
        node = _node(self.quad_monitor) if self.quad_monitor is not None else None
        if node is None:
            return "succeeded"
        try:
            client = node.create_client(MR.CommandBool, self.service)
            req = MR.CommandBool.Request()
            req.value = self.value
            client.call_async(req)
        except Exception as e:
            node.get_logger().error(f"Error thrown while trying to arm motors {str(self.value)}: {e}")
            return "succeeded"
        return "succeeded"


class PublishBoolMsgState(smach.State):
    """PublishBoolMsgState published a std_msgs Bool on topic with data value"""

    def __init__(self, topic, value):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.topic = topic
        self.pub = None
        self.quad_monitor = None  # set externally if needed
        # setup internal msgs
        msg = msgs.Bool()
        msg.data = value
        self.msg = msg

    def execute(self, _userdata):
        node = _node(self.quad_monitor) if self.quad_monitor is not None else None
        if node is None:
            return "failed"
        try:
            if self.pub is None:
                self.pub = node.create_publisher(msgs.Bool, self.topic, 1)
            self.pub.publish(self.msg)
        except Exception as e:
            node.get_logger().error(
                f"Error thrown while executing condition callback: {e}"
            )
            return "failed"
        return "succeeded"


class TrackerTransition(smach_ros.ServiceState):
    def execute(self, _userdata):
        node = _node(self.quad_monitor)
        if node is None:
            return "aborted"
        try:
            while not self.prox.wait_for_service(timeout_sec=1.0):
                node.get_logger().info(f"Waiting for service {self.service_name}")
            self.prox.call_async(self.req)
            return "succeeded"
        except Exception as e:
            node.get_logger().error(
                f"Error thrown while executing condition callback {str(self.execute)}: {e}"
            )
            return "aborted"

    def __init__(self, service, tracker, quad_monitor, safety_critical=False):
        """Wraps service state with a tracker transition"""
        self.quad_monitor = quad_monitor
        self.safety_critical = safety_critical
        smach.State.__init__(self, outcomes=["succeeded", "aborted", "preempted"])
        self.req = TM.Transition.Request()
        self.req.tracker = tracker
        # Preserve the namespace handling: a leading "/" indicates absolute topic
        node = _node(quad_monitor)
        namespace = node.get_namespace() if node is not None else "/"
        if not namespace.endswith("/"):
            namespace += "/"
        self.service_name = namespace + service
        if node is not None:
            self.prox = node.create_client(TM.Transition, self.service_name)
        else:
            self.prox = None


# Define States Which Need Extra Data (ex. home, goal, etc)
class TakingOff(smach_ros.SimpleActionState):
    """docstring for TakeOff"""

    def goal_cb(self, _userdata, goal):
        # need to generate goal at run time
        goal.height = self.quad_tracker.takeoff_height
        return goal

    def __init__(self, action_topic, quad_tracker):
        self.quad_tracker = quad_tracker
        node = _node(quad_tracker)
        namespace = node.get_namespace() if node is not None else "/"
        if not namespace.endswith("/"):
            namespace += "/"
        smach_ros.SimpleActionState.__init__(
            self, namespace + action_topic, AT.TakeOffAction, goal_cb=self.goal_cb
        )


class SetHomeHere(smach.State):
    def __init__(self, quad_tracker):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.quad_tracker = quad_tracker

    def execute(self, _userdata):
        try:
            self.quad_tracker.set_home_from_air()
        except Exception as e:
            node = _node(self.quad_tracker)
            if node is not None:
                node.get_logger().error(f"Error thrown while executing set home here {e}")
            return "failed"
        return "succeeded"


class Landing(smach_ros.SimpleActionState):
    """docstring for Landing"""

    def goal_cb(self, _userdata, goal):
        # need to generate goal at run time
        # goal.goal = self.quad_tracker.get_ground_msg()
        return goal

    def __init__(self, action_topic, quad_tracker):
        self.quad_tracker = quad_tracker
        node = _node(quad_tracker)
        namespace = node.get_namespace() if node is not None else "/"
        if not namespace.endswith("/"):
            namespace += "/"
        smach_ros.SimpleActionState.__init__(
            self, namespace + action_topic, AT.LandAction, goal_cb=self.goal_cb
        )
