# TODO: smach is ROS1-only. This file will not work under ROS2 until the smach
# ecosystem is ported (smach_ros2 or yasmin). The rospy -> rclpy translation
# below is best-effort; the smach state machine itself needs a separate migration.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading

import smach
import nav_msgs.msg as NM

__all__ = ["SwitchState"]


class SwitchState(smach.State):
    """A state that will check a given ROS topic with type std_msgs/String and prescribed transitions trans
    """

    def __init__(self, topic, topic_type, trans, quad_monitor):
        smach.State.__init__(
            self,
            outcomes=trans + ["invalid", "preempted"],
            input_keys=[],
            output_keys=["transition_data"],
        )

        self._topic = topic
        self.trans = trans
        self._topic_type = topic_type

        self._trigger_event = threading.Event()
        self.quad_monitor = quad_monitor
        self._sub = None
        self.valid = None
        self.next_state = None

    def execute(self, ud):
        # If preempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return "preempted"

        self._trigger_event.clear()

        # quad_monitor is expected to carry a reference to the rclpy Node (set by
        # main_state_machine) so that subscriptions can be created here.
        node = getattr(self.quad_monitor, "node", None)
        if node is None:
            # Fallback: the smach code expects a ROS context; warn and stall.
            self._trigger_event.wait()
            return "invalid"

        self._sub = node.create_subscription(
            self._topic_type, self._topic,
            lambda msg: self._cb(msg, ud), 10
        )

        self._trigger_event.wait()
        node.destroy_subscription(self._sub)

        if self.preempt_requested():
            self.service_preempt()
            return "preempted"

        if self.valid:
            return self.next_state

        return "invalid"

    def _cb(self, msg, ud):
        # check data against all possible inputs
        self.valid = False
        for val in self.trans:
            if val == msg.transition.data:
                self.valid = True
                self.next_state = val
                ud.transition_data = msg.path_data
                if val == "load_trajectory":
                    self.quad_monitor.trajectory = msg.traj
                if type(msg.path_data) == type(NM.Path()):
                    if len(msg.path_data.poses) >= 2:
                        node = getattr(self.quad_monitor, "node", None)
                        if node is not None:
                            node.get_logger().error("Setting path")
                        self.quad_monitor.pose_goal = msg.path_data.poses[1].pose

        self._trigger_event.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
