from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading

import roslib
import smach
import rospy
import nav_msgs.msg as NM

roslib.load_manifest("smach_ros")

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

        self._sub = rospy.Subscriber(self._topic, self._topic_type, self._cb, callback_args=ud)

        self._trigger_event.wait()
        self._sub.unregister()

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
                        rospy.logerr("Setting path")
                        self.quad_monitor.pose_goal = msg.path_data.poses[1].pose

        self._trigger_event.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
