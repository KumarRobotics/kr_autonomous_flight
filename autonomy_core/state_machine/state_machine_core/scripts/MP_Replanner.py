#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import smach
import smach_ros
import state_machine.msg as SM
from Utils import *


# Yaw-related classes (CheckYaw AlignYaw YawSearch) removed (exist in autonomy_stack repo before 8/16/2020).

class StoppingPolicyDone(smach.State):
    def __init__(self, quad_monitor):
        smach.State.__init__(self, outcomes=["done"])
        self.quad_monitor = quad_monitor

    def execute(self, userdata):
        return "done"

class CheckRePlan(smach.State):
    def __init__(self, quad_monitor):
        smach.State.__init__(
            self,
            outcomes=[
                "success",
                "critical_error",
                "abort",
                "no_path",
                "abort_full_mission",
            ],
        )
        self.quad_monitor = quad_monitor

    def execute(self, userdata):
        if self.quad_monitor.abort:
            return "abort"
        if self.quad_monitor.replan_status is None:
            return "critical_error"
        return self.quad_monitor.replan_status


class RePlan(smach_ros.SimpleActionState):
    def goal_cb(self, userdata, goal):
        # rospy.logerr(type(goal.p_init))
        goal.p_init = copy.deepcopy(self.quad_monitor.get_curr_poseC()) 
        goal.p_final = copy.deepcopy(self.quad_monitor.pose_goal)
        if self.quad_monitor.avoid is None:
          goal.avoid_obstacles = True # default avoid obstacle
        else:
          goal.avoid_obstacles = self.quad_monitor.avoid
          
        if self.quad_monitor.pose_goals is not None:
            goal.p_finals = self.quad_monitor.pose_goals

        goal.replan_rate = self.quad_monitor.replan_rate
        self.quad_monitor.replan_status = None


    def result_cb(self, userdata, status, result):
        # rospy.logwarn(result)
        # rospy.loginfo(result)
        if result.status == 0:
            self.quad_monitor.replan_status = "success"
        elif result.status == 1:
            self.quad_monitor.replan_status = "abort"
            self.quad_monitor.abort = True
        elif result.status == 2:
            self.quad_monitor.replan_status = "no_path"
            self.quad_monitor.abort = True
        elif result.status == 3:
            self.quad_monitor.replan_status = "critical_error"
            self.quad_monitor.abort = True
        elif result.status == 4:
            raise Exception("The IN_PROGRESS result should have been disabled!")
            # self.quad_monitor.replan_status = "in_progress"
        elif result.status == 5:
            self.quad_monitor.replan_status = "abort_full_mission"

        print("Final result is:", self.quad_monitor.replan_status)


    def __init__(self, action_topic, quad_monitor):
        smach_ros.SimpleActionState.__init__(
            self, action_topic, SM.ReplanAction, goal_cb=self.goal_cb, result_cb=self.result_cb
        )
        self.quad_monitor = quad_monitor


class REPLANNER(smach.StateMachine):
    def child_cb(self, outcome_map):
        return True

    def __init__(self, quad_monitor):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "no_path", "failed"])
        safe = True
        self.quad_monitor = quad_monitor
        with self:
            smach.StateMachine.add('TrajTransitionMP', TrackerTransition("trackers_manager/transition","action_trackers/ActionTrajectoryTracker", quad_monitor),
                                   transitions={'succeeded':'RePlan',
                                                'aborted':'RePlan',
                                                'preempted':'RePlan'})

            smach.StateMachine.add(
                "RePlan",
                RePlan("local_global_replan_server/replan", quad_monitor),
                transitions={
                    "succeeded": "CheckRePlan",
                    "aborted": "CheckRePlan",
                    "preempted": "CheckRePlan",
                },
            )
            smach.StateMachine.add(
                "CheckRePlan",
                CheckRePlan(quad_monitor),
                transitions={
                    "success": "StoppingPolicySuccess",
                    "abort": "StoppingPolicy",
                    "no_path": "StoppingPolicyNoPath",
                    "abort_full_mission": "StoppingPolicy",
                    "critical_error": "StoppingPolicy",
                },
            )

            smach.StateMachine.add(
                "StoppingPolicy",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDone",
                    "aborted": "StoppingPolicyDone",
                    "preempted": "StoppingPolicyDone",
                },
            )

            smach.StateMachine.add(
                "StoppingPolicyDone",
                StoppingPolicyDone(quad_monitor),
                transitions={"done": "failed"},
            )
            smach.StateMachine.add(
                "StoppingPolicyNoPath",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDoneNP",
                    "aborted": "StoppingPolicyDoneNP",
                    "preempted": "StoppingPolicyDoneNP",
                },
            )
            smach.StateMachine.add(
                "StoppingPolicyDoneNP",
                StoppingPolicyDone(quad_monitor),
                transitions={"done": "no_path"},
            )

            smach.StateMachine.add(
                "StoppingPolicySuccess",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDoneSuccess",
                    "aborted": "StoppingPolicyDoneSuccess",
                    "preempted": "StoppingPolicyDoneSuccess",
                },
            )
            smach.StateMachine.add(
                "StoppingPolicyDoneSuccess",
                StoppingPolicyDone(quad_monitor),
                transitions={"done": "succeeded"},
            )
