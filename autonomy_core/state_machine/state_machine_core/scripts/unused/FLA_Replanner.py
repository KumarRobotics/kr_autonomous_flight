#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import smach
import smach_ros
from MainStates import *
import geometry_msgs.msg as GM
import kr_mav_msgs.msg as QM
import state_machine.msg as SM

from Utils import *
import tf
import threading
import numpy as np

# Yaw-related classes (CheckYaw AlignYaw YawSearch) removed (exist in autonomy_stack repo before 8/16/2020).


class StoppingPolicyDone(smach.State):
    def __init__(self, quad_monitor):
        smach.State.__init__(self, outcomes=["done"])
        self.quad_monitor = quad_monitor

    def execute(self, userdata):
        # clear all original waypoints for safety  (because no path has been found for them)
        self.quad_monitor.waypoints = None
        print("waypoints cleared!")
        return "done"


class CheckRePlan(smach.State):
    def __init__(self, quad_monitor):
        smach.State.__init__(
            self,
            outcomes=[
                "success",
                "critical_error",
                "abort",
                "in_progress",
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
            self.quad_monitor.replan_status = "in_progress"
        elif result.status == 5:
            self.quad_monitor.replan_status = "abort_full_mission"

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
            # action_trackers/src/path_tracker.cpp
            smach.StateMachine.add(
                "TrajTransitionSR",
                TrackerTransition(
                    "trackers_manager/transition",
                    "action_trackers/ActionPathTracker",
                    quad_monitor,
                ),
                transitions={"succeeded": "RePlan", "aborted": "RePlan", "preempted": "RePlan"},
            )

            smach.StateMachine.add(
                "RePlan",
                RePlan("replanner/replan", quad_monitor),
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
                    "in_progress": "WaitForReplan",
                    "no_path": "StoppingPolicyNoPath",
                    "abort_full_mission": "StoppingPolicy",
                    "critical_error": "StoppingPolicy",
                },
            )
            replan_rate = self.quad_monitor.replan_rate # Planner will run at replan_rate) hz
            smach.StateMachine.add(
                "WaitForReplan", WaitState(1.0 / replan_rate), transitions={"done": "RePlan"}
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


# This is removed from MainStates.py, maybe useful if come back to use JPS + line tracker
    # class PlanPath(smach_ros.SimpleActionState):
    #     def goal_cb(self, userdata, goal):
    #         goal.p_init = self.quad_monitor.get_curr_poseC()
    #         goal.p_final = self.quad_monitor.pose_goal
    #         goal.p_finals = self.quad_monitor.pose_goals

    #         return goal

    #     def result_cb(self, userdata, status, result):
    #         self.quad_monitor.path = result.path

    #     def __init__(self, action_topic, quad_monitor):
    #         smach_ros.SimpleActionState.__init__(
    #             self, action_topic, AP.PlanPathAction, goal_cb=self.goal_cb, result_cb=self.result_cb
    #         )
    #         self.quad_monitor = quad_monitor
