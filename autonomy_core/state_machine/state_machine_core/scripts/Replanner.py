#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import rospy

import smach
import smach_ros
from MainStates import TrackerTransition
import state_machine.msg as SM


# Yaw-related classes (CheckYaw AlignYaw YawSearch) removed (exist in kr_autonomous_flight repo before 8/16/2020).

class StoppingPolicyDone(smach.State):
    def __init__(self, quad_monitor, wait_for_stop):
        smach.State.__init__(self, outcomes=["done"])
        self.quad_monitor = quad_monitor
        self.wait_for_stop = wait_for_stop

    def execute(self, _userdata):
        print("[state_machine:] waiting for stopping policy to finish, wait time is: ", self.wait_for_stop, " seconds. Change this param in main_state_machine.py if needed.\n")
        rospy.sleep(self.wait_for_stop)
        print("[state_machine:] robot should have COMPLETELY STOPPED, otherwise, increase the wait_for_stop in main_state_machine.py!.\n")
        return "done"

class CheckRePlan(smach.State):
    def __init__(self, quad_monitor):
        smach.State.__init__(
            self,
            outcomes=[
                "success",
                "dynamically_infeasible",
                "critical_error",
                "no_path",
                "abort_full_mission"
            ],
        )
        self.quad_monitor = quad_monitor

    def execute(self, _userdata):
        # if self.quad_monitor.abort:
        #     return "abort"
        if self.quad_monitor.replan_status is None:
            return "critical_error"
        return self.quad_monitor.replan_status


class RePlan(smach_ros.SimpleActionState):
    def goal_cb(self, _userdata, goal):
        # rospy.logerr(type(goal.p_init))
        goal.p_init = copy.deepcopy(self.quad_monitor.get_curr_poseC())
        goal.p_final = copy.deepcopy(self.quad_monitor.pose_goal)
        if self.quad_monitor.avoid is None:
            goal.avoid_obstacles = True # default avoid obstacle
        else:
            goal.avoid_obstacles = self.quad_monitor.avoid

        if self.quad_monitor.pose_goals is not None:
            goal.p_finals = self.quad_monitor.pose_goals

        goal.continue_mission = self.quad_monitor.continue_mission

        goal.local_replan_rate = self.quad_monitor.local_replan_rate
        goal.global_replan_rate_factor = self.quad_monitor.global_replan_rate_factor
        self.quad_monitor.replan_status = None


    def result_cb(self, _userdata, status, result):
        if result.status == 0:
            self.quad_monitor.replan_status = "success"
        elif result.status == 1:
            self.quad_monitor.replan_status = "dynamically_infeasible"
        elif result.status == 2:
            self.quad_monitor.replan_status = "no_path"
        elif result.status == 3:
            self.quad_monitor.replan_status = "critical_error"
        elif result.status == 4:
            raise Exception("The IN_PROGRESS result should have been disabled!")
        elif result.status == 5:
            self.quad_monitor.replan_status = "abort_full_mission"

        print("Current replan result is:", self.quad_monitor.replan_status)

    def __init__(self, action_topic, quad_monitor):
        smach_ros.SimpleActionState.__init__(
            self, action_topic, SM.ReplanAction, goal_cb=self.goal_cb, result_cb=self.result_cb
        )
        self.quad_monitor = quad_monitor


class REPLANNER(smach.StateMachine):
    def child_cb(self, outcome_map):
        return True

    def __init__(self, quad_monitor, wait_for_stop):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])
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
                    "success": "StoppingPolicySucceeded",
                    "dynamically_infeasible": "StoppingPolicyFailed",
                    "no_path": "StoppingPolicyFailed",
                    "critical_error": "StoppingPolicyFailed",
                    "abort_full_mission": "StoppingPolicyAborted",
                },
            )

            # If all waypoints have been reached, exit replanner with "succeeded" outcome
            smach.StateMachine.add(
                "StoppingPolicySucceeded",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDoneSucceeded",
                    "aborted": "StoppingPolicyDoneSucceeded",
                    "preempted": "StoppingPolicyDoneSucceeded",
                },
            )
            smach.StateMachine.add(
                "StoppingPolicyDoneSucceeded",
                StoppingPolicyDone(quad_monitor, wait_for_stop),
                transitions={"done": "succeeded"},
            )


            # If failed, exit replanner with "failed" outcome, the state machine may re-start replanning
            smach.StateMachine.add(
                "StoppingPolicyFailed",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDoneFailed",
                    "aborted": "StoppingPolicyDoneFailed",
                    "preempted": "StoppingPolicyDoneFailed",
                },
            )
            smach.StateMachine.add(
                "StoppingPolicyDoneFailed",
                StoppingPolicyDone(quad_monitor, wait_for_stop),
                transitions={"done": "failed"},
            )



            # If aborted, exit replanner with "aborted" outcome, the state machine will enter hover mode, and will NOT re-start replanning
            smach.StateMachine.add(
                "StoppingPolicyAborted",
                TrackerTransition(
                    "trackers_manager/transition", "action_trackers/StoppingPolicy", quad_monitor
                ),
                transitions={
                    "succeeded": "StoppingPolicyDoneAborted",
                    "aborted": "StoppingPolicyDoneAborted",
                    "preempted": "StoppingPolicyDoneAborted",
                },
            )
            smach.StateMachine.add(
                "StoppingPolicyDoneAborted",
                StoppingPolicyDone(quad_monitor, wait_for_stop),
                transitions={"done": "aborted"},
            )
