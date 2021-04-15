#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
import planning_ros_msgs.msg as MHL

# from Helpers import *
from MainStates import *
from SwitchState import *

from QuadTracker import *
from Replanner import *
import MP_Replanner

import threading
from multiprocessing.pool import ThreadPool

# state naming conventions
# UPPER_CASE - state machines or states which are state machines
# CammelCase - regular states
# lower_case - transitions between states


def main():

    rospy.init_node("smach_state_machine")

    # Create holder for tracker object

    quad_tracker = QuadTracker(rospy.names.get_namespace() + "abort")
    # specify replan rate, this will be recorded in the goal msg, as well as
    replan_rate = 1
    quad_tracker.replan_rate = replan_rate
    quad_tracker.avoid = True  # obstacle avoidance in planner

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["done"])
    with sm:
        # Open the container

        # On and Off States:
        smach.StateMachine.add(
            "IdleTransition",
            TrackerTransition("trackers_manager/transition",
                              "action_trackers/TakeOffTracker", quad_tracker),
            transitions={
                "succeeded": "Off",
                "aborted": "Off",
                "preempted": "Off"
            },
        )
        smach.StateMachine.add(
            "Off",
            SwitchState("state_trigger", MHL.StateTransition, ["motors_on"],
                        quad_tracker),
            transitions={
                "invalid": "Off",
                "preempted": "Off",
                "motors_on": "MotorOn"
            },
        )

        smach.StateMachine.add(
            "MotorOn",
            PublishBoolMsgState("motors", True),
            transitions={
                "succeeded": "WaitForOne",
                "failed": "Off"
            },
        )

        smach.StateMachine.add("WaitForOne",
                               WaitState(1),
                               transitions={"done": "Idle"})

        smach.StateMachine.add(
            "MavrosArm",
            ArmDisarmMavros("mavros/cmd/arming", True),
            transitions={
                "succeeded": "WaitForOne",
                "failed": "Off"
            },
        )

        smach.StateMachine.add(
            "MavrosDisarm",
            ArmDisarmMavros("mavros/cmd/arming", False),
            transitions={
                "succeeded": "Off",
                "failed": "Off"
            },
        )

        smach.StateMachine.add(
            "MotorOff",
            PublishBoolMsgState("motors", False),
            transitions={
                "succeeded": "MavrosDisarm",
                "failed": "MotorOff"
            },
        )
        smach.StateMachine.add(
            "Idle",
            SwitchState(
                "state_trigger",
                MHL.StateTransition,
                ["motors_off", "takeoff"],
                quad_tracker,
            ),
            transitions={
                "invalid": "Idle",
                "preempted": "Idle",
                "motors_off": "MotorOff",
                "takeoff": "TakingOff",
            },
        )

        # Up and Down States:
        smach.StateMachine.add(
            "TakingOff",
            TakingOff("trackers_manager/take_off", quad_tracker),
            transitions={
                "succeeded": "Hover",
                "aborted": "Idle",
                "preempted": "Idle"
            },
        )

        smach.StateMachine.add(
            "Landing",
            Landing("trackers_manager/land", quad_tracker),
            transitions={
                "succeeded": "Idle",
                "aborted": "Hover",
                "preempted": "Hover"
            },
        )
        smach.StateMachine.add(
            "SetHomeHere",
            SetHomeHere(quad_tracker),
            transitions={
                "succeeded": "LandTransition",
                "failed": "Hover"
            },
        )
        smach.StateMachine.add(
            "LandTransition",
            TrackerTransition("trackers_manager/transition",
                              "action_trackers/LandTracker", quad_tracker),
            transitions={
                "succeeded": "Landing",
                "aborted": "Hover",
                "preempted": "Hover"
            },
        )
        # Hover States
        smach.StateMachine.add(
            "Hover",
            SwitchState(
                "state_trigger",
                MHL.StateTransition,
                ["land_here", "short_range", "waypoints", "motion_plan"],
                quad_tracker,
            ),
            transitions={
                "invalid": "Hover",
                "preempted": "Hover",
                "land_here": "SetHomeHere",
                "short_range": "GetShort",
                # "waypoints": "GetWaypoints",
                "waypoints":
                "GetMPWaypoints",  #TODO: temporary change for test mp planner
                "motion_plan": "GetMPWaypoints",
            },
        )

        smach.StateMachine.add(
            "GetWaypoints",
            GetWaypoints(quad_tracker),
            transitions={
                "succeeded": "GetPath",
                "multi": "GetPath",
                "failed": "Hover"
            },
        )

        smach.StateMachine.add(
            "GetShort",
            GetWaypoints(quad_tracker),
            transitions={
                "succeeded": "ShortRange",
                "multi": "ShortRange",
                "failed": "Hover"
            },
        )

        # the tpplanner is launched in map_plan_launch planner.launch
        smach.StateMachine.add(
            "GetPath",
            PlanPath("tpplanner/plan_path", quad_tracker),
            transitions={
                "succeeded": "ShortRange",
                "aborted": "Hover",
                "preempted": "Hover"
            },
        )

        smach.StateMachine.add(
            "ShortRange",
            REPLANNER(quad_tracker),
            transitions={
                "succeeded": "Hover",
                "no_path": "Hover",
                "failed": "Hover"
            },
        )

        # for motion primitive planner:
        smach.StateMachine.add('GetMPWaypoints',
                               GetWaypoints(quad_tracker),
                               transitions={
                                   'succeeded': 'ExecuteMotionPrimitive',
                                   'multi': 'ExecuteMotionPrimitive',
                                   'failed': 'Hover'
                               })
        # smach.StateMachine.add('GetTrajectory', PlanTrajectory("tpplanner/plan_trajectory", quad_tracker),
        #                        transitions={'succeeded':'ExecuteMotionPrimitive',
        #                                     'aborted':'Hover',
        #                                     'preempted':'Hover'})
        # smach.StateMachine.add('GetTrajectoryWaypoints', PlanTrajectoryWaypoints("tpplanner/plan_trajectory", quad_tracker),
        #                        transitions={'succeeded':'ExecuteMotionPrimitive',
        #                                     'aborted':'Hover',
        #                                     'preempted':'Hover'}) # plan_trajectory_waypoints seems deprecated per comments in mp_planner.cpp
        smach.StateMachine.add(
            "ExecuteMotionPrimitive",
            MP_Replanner.REPLANNER(quad_tracker),
            transitions={
                "succeeded": "Hover",
                "no_path": "Hover",
                "failed": "Hover"
            },
        )
        # the following moved to MP_Replanner
        # smach.StateMachine.add('CheckTrajectory', CheckTrajectory( quad_tracker),
        #                        transitions={'succeeded':'TrajTransition',
        #                                     'failed':'Hover'})
        # smach.StateMachine.add('TrajTransition', TrackerTransition("trackers_manager/transition","action_trackers/ActionTrajectoryTracker", quad_tracker),
        #                        transitions={'succeeded':'RunTrajectory',
        #                                     'aborted':'Hover',
        #                                     'preempted':'Hover'})
        # # execute_trajectory defined in action_trackers/src/trajectory_tracker.cpp
        # smach.StateMachine.add('RunTrajectory', RunTrajectory("trackers_manager/execute_trajectory", quad_tracker),
        #                        transitions={'succeeded':'Hover',
        #                                     'preempted':'Hover',
        #                                     'aborted':'Hover'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("introspection_server", sm, "/SM_ROOT")
    sis.start()

    sm_with_monitor = smach.Concurrence(outcomes=["done"],
                                        default_outcome="done")

    with sm_with_monitor:
        smach.Concurrence.add("MAIN", sm)

    pool = ThreadPool(processes=1)
    smach_thread = pool.apply_async(sm_with_monitor.execute)

    # Wait for ctrl-c
    rospy.spin()

    # stop the introspection server
    sis.stop()

    # Request the container to preempt
    sm_with_monitor.request_preempt()

    smach_thread.join()


if __name__ == "__main__":
    main()
