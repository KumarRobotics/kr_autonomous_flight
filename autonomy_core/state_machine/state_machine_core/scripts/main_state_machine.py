#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import smach
import smach_ros
import planning_ros_msgs.msg as MHL

from MainStates import GetWaypoints, RetryWaypoints, WaitState, ArmDisarmMavros, PublishBoolMsgState, TrackerTransition, TakingOff, SetHomeHere, Landing 
from SwitchState import SwitchState

from QuadTracker import QuadTracker
import Replanner
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
    replan_rate = 2.0 
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
                ["land_here", "waypoints"],
                quad_tracker,
            ),
            transitions={
                "invalid": "Hover",
                "preempted": "Hover",
                "land_here": "SetHomeHere",
                "waypoints":
                "GetMPWaypoints"
            },
        )

        # Set home position for land_here command
        smach.StateMachine.add(
            "SetHomeHere",
            SetHomeHere(quad_tracker),
            transitions={
                "succeeded": "LandTransition",
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

        smach.StateMachine.add(
            "ExecuteMotionPrimitive",
            Replanner.REPLANNER(quad_tracker),
            transitions={
                "succeeded": "RetryMPWaypoints",
                "no_path": "RetryMPWaypoints",
                "failed": "RetryMPWaypoints"
            },
        )

        # for motion primitive planner:
        smach.StateMachine.add('RetryMPWaypoints',
                               RetryWaypoints(quad_tracker),
                               transitions={
                                   'succeeded': 'ExecuteMotionPrimitive',
                                   'multi': 'ExecuteMotionPrimitive',
                                   'failed': 'Hover'
                               })
        

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
