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

    # replan_rate parameter should be set in the ros parameter YAML file under the state_machine field
    if rospy.has_param("~state_machine/replan_rate"):
        replan_rate = rospy.get_param("~state_machine/replan_rate")
        print("[State machine:] Setting replan_rate as:", replan_rate)
    else:
        raise Exception("[State machine:] state_machine/replan_rate is not set in the ros param YAML file!")

    # max_replan_trials parameter should be set in the ros parameter YAML file under the state_machine field
    if rospy.has_param("~state_machine/max_replan_trials"):
        max_replan_trials = rospy.get_param("~state_machine/max_replan_trials")
        print("[State machine:] Setting max_replan_trials as:", max_replan_trials)
    else:
        raise Exception("[State machine:] state_machine/max_replan_trials is not set in the ros param YAML file!")

    # Create holder for tracker object
    quad_tracker = QuadTracker(rospy.names.get_namespace() + "abort")
    quad_tracker.replan_rate = replan_rate
    quad_tracker.max_replan_trials = max_replan_trials
    quad_tracker.avoid = True  # obstacle avoidance in planner

    # Seconds to wait for the stopping policy to finish, should be large enough so that the robot fully stops
    # THIS IS VERY SAFETY CRITICAL! DO NOT CHANGE UNLESS YOU ARE SURE!      
    # TODO(xu:) get feedback from stopping policy, instead of hard-coding a wait time
    wait_for_stop = 5.0

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["done"])
    with sm:

        # IdleTransition state, activating take off tracker to prepare for taking off
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

        # off state
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

        # motor on state
        smach.StateMachine.add(
            "MotorOn",
            PublishBoolMsgState("motors", True),
            transitions={
                "succeeded": "WaitForOne",
                "failed": "Off"
            },
        )

        # wait state, sleep for 1 second
        smach.StateMachine.add("WaitForOne",
                               WaitState(1),
                               transitions={"done": "Idle"})

        # mavros arm state
        smach.StateMachine.add(
            "MavrosArm",
            ArmDisarmMavros("mavros/cmd/arming", True),
            transitions={
                "succeeded": "WaitForOne",
                "failed": "Off"
            },
        )

        # mavros disarm state
        smach.StateMachine.add(
            "MavrosDisarm",
            ArmDisarmMavros("mavros/cmd/arming", False),
            transitions={
                "succeeded": "Off",
                "failed": "Off"
            },
        )

        # motor off state
        smach.StateMachine.add(
            "MotorOff",
            PublishBoolMsgState("motors", False),
            transitions={
                "succeeded": "MavrosDisarm",
                "failed": "MotorOff"
            },
        )

        # idle state
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

        # taking off state
        smach.StateMachine.add(
            "TakingOff",
            TakingOff("trackers_manager/take_off", quad_tracker),
            transitions={
                "succeeded": "Hover",
                "aborted": "Idle",
                "preempted": "Idle"
            },
        )

        # landing state
        smach.StateMachine.add(
            "Landing",
            Landing("trackers_manager/land", quad_tracker),
            transitions={
                "succeeded": "Idle",
                "aborted": "Hover",
                "preempted": "Hover"
            },
        )

        # Hover state
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

        # LandTransition state, activating land tracker
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


        # GetMPWaypoints state, processing waypoints for motion primitive based replanner:
        smach.StateMachine.add('GetMPWaypoints',
                               GetWaypoints(quad_tracker),
                               transitions={
                                   'succeeded': 'ExecuteMotionPrimitive',
                                   'multi': 'ExecuteMotionPrimitive',
                                   'failed': 'Hover'
                               })

        # ExecuteMotionPrimitive state, entering replanner (sub-)state-machine
        smach.StateMachine.add(
            "ExecuteMotionPrimitive",
            Replanner.REPLANNER(quad_tracker),
            transitions={
                "succeeded": "Hover",
                "failed": "RetryMPWaypoints"
            },
        )

        # RetryMPWaypoints state, re-entering ExecuteMotionPrimitive state if RetryWaypoints returns true (i.e. trail times < pre-defined max_trail_time);
        # Otherwise, calling stopping policy and transiting to hover.
        smach.StateMachine.add('RetryMPWaypoints',
                               RetryWaypoints(quad_tracker, wait_for_stop),
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
