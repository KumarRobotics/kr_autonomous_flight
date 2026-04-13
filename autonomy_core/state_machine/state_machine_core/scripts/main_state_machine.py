#!/usr/bin/env python3

# TODO: smach is ROS1-only. This file will not work under ROS2 until the smach
# ecosystem is ported (smach_ros2 or yasmin). The rospy -> rclpy translation
# below is best-effort; the smach state machine itself needs a separate
# migration.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from multiprocessing.pool import ThreadPool
import rclpy
from rclpy.node import Node
import smach
import smach_ros
import kr_planning_msgs.msg as MHL

from MainStates import GetWaypoints, RetryWaypoints, WaitState, ArmDisarmMavros, PublishBoolMsgState, TrackerTransition, TakingOff, SetHomeHere, Landing
from SwitchState import SwitchState

from QuadTracker import QuadTracker
import Replanner

# state naming conventions
# UPPER_CASE - state machines or states which are state machines
# CammelCase - regular states
# lower_case - transitions between states


def main():

    rclpy.init()
    node = rclpy.create_node("smach_state_machine")

    # local_replan_rate parameter should be set in the ros parameter YAML file under the state_machine field
    if node.has_parameter("state_machine/local_replan_rate"):
        local_replan_rate = node.get_parameter("state_machine/local_replan_rate").value
        print("[State machine:] Setting local_replan_rate as:", local_replan_rate)
    else:
        # Attempt to declare and read, raising if still unset
        node.declare_parameter("state_machine/local_replan_rate", rclpy.Parameter.Type.DOUBLE)
        try:
            local_replan_rate = node.get_parameter("state_machine/local_replan_rate").value
        except Exception:
            raise Exception("[State machine:] state_machine/local_replan_rate is not set in the ros param YAML file!")
        print("[State machine:] Setting local_replan_rate as:", local_replan_rate)

    # global_replan_rate_factor parameter should be set in the ros parameter YAML file under the state_machine field
    if node.has_parameter("state_machine/global_replan_rate_factor"):
        global_replan_rate_factor = node.get_parameter("state_machine/global_replan_rate_factor").value
        print("[State machine:] Setting global_replan_rate_factor as:", global_replan_rate_factor)
    else:
        node.declare_parameter("state_machine/global_replan_rate_factor", rclpy.Parameter.Type.DOUBLE)
        try:
            global_replan_rate_factor = node.get_parameter("state_machine/global_replan_rate_factor").value
        except Exception:
            raise Exception("[State machine:] state_machine/global_replan_rate_factor is not set in the ros param YAML file!")
        print("[State machine:] Setting global_replan_rate_factor as:", global_replan_rate_factor)

    # max_replan_trials parameter should be set in the ros parameter YAML file under the state_machine field
    if node.has_parameter("state_machine/max_replan_trials"):
        max_replan_trials = node.get_parameter("state_machine/max_replan_trials").value
        print("[State machine:] Setting max_replan_trials as:", max_replan_trials)
    else:
        node.declare_parameter("state_machine/max_replan_trials", rclpy.Parameter.Type.INTEGER)
        try:
            max_replan_trials = node.get_parameter("state_machine/max_replan_trials").value
        except Exception:
            raise Exception("[State machine:] state_machine/max_replan_trials is not set in the ros param YAML file!")
        print("[State machine:] Setting max_replan_trials as:", max_replan_trials)

    # Create holder for tracker object
    namespace = node.get_namespace()
    if not namespace.endswith("/"):
        namespace += "/"
    quad_tracker = QuadTracker(node, namespace + "abort")
    quad_tracker.local_replan_rate = local_replan_rate
    quad_tracker.global_replan_rate_factor = global_replan_rate_factor
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
        motor_on_state = PublishBoolMsgState("motors", True)
        motor_on_state.quad_monitor = quad_tracker
        smach.StateMachine.add(
            "MotorOn",
            motor_on_state,
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
        mavros_arm_state = ArmDisarmMavros("mavros/cmd/arming", True)
        mavros_arm_state.quad_monitor = quad_tracker
        smach.StateMachine.add(
            "MavrosArm",
            mavros_arm_state,
            transitions={
                "succeeded": "WaitForOne",
                "failed": "Off"
            },
        )

        # mavros disarm state
        mavros_disarm_state = ArmDisarmMavros("mavros/cmd/arming", False)
        mavros_disarm_state.quad_monitor = quad_tracker
        smach.StateMachine.add(
            "MavrosDisarm",
            mavros_disarm_state,
            transitions={
                "succeeded": "Off",
                "failed": "Off"
            },
        )

        # motor off state
        motor_off_state = PublishBoolMsgState("motors", False)
        motor_off_state.quad_monitor = quad_tracker
        smach.StateMachine.add(
            "MotorOff",
            motor_off_state,
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

        # LandingExecute state
        smach.StateMachine.add(
            "LandingExecute",
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
                "succeeded": "LandingRequest",
                "failed": "Hover"
            },
        )

        # LandingRequest state, activating land tracker
        smach.StateMachine.add(
            "LandingRequest",
            TrackerTransition("trackers_manager/transition",
                              "action_trackers/LandTracker", quad_tracker),
            transitions={
                "succeeded": "LandingExecute",
                "aborted": "Hover",
                "preempted": "Hover"
            },
        )


        # GetMPWaypoints state, processing waypoints for motion primitive based replanner:
        smach.StateMachine.add('GetMPWaypoints',
                               GetWaypoints(quad_tracker),
                               transitions={
                                   'succeeded': 'EnterReplanner',
                                   'multi': 'EnterReplanner',
                                   'failed': 'Hover'
                               })

        # EnterReplanner state, entering replanner (sub-)state-machine
        # succeeded means that all waypoints have been reached
        smach.StateMachine.add(
            "EnterReplanner",
            Replanner.REPLANNER(quad_tracker, wait_for_stop),
            transitions={
                "succeeded": "Hover", # stay hovering
                "aborted": "Hover", # TODO(xu) better to reset num_trials in RetryWaypoints class to 1 too
                # "succeeded": "LandingRequest", # land automatically
                "failed": "RetryMPWaypoints"
            },
        )

        # RetryMPWaypoints state, re-entering EnterReplanner state if RetryWaypoints returns true (i.e. trail times < pre-defined max_trail_time);
        # Otherwise, calling stopping policy and transiting to hover.
        smach.StateMachine.add('RetryMPWaypoints',
                               RetryWaypoints(quad_tracker),
                               transitions={
                                   'succeeded': 'EnterReplanner',
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # stop the introspection server
    sis.stop()

    # Request the container to preempt
    sm_with_monitor.request_preempt()

    smach_thread.wait()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
