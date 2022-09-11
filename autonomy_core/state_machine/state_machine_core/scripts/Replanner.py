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
from std_msgs.msg import Int8
from std_msgs.msg import String

from nav_msgs.msg import Odometry
import geometry_msgs.msg as GM

from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R

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







################################################################################ RePlan Class #######################################################################################
class RePlan(smach_ros.SimpleActionState):
    def __init__(self, action_topic, quad_monitor):
        smach_ros.SimpleActionState.__init__(
            self, action_topic, SM.ReplanAction, goal_cb=self.goal_cb, result_cb=self.result_cb
        )


        # enable exploration or not (if True, will reactively explore the environment when a cuboid message is published)
        self.perform_exploration = False   
        self.active_localization = False
        self.closest_cylinder_xy = None

        self.default_waypoint_z_height = 5

        self.quad_monitor = quad_monitor

        self.waypoint_idx_sub = rospy.Subscriber("/waypoint_idx", Int8, self.waypoint_idx_cb, queue_size=5)

        self.current_executing_waypoint_idx = None
    
        self.exploration_state_trigger = rospy.Publisher("replan_state_trigger",String, queue_size=1)

        self.prev_pub_time = rospy.Time.now().secs

        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.exploration_waypoints = None
        self.start_new_exploration = False
        self.explored_cuboid_xy = []

        self.last_active_localization_time = None
        self.active_localization_requested = False
        self.active_localization_max_interval = 300 # seconds

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb, queue_size=10)
        if self.perform_exploration:
            data_dir = "/home/dcist/bags/stats-for-testing/"
            filename = "asslam_front_driver_counter_clockwise.txt"
            self.active_mapping_waypoints = np.loadtxt(data_dir + filename)
            self.car_cuboids_sub = rospy.Subscriber("/car_cuboids", MarkerArray, callback=self.car_cuboids_cb, queue_size=1)
        if self.active_localization:
            # active localization will be carried out either
            # (1) the previous active localization call has been more than self.active_localization_max_interval, or 
            # (2) active_localization_trigger_cb is called (when covariance is too large, generic sloam will trigger this)

            # from generic sloam
            self.car_cuboids_sub = rospy.Subscriber("/sloam/map_cylinder_models", MarkerArray, callback=self.cylinder_cb, queue_size=1)
            # from generic sloam, when covariance is larger than a threshold
            self.car_cuboids_sub = rospy.Subscriber("/sloam/active_localization_trigger", String, callback=self.active_localization_trigger_cb, queue_size=1)

        
        # 0 means no exploration is in progress
        # 1 means exploration phase 1 is yet to start, meaning that it is the first observation of the cuboid (unstable)
        # 2 means exploration phase 1 is in progress, meaning that it is going to the first waypoint of the cuboid (unstable)
        # 3 means exploration phase 1 is done, and phase 2 is yet to start, meaning that the first waypoint of the cuboid is reached and cuboid stabilized, replan to explore all 4 waypoints of the stable cuboid
        # 4 means exploration phase 1 is done, and phase 2 is in progress, exploring the all 4 waypoints of the stable cuboid
        # will go back to 0 when phase 2 is done
        self.exploration_status = 0
        # self.exploration_phase_1_in_progress = False
        # self.exploration_phase_2_in_progress = False
        self.exploration_start_idx = None
        
    
    def odom_cb(self, msg):
        self.robot_x = np.round(msg.pose.pose.position.x, 4)
        self.robot_y = np.round(msg.pose.pose.position.y, 4)
        self.robot_z = np.round(msg.pose.pose.position.z, 4)
        if self.perform_exploration:
            self.explore()
        if self.active_localization:
            if self.last_active_localization_time is not None:
                if rospy.Time.now().secs - self.last_active_localization_time > self.active_localization_max_interval:
                    self.active_localization_requested = True
            self.active_localize()
            
    
    def active_localization_trigger_cb(self,msg):
        self.active_localization_requested = True

    def active_localize(self):
        if self.active_localization_requested:
            if self.closest_cylinder_xy is not None: 
                print('Cylinder suitable for active localization is found, now performing active localization!')
                additional_waypoint = [[self.closest_cylinder_xy[0],self.closest_cylinder_xy[1],self.default_waypoint_z_height]]
                self.update_waypoints_trigger_exploration(additional_waypoint)
                # reset flag
                self.active_localization_requested = False
                # reset timer
                self.last_active_localization_time = rospy.Time.now().secs
            else:
                print('Cylinder suitable for active localization is NOT found, skipping active localization! This should NOT happen except for the very beginning where no trees/light poles are observed!')


    def explore(self):
        if (self.current_executing_waypoint_idx is None) or (self.exploration_waypoints is None):
            print('Not initialized enough for exploration...')
            return

        exploration_waypoints = self.exploration_waypoints
        # exploration_waypoints = [[self.robot_x + 20, self.robot_y, 5], [self.robot_x+20, self.robot_y+ 20, 5], [self.robot_x-20, self.robot_y+20, 5], [self.robot_x-20, self.robot_y-20, 5]]
        assert(len(exploration_waypoints) == 4)
        if self.start_new_exploration:
        # if (rospy.Time.now().secs - self.prev_pub_time > 30.0): 
            if self.exploration_status == 0:
            # if (self.exploration_phase_1_in_progress == False) and (self.exploration_phase_2_in_progress == False):
                print('starting exploration...')
                self.exploration_start_idx = self.current_executing_waypoint_idx
                self.original_number_of_waypoints = len(self.quad_monitor.waypoints.poses)
                self.exploration_status = 1
                self.termination_idx_of_phase_1 = self.exploration_start_idx + 1
                self.termination_idx_of_phase_2 = self.exploration_start_idx + len(exploration_waypoints) + 1


            # update waypoints from information_gain_planner
            # check if cuboid is explored
            if (self.exploration_status == 2) and (self.current_executing_waypoint_idx >= self.termination_idx_of_phase_1):
                self.exploration_status = 3
            if (self.exploration_status == 4) and (self.current_executing_waypoint_idx >= self.termination_idx_of_phase_2):
                # de-activate exploration to enable receiving new exploration waypoints
                self.exploration_status = 0
                # self.prev_pub_time = rospy.Time.now().secs
                self.start_new_exploration = False


                
            if self.exploration_status == 1: 
                self.exploration_status = 2
                self.exploration_phase_1_in_progress = True
                waypoint_to_add = copy.deepcopy([exploration_waypoints[0]])
                self.update_waypoints_trigger_exploration(waypoint_to_add)
                print('starting phase 1 of exploration')
            elif self.exploration_status == 3:
                self.exploration_status = 4
                waypoint_to_add = copy.deepcopy(exploration_waypoints)
                self.update_waypoints_trigger_exploration(waypoint_to_add)
                print('starting phase 2 of exploration')



    def cylinder_cb(self, cylinder_msg):
        if self.last_active_localization_time is None:
            # initialize the timer only after when cylinder is received to avoid active localization from the very beginning
            self.last_active_localization_time = rospy.Time.now().secs

        cur_cylinders_xy_yaw = []
        current_detection_range = 18 # no need to revist landmarks closer than current detection range
        min_distance = 10000 # initialization

        if self.robot_x is None:
            print(f'odometry not yet received')
            return
        for cylinder in cylinder_msg.markers:
            # only do this if the marker is submap, instead of current detection
            if cylinder.id > 20000:
                already_explored = False
                R_lidar_odom_to_world = R.from_euler('z', np.pi / 2.0).as_matrix()
                cylinder_x_lidar_odom = cylinder.pose.position.x
                cylinder_y_lidar_odom = cylinder.pose.position.y
                cylinder_pos_world = R_lidar_odom_to_world @ np.array([cylinder_x_lidar_odom, cylinder_y_lidar_odom, 0])
                cylinder_yaw = cylinder.pose.orientation.z
                cylinder_x_world = cylinder_pos_world[0]
                cylinder_y_world = cylinder_pos_world[1]
                cylinder_yaw_world = cylinder_yaw + np.pi / 2.0
                cur_cylinders_xy_yaw.append((cylinder_x_world, cylinder_y_world, cylinder_yaw_world))
                diff_x = cylinder_x_world - self.robot_x
                diff_y = cylinder_y_world - self.robot_y
                distance = np.linalg.norm(np.array([diff_x, diff_y]))
                if distance > current_detection_range and distance < min_distance:
                    min_distance = distance
                    self.closest_cylinder_xy = np.array([cylinder_x_world,cylinder_y_world])





    def car_cuboids_cb(self, car_cuboids_msg):

        cur_cuboids_xy_yaw = []
        explored_threshold = 5
        threshold_to_explore = 15

        if self.robot_x is None:
            print(f'odometry not yet received')
            return
        for car_cuboid in car_cuboids_msg.markers:
            already_explored = False
            R_lidar_odom_to_world = R.from_euler('z', np.pi / 2.0).as_matrix()
            cuboid_x_lidar_odom = car_cuboid.pose.position.x 
            cuboid_y_lidar_odom = car_cuboid.pose.position.y
            cuboid_pos_world = R_lidar_odom_to_world @ np.array([cuboid_x_lidar_odom, cuboid_y_lidar_odom, 0])
            cuboid_yaw = car_cuboid.pose.orientation.z 
            cuboid_x_world = cuboid_pos_world[0]
            cuboid_y_world = cuboid_pos_world[1]
            cuboid_yaw_world = cuboid_yaw + np.pi / 2.0
            cur_cuboids_xy_yaw.append((cuboid_x_world, cuboid_y_world, cuboid_yaw_world))

            # check if this cuboid is already explored
            for cur_explored_cuboid in self.explored_cuboid_xy:
                diff_x = cuboid_x_world - cur_explored_cuboid[0]
                diff_y = cuboid_y_world - cur_explored_cuboid[1]
                distance = np.linalg.norm(np.array([diff_x, diff_y]))
                if distance < explored_threshold:
                    print('already explored')
                    already_explored = True
                    break
                

            if already_explored == False:
                # check if this cuboid has distance less than desired threshold
                diff_x = cuboid_x_world - self.robot_x
                diff_y = cuboid_y_world - self.robot_y
                distance = np.linalg.norm(np.array([diff_x, diff_y]))
                if distance < threshold_to_explore:
                    self.explored_cuboid_xy.append((cuboid_x_world, cuboid_y_world))
                    R_world2cube = R.from_euler('z', -cuboid_yaw_world).as_matrix()
                    robot_in_cuboid_frame = R_world2cube @ np.array([self.robot_x, self.robot_y, self.robot_z])
                    quadrant = None
                    x = robot_in_cuboid_frame[0] 
                    y = robot_in_cuboid_frame[1] 
                    if x > 0:
                        if y > 0:
                            # front driver
                            quadrant = 1 
                        else: 
                            # front passenger
                            quadrant = 4
                    else:
                        if y > 0:
                            # rear driver
                            quadrant = 2
                        else:
                            # rear passenger
                            quadrant = 3

                    start_idx = (quadrant - 1) * 4
                    end_idx = start_idx + 4
                    xy_point_to_explore = self.active_mapping_waypoints[start_idx:end_idx, :2]
                    print('valid cuboid added, starting exploration...')
                    self.exploration_waypoints = []
                    for row in xy_point_to_explore: 
                        
                        self.exploration_waypoints.append([cuboid_x_world + row[0], cuboid_y_world +row[1], self.default_waypoint_z_height])
                    self.start_new_exploration = True
                    break

                    

        # cur_cuboids_xy_yaw = np.array(cur_cuboids_xy_yaw)
        # # check if this cuboid is already explored
        # for cur_explored_cuboid in self.explored_cuboid_xy:
        #     xy_diff = cur_cuboids_xy_yaw[:,:2] - cur_explored_cuboid
        #     distances = np.linalg.norm(xy_diff, axis=1)


        # # check if this cuboid has distance less than desired threshold
        # min_distance_cuboid = 
            


        
    def update_waypoints_trigger_exploration(self, exploration_waypoints):
        # the all waypoints will include [original_reached_waypoints, exploration_waypoints, original_future_waypoints]
        self.quad_monitor.pose_goals= []
        original_all_waypoints = copy.deepcopy(self.quad_monitor.waypoints.poses)
        self.quad_monitor.waypoints.poses = []
        original_cur_future_waypoints = []

        for cur_wp_idx, cur_wp in enumerate(original_all_waypoints):
            if cur_wp_idx < self.current_executing_waypoint_idx:
                # reached waypoints
                self.quad_monitor.waypoints.poses.append(original_all_waypoints[cur_wp_idx])
            else:
                # original current and future waypoints
                original_cur_future_waypoints.append(original_all_waypoints[cur_wp_idx])

        for cur_wp in exploration_waypoints:
            posC = GM.PoseStamped()
            posC.pose.position.x = cur_wp[0]
            posC.pose.position.y = cur_wp[1]
            posC.pose.position.z = cur_wp[2]
            self.quad_monitor.waypoints.poses.append(posC)

        for cur_wp in original_cur_future_waypoints:
            self.quad_monitor.waypoints.poses.append(cur_wp)
        
        
        self.quad_monitor.pose_goals = [it.pose for it in self.quad_monitor.waypoints.poses]

        self.pub_exploration_state_trigger()

    def pub_exploration_state_trigger(self):
        self.exploration_state_trigger.publish("switch_to_exploration")

        

    def waypoint_idx_cb(self, msg):
        self.current_executing_waypoint_idx = msg.data
        print('For exploration: current_executing_waypoint_idx is ', self.current_executing_waypoint_idx)

    def goal_cb(self, _userdata, goal):
        # rospy.logerr(type(goal.p_init))
        goal.p_init = copy.deepcopy(self.quad_monitor.get_curr_poseC())
        goal.p_final = copy.deepcopy(self.quad_monitor.pose_goal)
        if self.quad_monitor.avoid is None:
            goal.avoid_obstacles = True # default avoid obstacle
        else:
            goal.avoid_obstacles = self.quad_monitor.avoid

        if self.quad_monitor.pose_goals is not None:
            # loading the waypoints with exploration waypoints
            # self.quad_monitor.pose_goals = [it.pose for it in
            #     self.quad_monitor.waypoints.poses]
            goal.p_finals = self.quad_monitor.pose_goals
            # TODO: add exploration waypoints here, but need to insert in the proper index!
            
        else:
            print("pose_goals is None, single waypoint mode (exploration is not supported)")
            # if exploration_on 
            # TODO: add exploration into waypoints, and pose_goal as the last element

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

################################################################################ RePlan Class ends #######################################################################################




















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
