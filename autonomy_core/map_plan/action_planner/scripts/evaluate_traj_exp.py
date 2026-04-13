#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
# from kr_planning_msgs.msg import SplineTrajectory
from kr_planning_msgs.action import PlanTwoPoint
from kr_planning_msgs.msg import VoxelMap
from kr_mav_msgs.msg import PositionCommand, OutputData
from kr_tracker_msgs.action import PolyTracker, LineTracker
import numpy as np
# import matplotlib.pyplot as plt
import pandas as pd
from copy import deepcopy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Empty, Trigger
from kr_tracker_msgs.srv import Transition
from nav_msgs.msg import Odometry
import random
# from std_msgs.msg import Bool
from std_msgs.msg import Int32
from tqdm import tqdm
import pickle
import yaml
import csv

import pcl
#sudo apt install python3-pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from datetime import datetime
from param_env_msgs.srv import changeMap



poly_service_name = "/quadrotor/mav_services/poly_tracker"
line_service_name = "/quadrotor/trackers_manager/transition"
use_odom_bool = False
multi_front_end = False
line_tracker_flight_time = 6.0 # seconds

# filename = '/home/laura/autonomy_ws/src/kr_autonomous_flight/autonomy_core/map_plan/action_planner/scripts/map_balls_start_goal.csv'


def differentiate(p, segment_time):
    v = np.zeros(p.size - 1)
    for i in range(1, p.size):
        v[i-1] = (p[i] * i / segment_time)
    return v


def evaluate(msg, t, deriv_num):
    result = np.zeros(msg.dimensions)

    for dim in range(msg.dimensions):
        spline = msg.data[dim]
        dt = 0
        for poly in spline.segs:
            poly_coeffs = np.array(poly.coeffs)
            # print(poly_coeffs)
            for d in range(deriv_num):
                poly_coeffs = differentiate(poly_coeffs, poly.dt)
            result[dim] = poly_coeffs[0]

            if (t < dt + poly.dt or poly == spline.segs[-1]):
                for j in range(1, poly_coeffs.size):
                    result[dim] += poly_coeffs[j] * ((t - dt) / poly.dt) ** j
                break
            dt += poly.dt
    return result


class Evaluater:
    def __init__(self, node):
        self.node = node
        # print("reading "+filename)
        # self.start_goals = pd.read_csv(filename)
        # self.path_pub = node.create_publisher(PlanTwoPointActionGoal, '/local_plan_server0/plan_local_trajectory/goal', 10)
        # self.client = ActionClient(node, PlanTwoPoint, '/local_plan_server0/plan_local_trajectory')
        self.client_list = []
        self.client_name_list = []
        self.client_name_front_list = []
        self.client_name_back_list = []

        self.num_planners = 10
        self.num_trials = 3
        for i in range(self.num_planners): #  0, 1, 2, ... not gonna include the one with no suffix
            self.client_list.append(ActionClient(node, PlanTwoPoint, '/local_plan_server'+str(i)+'/plan_local_trajectory'))
             # self.client2 = ActionClient(node, PlanTwoPoint, '/local_plan_server2/plan_local_trajectory')
        # self.client3 = ActionClient(node, PlanTwoPoint, '/local_plan_server3/plan_local_trajectory')

        self.start_and_goal_pub = node.create_publisher(MarkerArray, '/start_and_goal', 10)


        node.declare_parameter('/local_plan_server0/mav_name', '')
        node.declare_parameter('/local_plan_server0/map_name', '')
        node.declare_parameter('/local_plan_server0/mav_radius', 0.0)
        node.declare_parameter('/local_plan_server0/trajectory_planner/use_tracker_client', False)
        self.mav_name    = node.get_parameter('/local_plan_server0/mav_name').value
        self.map_type    = node.get_parameter('/local_plan_server0/map_name').value
        self.mav_radius  = node.get_parameter('/local_plan_server0/mav_radius').value


        self.wait_for_things = node.get_parameter('/local_plan_server0/trajectory_planner/use_tracker_client').value
        node.declare_parameter('/' + self.map_type + "/map/mode", 0)
        self.map_read_mode = node.get_parameter('/' + self.map_type + "/map/mode").value

        self.fix_start_end_location = (self.map_type == "read_grid_map" and self.map_read_mode == 1) # or structure_map

        node.declare_parameter('/' + self.map_type + "/map/x_origin", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/y_origin", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/z_origin", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/x_size", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/y_size", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/z_size", 0.0)
        node.declare_parameter('/' + self.map_type + "/map/inflate_radius", 0.0)
        self.map_origin_x = node.get_parameter('/' + self.map_type + "/map/x_origin").value
        self.map_origin_y = node.get_parameter('/' + self.map_type + "/map/y_origin").value
        self.map_origin_z = node.get_parameter('/' + self.map_type + "/map/z_origin").value

        self.map_range_x = node.get_parameter('/' + self.map_type + "/map/x_size").value
        self.map_range_y = node.get_parameter('/' + self.map_type + "/map/y_size").value
        self.map_range_z = node.get_parameter('/' + self.map_type + "/map/z_size").value

        self.inflate_radius = node.get_parameter('/' + self.map_type + "/map/inflate_radius").value

        self.set_state_pub      = node.create_publisher(PositionCommand, '/' + self.mav_name + '/set_state', 1)
        # self.client_tracker = ActionClient(node, PolyTracker, '/quadrotor/trackers_manager/poly_tracker/PolyTracker')
        self.client_line_tracker = ActionClient(node, LineTracker, '/' + self.mav_name + '/trackers_manager/line_tracker_min_jerk/LineTracker')
        node.create_subscription(PointCloud2, "global_cloud", self.point_clouds_callback, 10) # this needs to be ready early

        # self.change_map_pub     = node.create_publisher(Int32, '/' + self.map_name + '/change_map', 1)
        self.change_map_pub  = node.create_client(changeMap, '/' + self.map_type + '/change_map')


        print("waiting for tracker trigger service")
        self.poly_service_client = node.create_client(Trigger, poly_service_name)
        while not self.poly_service_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("Waiting for poly tracker service...")
        # self.poly_trigger = node.create_client(Trigger, poly_service_name)
        self.transition_tracker_client = node.create_client(Transition, line_service_name)
        node.create_subscription(Odometry, '/' + self.mav_name + "/odom", self.odom_callback, 10)
        self.effort_temp = 0.0 # these need to be defined before the callback
        self.effort_counter = 0
        node.create_subscription(OutputData, '/' + self.mav_name + "/quadrotor_simulator_so3/output_data", self.sim_output_callback, 10)
        node.get_logger().warn("Change topic name of OutputData on real quadrotor")

        # node.create_subscription(SplineTrajectory, "/local_plan_server0/trajectory", self.callback, 10)
        self.success = np.zeros((self.num_trials, self.num_planners), dtype=bool)
        self.success_detail = -2*np.ones((self.num_trials, self.num_planners), dtype=int)
        self.traj_time = np.zeros((self.num_trials, self.num_planners))
        self.traj_cost = np.zeros((self.num_trials, self.num_planners))
        self.traj_jerk = np.zeros((self.num_trials, self.num_planners))
        self.poly_compute_time = np.zeros((self.num_trials, self.num_planners))
        self.compute_time_front = np.zeros((self.num_trials, self.num_planners))
        self.compute_time_back = np.zeros((self.num_trials, self.num_planners))
        self.tracking_error = np.zeros((self.num_trials, self.num_planners))
        self.effort = np.zeros((self.num_trials, self.num_planners)) #unit in rpm
        self.rho = 50  # TODO(Laura) pull from param or somewhere
        self.collision_front = np.zeros((self.num_trials, self.num_planners), dtype=bool)
        self.collision_cnt = np.zeros((self.num_trials, self.num_planners), dtype=bool)
        self.dist_to_goal = np.zeros((self.num_trials, self.num_planners))

        self.kdtree = None
        self.pcl_data = None

        self.publisher()




    def sample_in_map(self, tol):

        curr_sample_idx = 0
        start_end_feasible = True
        max_iter = 200
        while curr_sample_idx < max_iter:
        
            rand_start_x = random.uniform(self.map_origin_x + 1 ,  self.map_range_x + self.map_origin_x - 1)
            rand_start_y = random.uniform(self.map_origin_y + 1 ,  self.map_range_y + self.map_origin_y - 1)
            rand_start_z = random.uniform(self.map_origin_z + 0.5, self.map_range_z + self.map_origin_z - 0.5)

            rand_goal_x = random.uniform(self.map_origin_x + 1 ,  self.map_range_x + self.map_origin_x - 1)
            rand_goal_y = random.uniform(self.map_origin_y + 1,   self.map_range_y + self.map_origin_y - 1)
            rand_goal_z = random.uniform(self.map_origin_z + 0.5, self.map_range_z + self.map_origin_z - 0.5)

            start = np.array([rand_start_x, rand_start_y, rand_start_z])
            goal = np.array([rand_goal_x, rand_goal_y, rand_goal_z])
            

            curr_sample_idx += 1
            dis = np.linalg.norm(start - goal)
            
            #check dist is far and collision free
            if dis > 0.7 * self.map_range_x and not self.evaluate_collision([Point(x=start[0], y=start[1], z=start[2]), Point(x=goal[0], y=goal[1], z=goal[2])], tol):
                break

        if curr_sample_idx >= max_iter:
            self.node.get_logger().error("Failed to sample a start and goal pair far enough apart && collision free")
            start_end_feasible = False
        return start, goal, start_end_feasible


    def odom_callback(self, msg):
        self.odom_data = msg.pose.pose.position

    def sim_output_callback(self, msg): #ToDo: This also has odom inside, consider combine with previous callback 
        self.effort_temp += np.mean(msg.motor_rpm) #rpm
        self.effort_counter += 1

    def point_clouds_callback(self, msg):
        tqdm.write("point cloud CALLBACK received")
        points_list = []

        for data in pc2.read_points(msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        self.pcl_data = pcl.PointCloud(np.array(points_list, dtype=np.float32))


        self.kdtree = self.pcl_data.make_kdtree_flann()

        return
    

    def evaluate_collision(self, pts, tol = -1.0):
        if tol < 0:
            tol = self.mav_radius
        min_sq_dist = 1000000
        min_idx = 0
        if len(pts) > 200:
            pts = pts[::10]
        # tqdm.write("odom length = " + str( len(pts)))
        if self.kdtree is None:
            time.sleep(0.1)
            self.node.get_logger().error("No KD Tree, skipping collision check")
            return True # assume collision
        for pt_idx in range(len(pts)):
            pt = pts[pt_idx]
            sp = pcl.PointCloud()
            sps = np.zeros((1, 3), dtype=np.float32)
            sps[0][0] = pt.x
            sps[0][1] = pt.y
            sps[0][2] = pt.z
            sp.from_array(sps)
            [ind, sqdist] = self.kdtree.nearest_k_search_for_cloud(sp, 1) #which pointcloud pt has min dist
            if sqdist[0][0] < min_sq_dist:

                min_sq_dist = sqdist[0][0]
                min_idx = pt_idx

        # tqdm.write("min dist = " + str(np.sqrt(min_sq_dist)) + "@ traj percentage = " + str(min_idx/len(pts)))
        if np.sqrt(min_sq_dist) < tol:

            if tol == self.mav_radius:
                self.node.get_logger().warn("Collision Detected")
                print("np.sqrt(min_sq_dist) is ", np.sqrt(min_sq_dist))
            return True
        else:
            # print("min dist is ", np.sqrt(min_sq_dist))
            return False            

    def computeJerk(self, traj):
        # creae empty array for time
        t_vec = np.array([])
        # create empty array for jerk norm sq
        jerk_sq = np.array([])
        # jerk = 0
        dt = .01
        for t in np.arange(0, traj.data[0].t_total, dt):
            t_vec = np.append(t_vec, t)
            jerk_sq = np.append(jerk_sq, (np.linalg.norm(evaluate(traj, t, 3)))**2 )
        return np.sqrt(np.trapz(jerk_sq, t_vec))/traj.data[0].t_total
        

    def computeCost(self, traj, rho):
        time = traj.data[0].t_total
        cost = rho*time + self.computeJerk(traj)
        return cost
    def send_start_goal_viz(self, msg):
        start_and_goal = MarkerArray()
        start = Marker()
        start.header.frame_id = "map"
        start.header.stamp = self.node.get_clock().now().to_msg()
        start.pose.position = msg.p_init.position
        start.pose.orientation.w = 1.0
        start.color.g = 1.0
        start.color.a = 1.0
        start.type = 2
        start.scale.x = start.scale.y = start.scale.z = 1.0
        goal = deepcopy(start)
        goal.pose.position = msg.p_final.position
        goal.id = 1
        goal.color.r = 1.0
        goal.color.g = 0.0
        start_and_goal.markers.append(start)
        start_and_goal.markers.append(goal)
        # self.path_pub.publish(msg)
        self.start_and_goal_pub.publish(start_and_goal) # viz
    def publisher(self):
        self.node.declare_parameter('/local_plan_server0/trajectory_planner/search_planner_text', [''])
        self.node.declare_parameter('/local_plan_server0/trajectory_planner/opt_planner_text', [''])
        search_planner_text = self.node.get_parameter('/local_plan_server0/trajectory_planner/search_planner_text').value
        opt_planner_text = self.node.get_parameter('/local_plan_server0/trajectory_planner/opt_planner_text').value
        print("Running ", self.num_planners, "planner combinations for", self.num_trials, "trials", "on map", self.map_type)
        for i in range(self.num_planners):
            print("waiting for action server ", i)
            self.client_list[i].wait_for_server()
            self.node.declare_parameter('/local_plan_server'+str(i)+'/trajectory_planner/search_planner_type', 0)
            self.node.declare_parameter('/local_plan_server'+str(i)+'/trajectory_planner/opt_planner_type', 0)
            planner_name_front = search_planner_text[self.node.get_parameter('/local_plan_server'+str(i)+'/trajectory_planner/search_planner_type').value]
            planner_name_back  = opt_planner_text[self.node.get_parameter('/local_plan_server'+str(i)+'/trajectory_planner/opt_planner_type').value]
            self.client_name_front_list.append(planner_name_front)
            self.client_name_back_list.append(planner_name_back)
            self.client_name_list.append(planner_name_front + '+'+ planner_name_back)


        print("All action server connected, number of planners = ", self.num_planners)
        now = datetime.now()
        file_name_save_time = now.strftime("%m-%d_%H-%M-%S")

        # TODO: ROS2 has no direct equivalent to rospy.get_param_names() for arbitrary
        # global parameters; only parameters declared on this node are visible here.
        params = {}
        for name in self.node._parameters.keys() if hasattr(self.node, '_parameters') else []:
            try:
                params[name] = self.node.get_parameter(name).value
            except Exception:
                pass

        with open('ECI_params_'+file_name_save_time+'.yaml', 'w') as f:
            yaml.dump(params, f)

        try:
            with open('ECI_single_line_'+file_name_save_time+'.csv', 'w') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['map_type', 'map_seed','map_filename', 'density_index', 'clutter_index', 'structure_index',
                                     'start_end_feasible', 'planner_frontend', 'planner_backend', 
                                     'success', 'success_detail', 'traj_time(s)', 'traj_length(m)', 'traj_jerk', 'traj_effort(rpm)',
                                     'compute_time_poly(ms)', 'compute_time_frontend(ms)', 'compute_time_backend(ms)', 
                                     'tracking_error(m) avg', 'collision_frontend', 'collision_status','dist_to_goal(m)'])

                for i in tqdm(range(self.num_trials)):
                    if not rclpy.ok():
                        break
                    ######## CHANGE MAP ######
                    seed_val = i +3000
                    random.seed(seed_val)


                    change_map_req = changeMap.Request()
                    change_map_req.seed = seed_val
                    change_map_future = self.change_map_pub.call_async(change_map_req) # this is only active when using structure map
                    rclpy.spin_until_future_complete(self.node, change_map_future)
                    map_response = change_map_future.result()
                    time.sleep(7) # maze map is still reading files sequentially
                        # When change_map returns, the map is changed, but becuase delay, wait a little longer
                    # print(map_response)
                    start_end_feasible = True
                    ####### DEFINE START #####
                    # define start location not actually sending pos_msg since we using a tracker to get there
                    if not use_odom_bool: # hopefully this is always the case, we can specify the start
                        pos_msg = PositionCommand() # change position in simulator
                        pos_msg.header.frame_id = "map"
                        pos_msg.header.stamp = self.node.get_clock().now().to_msg()

                        if self.fix_start_end_location:
                            start = np.array([-9.0, -4, 1.0])
                            end = np.array([9.0, 4, 1.0])
                        else:
                            start, end, start_end_feasible = self.sample_in_map(tol = self.mav_radius + self.inflate_radius + 0.1) #0.1 for fillMap_inflation
                        if not start_end_feasible:
                            continue
                        pos_msg.position.x = start[0]
                        pos_msg.position.y = start[1]
                        pos_msg.position.z = start[2]

                        pos_msg.velocity.x = 0.0
                        pos_msg.velocity.y = 0.0
                        pos_msg.velocity.z = 0.0
                        pos_msg.yaw = random.uniform(-np.pi,np.pi)


                        ##### GO TO START #####
                    if not use_odom_bool and self.wait_for_things:  #this needs to be done for every client
                        traj_act_msg = LineTracker.Goal()
                        traj_act_msg.x = pos_msg.position.x
                        traj_act_msg.y = pos_msg.position.y
                        traj_act_msg.z = pos_msg.position.z
                        traj_act_msg.yaw = pos_msg.yaw
                        traj_act_msg.v_des = 0.0
                        traj_act_msg.a_des = 0.0
                        traj_act_msg.relative = False
                        traj_act_msg.t_start = self.node.get_clock().now().to_msg()
                        traj_act_msg.duration = Duration(seconds=line_tracker_flight_time).to_msg()
                        line_send_goal_future = self.client_line_tracker.send_goal_async(traj_act_msg)# first change tracker goal msg
                        rclpy.spin_until_future_complete(self.node, line_send_goal_future)
                        line_goal_handle = line_send_goal_future.result()
                        if line_goal_handle is not None and line_goal_handle.accepted:
                            tqdm.write("Line Tracker Goal Received")
                        else:
                            self.node.get_logger().info("Waiting for line tracker goal.")
                        # Call the transition_tracker service
                        transition_req = Transition.Request()
                        transition_req.tracker = 'kr_trackers/LineTrackerMinJerk'
                        transition_future = self.transition_tracker_client.call_async(transition_req)
                        rclpy.spin_until_future_complete(self.node, transition_future)
                        response = transition_future.result()
                        # self.set_state_pub.publish(pos_msg) #then change state so no error remain

                        tqdm.write(response.message)

                        if line_goal_handle is not None and line_goal_handle.accepted:
                            get_result_future = line_goal_handle.get_result_async()
                            rclpy.spin_until_future_complete(self.node, get_result_future,
                                                              timeout_sec=line_tracker_flight_time + 3.0) #flying
                            result_wrapper = get_result_future.result()
                            response = result_wrapper.result if result_wrapper is not None else None
                        else:
                            response = None
                        if response is not None:
                            tqdm.write("Line Tracker Finished")
                        else:
                            tqdm.write("Line Tracker Failed!!!!!!!!!!!!!")


                        ##### SET GOAL #####
                    msg = PlanTwoPoint.Goal()
                    if use_odom_bool:
                        msg.p_init.position = self.odom_data # if starting from current position
                        msg.p_final.position.z = self.odom_data.z # this is usually hardware, so z is more sensitive
                    else:
                        msg.p_init.position = pos_msg.position # if starting from random position
                        msg.p_final.position.z = end[2] # this is not hardware, so set it to whatever
                    # set goal to be random
                    msg.p_final.position.x = end[0]
                    msg.p_final.position.y = end[1]
                    msg.check_vel = False

                    self.send_start_goal_viz(msg)
                    goal_handles = []
                    for client_idx in range(self.num_planners):
                        client = self.client_list[client_idx]
                        send_goal_future = client.send_goal_async(msg) #motion
                        rclpy.spin_until_future_complete(self.node, send_goal_future)
                        goal_handles.append(send_goal_future.result())
                    result_list = []
                    valid_result = True
                    for client_idx in range(self.num_planners):
                        goal_handle = goal_handles[client_idx]

                        self.effort_temp = 0.0 #
                        self.effort_counter = 1 # to avoid divide by zero

                        # Waits for the server to finish performing the action.
                        result = None
                        if goal_handle is not None and goal_handle.accepted:
                            get_result_future = goal_handle.get_result_async()
                            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=20.0)
                            if get_result_future.done():
                                result_wrapper = get_result_future.result()
                                result = result_wrapper.result if result_wrapper is not None else None

                        # stop accumulating the effort
                        self.effort[i,client_idx] = self.effort_temp / self.effort_counter
                        if not result:
                            tqdm.write("Server Failure: trial " + str(i) + " planner: " + str(client_idx))
                            valid_result = False
                            break
                        result_list.append(result)
                    if valid_result:
                        for client_idx in range(self.num_planners):
                            result = result_list[client_idx]
                            #TODO(Laura) check if the path is collision free and feasible
                            if result:
                                self.success[i,client_idx] = result.success
                                tqdm.write("Solve Status: trial "+ str(seed_val) + " planner: " + str(client_idx) + " status: "+ str(result.policy_status))
                                self.success_detail[i,client_idx] = result.policy_status
                                # print(result.odom_pts) #@Yuwei: this should work, try this out! 
                                # Odom is also returned in result.odom_pts
                                    # rospy.loginfo(result.odom_pts)

                                self.poly_compute_time[i,client_idx] = result.computation_time
                                self.compute_time_front[i,client_idx] = result.compute_time_front_end
                                self.compute_time_back[i,client_idx] = result.compute_time_back_end
                                self.tracking_error[i,client_idx] = result.tracking_error
                                if result.success:
                                
                                    self.traj_time[i,client_idx] = result.traj.data[0].t_total
                                    # self.traj_cost[i,client_idx] = self.computeCost(result.traj, self.rho)
                                    self.traj_jerk[i,client_idx] = self.computeJerk(result.traj)
                                    if ~self.wait_for_things: # if no tracking then check collision of the planned traj
                                        result.odom_pts.clear()
                                        for t in np.arange(0, result.traj.data[0].t_total, 0.02):
                                            pos_t = evaluate(result.traj, t, 0)
                                            pos_t_pt = Point()
                                            pos_t_pt.x = pos_t[0]
                                            pos_t_pt.y = pos_t[1]
                                            pos_t_pt.z = pos_t[2]

                                            result.odom_pts.append(pos_t_pt)
                                        self.collision_cnt[i,client_idx] = self.evaluate_collision(result.odom_pts)
                                        result.odom_pts.clear()
                                        for t in np.arange(0, result.search_traj.data[0].t_total, 0.02):
                                            pos_t = evaluate(result.search_traj, t, 0)
                                            pos_t_pt = Point()
                                            pos_t_pt.x = pos_t[0]
                                            pos_t_pt.y = pos_t[1]
                                            pos_t_pt.z = pos_t[2]

                                            result.odom_pts.append(pos_t_pt)
                                        self.collision_front[i,client_idx] = self.evaluate_collision(result.odom_pts)
                                        
                                    else:
                                        self.collision_cnt[i,client_idx] = self.evaluate_collision(result.odom_pts)
                                    

                                    traj_end_point = np.zeros(3, dtype=np.float32)
                                    traj_end_point[0] = result.odom_pts[-1].x
                                    traj_end_point[1] = result.odom_pts[-1].y
                                    traj_end_point[2] = result.odom_pts[-1].z


                                    self.dist_to_goal[i, client_idx] = np.linalg.norm(end - traj_end_point)
    
                            else:
                                tqdm.write("Server Failure: trial " + str(i) + " planner: " + self.client_name_front_list[client_idx]+ self.client_name_back_list[client_idx])
                                self.success_detail[i,client_idx] = -1
                            
                    #dont have traj length, so just put 0
                            csv_writer.writerow([self.map_type, seed_val, map_response.file_name.data, map_response.density_index, map_response.clutter_index, map_response.structure_index,
                                                start_end_feasible, self.client_name_front_list[client_idx], self.client_name_back_list[client_idx],
                                                self.success[i,client_idx], self.success_detail[i,client_idx], self.traj_time[i,client_idx], 0.0, self.traj_jerk[i,client_idx], self.effort[i,client_idx],
                                                self.poly_compute_time[i,client_idx], self.compute_time_front[i,client_idx], self.compute_time_back[i,client_idx],
                                                self.tracking_error[i,client_idx], self.collision_front[i,client_idx], self.collision_cnt[i,client_idx], self.dist_to_goal[i, client_idx]])

        except KeyboardInterrupt:
            tqdm.write("Keyboard Interrupt!")


        #save results
        # TODO: ROS2 has no direct equivalent to rospy.get_param_names() for arbitrary
        # global parameters; only parameters declared on this node are visible here.
        params = {}
        for name in self.node._parameters.keys() if hasattr(self.node, '_parameters') else []:
            try:
                params[name] = self.node.get_parameter(name).value
            except Exception:
                pass
        data_all = {}

        data_all['success'] = self.success
        data_all['success_detail'] = self.success_detail
        data_all['traj_time'] = self.traj_time
        data_all['traj_cost'] = self.traj_cost
        data_all['traj_jerk'] = self.traj_jerk
        data_all['poly_compute_time'] = self.poly_compute_time
        data_all['compute_time_front'] = self.compute_time_front
        data_all['compute_time_back'] = self.compute_time_back
        data_all['tracking_error'] = self.tracking_error
        data_all['effort'] = self.effort
        data_all['collision_front'] = self.collision_front
        data_all['collision_cnt'] = self.collision_cnt
        data_all['dist_to_goal'] = self.dist_to_goal

        print(self.success)
        print("Legend: -2: not run, -1: server failure, 0: front failure, 1: front success, 2: poly success, 3: back success")
        print(self.success_detail)
        print("Traj Time", self.traj_time)
        print("Traj Cost",self.traj_cost)
        print("Jerk", self.traj_jerk)
        print("Compute Time Front", self.compute_time_front)
        print("Compute Time Poly", self.poly_compute_time)
        print("Compute Time Back", self.compute_time_back)
        print("Tracking Error", self.tracking_error)
        print("Effort", self.effort)
        print("Is Collide Front", self.collision_front)
        print("Is Collide", self.collision_cnt)
        print("Distance To Goal", self.dist_to_goal)



        #save pickle with all the data, use date time as name
        # with open('ECI_eval_data_'+file_name_save_time+'.pkl', 'wb') as f:
        #     pickle.dump([self.success, self.success_detail, self.traj_time, self.traj_cost, self.traj_jerk, self.traj_compute_time, self.compute_time_front, self.compute_time_back, self.tracking_error, self.effort], f)
        with open('ECI_Result_' + file_name_save_time + '.pkl', 'wb') as f:
            pickle.dump(data_all, f)# result and config

        #create variables to store the average values
        success_front_rate = np.sum(self.success_detail >= 1, axis = 0)/self.success.shape[0]
        success_rate_avg = np.sum(self.success,axis = 0)/self.success.shape[0]
        traj_time_avg = np.sum(self.traj_time,axis = 0) / np.sum(self.success, axis = 0)
        # traj_cost_avg = np.sum(self.traj_cost[self.success]) / np.sum(self.success)
        traj_jerk_avg = np.sum(self.traj_jerk, axis = 0) / np.sum(self.success, axis = 0)
        poly_compute_time_avg = np.sum(self.poly_compute_time, axis = 0) / np.sum(self.success, axis = 0)
        compute_time_front_avg = np.sum(self.compute_time_front, axis = 0) / np.sum(self.success, axis = 0)
        compute_time_back_avg = np.sum(self.compute_time_back, axis = 0) / np.sum(self.success, axis = 0)
        tracking_error_avg = np.sum(self.tracking_error, axis = 0) / np.sum(self.success, axis = 0)
        effort_avg = np.sum(self.effort, axis = 0) / np.sum(self.success, axis = 0)
        collision_rate_avg = np.sum(self.collision_front, axis = 0) / np.sum(self.success, axis = 0)
        collision_rate_avg = np.sum(self.collision_cnt, axis = 0) / np.sum(self.success, axis = 0)
        dist_to_goal_avg = np.sum(self.dist_to_goal, axis = 0) / np.sum(self.success, axis = 0)
        # rewrite the above section with defined avg variables
        print("frontend success rate: " + str(success_front_rate)+ " out of " + str(self.success.shape[0]))
        print("success rate: " + str(success_rate_avg)+ " out of " + str(self.success.shape[0]))
        print("avg traj time(s): " + str(traj_time_avg))
        # print("avg traj cost(time + jerk): " + str(traj_cost_avg))
        print("avg traj jerk: " + str(traj_jerk_avg))
        print("avg compute time front(ms): " + str(compute_time_front_avg))
        print("avg compute time poly(ms): " + str(poly_compute_time_avg))
        print("avg compute time back(ms): " + str(compute_time_back_avg))
        print("avg tracking error(m): " + str(tracking_error_avg))
        print("avg effort(rpm): " + str(effort_avg))# this is bugg!! need to consider success
        print("avg dist to goal: " + str(dist_to_goal_avg))
        print("collision rate: " + str(collision_rate_avg))


        
        # save the avg values to a csv file by appending to the end of the file
        csv_name = 'ECI_Summary_'+file_name_save_time+'.csv'

        with open(csv_name, 'w') as f: #result summary
            writer = csv.writer(f)
            writer.writerow(['Map:'+self.map_type+ ' Run:' + str(self.num_trials),'success rate', 'frontend success','traj time', 'traj jerk', 'compute time(ms)', 'compute time front(ms)', 'compute time back(ms)', 'tracking error(m)', 'effort(rpm)', 'collision rate'])
            for i in range(self.num_planners):
                writer.writerow([self.client_name_list[i], success_rate_avg[i], success_front_rate[i], traj_time_avg[i], traj_jerk_avg[i], poly_compute_time_avg[i], compute_time_front_avg[i], compute_time_back_avg[i], tracking_error_avg[i], effort_avg[i], collision_rate_avg[i]])
           

def subscriber():
    rclpy.init()
    node = rclpy.create_node('evaluate_traj')
    Evaluater(node)

    # spin() simply keeps python from exiting until this node is stopped
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        subscriber()
    except KeyboardInterrupt:
        pass
