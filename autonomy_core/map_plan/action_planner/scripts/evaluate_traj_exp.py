#!/usr/bin/env python3
import rospy
# from kr_planning_msgs.msg import SplineTrajectory
from kr_planning_msgs.msg import PlanTwoPointAction, PlanTwoPointGoal, VoxelMap
from kr_mav_msgs.msg import PositionCommand, OutputData
from kr_tracker_msgs.msg import PolyTrackerGoal, PolyTrackerAction, LineTrackerAction, LineTrackerGoal
import numpy as np
# import matplotlib.pyplot as plt
import pandas as pd
from copy import deepcopy
from visualization_msgs.msg import MarkerArray, Marker
from actionlib import SimpleActionClient
from std_srvs.srv import Empty, Trigger
from kr_tracker_msgs.srv import Transition
from nav_msgs.msg import Odometry
import random 
import actionlib
# from std_msgs.msg import Bool
from std_msgs.msg import Int32
from tqdm import tqdm
import pickle


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
    def __init__(self):
        # print("reading "+filename)
        # self.start_goals = pd.read_csv(filename)
        # self.path_pub = rospy.Publisher('/local_plan_server0/plan_local_trajectory/goal', PlanTwoPointActionGoal, queue_size=10, latch=True)
        # self.client = SimpleActionClient('/local_plan_server0/plan_local_trajectory', PlanTwoPointAction)
        self.client_list = []
        self.num_planners = 5
        for i in range(self.num_planners): #  0, 1, 2, ... not gonna include the one with no suffix
            self.client_list.append(SimpleActionClient('/local_plan_server'+str(i)+'/plan_local_trajectory', PlanTwoPointAction))
        # self.client2 = SimpleActionClient('/local_plan_server2/plan_local_trajectory', PlanTwoPointAction)
        # self.client3 = SimpleActionClient('/local_plan_server3/plan_local_trajectory', PlanTwoPointAction)

        self.start_and_goal_pub = rospy.Publisher('/start_and_goal', MarkerArray, queue_size=10, latch=True)
        self.mav_name     = rospy.get_param("/local_plan_server0/mav_name")
        self.map_name     =  rospy.get_param("/local_plan_server0/map_name")
        self.wait_for_things = rospy.get_param("/local_plan_server0/trajectory_planner/use_tracker_client")
        self.fix_start_end_location = self.map_name == "read_grid_map"
        
        self.map_origin_x = rospy.get_param('/' + self.map_name + "/map/x_origin")
        self.map_origin_y = rospy.get_param('/' + self.map_name + "/map/y_origin")
        self.map_origin_z = rospy.get_param('/' + self.map_name + "/map/z_origin")

        self.map_range_x = rospy.get_param('/' + self.map_name + "/map/x_size")
        self.map_range_y = rospy.get_param('/' + self.map_name + "/map/y_size")
        self.map_range_z = rospy.get_param('/' + self.map_name + "/map/z_size")
        
        self.set_state_pub      = rospy.Publisher( '/' + self.mav_name + '/set_state', PositionCommand, queue_size=1, latch=False)
        # self.client_tracker = actionlib.SimpleActionClient('/quadrotor/trackers_manager/poly_tracker/PolyTracker', PolyTrackerAction)
        self.client_line_tracker = actionlib.SimpleActionClient('/' + self.mav_name + '/trackers_manager/line_tracker_min_jerk/LineTracker', LineTrackerAction)
        self.change_map_pub     = rospy.Publisher('/' + self.map_name + '/change_map', Int32, queue_size=1)

        print("waiting for tracker trigger service")
        rospy.wait_for_service(poly_service_name)
        # self.poly_trigger = rospy.ServiceProxy(poly_service_name, Trigger)
        self.transition_tracker = rospy.ServiceProxy(line_service_name, Transition)
        rospy.Subscriber('/' + self.mav_name + "/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mapper/local_voxel_map", VoxelMap, self.point_clouds_callback)
        self.effort_temp = 0.0 # these need to be defined before the callback
        self.effort_counter = 0
        rospy.Subscriber("/quadrotor/quadrotor_simulator_so3/output_data", OutputData, self.sim_output_callback)
        rospy.logwarn("Change topic name of OutputData on real quadrotor")

        # rospy.Subscriber("/local_plan_server0/trajectory", SplineTrajectory, self.callback)
        self.num_trials = 5
        self.success = np.zeros((self.num_trials, self.num_planners), dtype=bool)
        self.success_detail = np.zeros((self.num_trials, self.num_planners), dtype=int)
        self.traj_time = np.zeros((self.num_trials, self.num_planners))
        self.traj_cost = np.zeros((self.num_trials, self.num_planners))
        self.traj_jerk = np.zeros((self.num_trials, self.num_planners))
        self.traj_compute_time = np.zeros((self.num_trials, self.num_planners))
        self.compute_time_front = np.zeros((self.num_trials, self.num_planners))
        self.compute_time_back = np.zeros((self.num_trials, self.num_planners))
        self.tracking_error = np.zeros((self.num_trials, self.num_planners))
        self.effort = np.zeros((self.num_trials, self.num_planners)) #unit in rpm
        self.rho = 50  # TODO(Laura) pull from param or somewhere
    
        self.publisher()




    def sample_in_map(self):

        curr_sample_idx = 0

        while curr_sample_idx < 100:
        
            rand_start_x = random.uniform(self.map_origin_x, self.map_range_x + self.map_origin_x)
            rand_start_y = random.uniform(self.map_origin_y, self.map_range_y + self.map_origin_y)
            rand_start_z = random.uniform(self.map_origin_z + 0.2, self.map_range_z + self.map_origin_z - 0.2)

            rand_goal_x = random.uniform(self.map_origin_x, self.map_range_x + self.map_origin_x)
            rand_goal_y = random.uniform(self.map_origin_y, self.map_range_y + self.map_origin_y)
            rand_goal_z = random.uniform(self.map_origin_z + 0.2, self.map_range_z + self.map_origin_z - 0.2)

            start = np.array([rand_start_x, rand_start_y, rand_start_z])
            goal = np.array([rand_goal_x, rand_goal_y, rand_goal_z])
            

            curr_sample_idx += 1
            dis = np.linalg.norm(start - goal)
            if dis > 0.8 * self.map_range_x: 
                break
                #print("start and goal are too close! Skipping...")
                # continue
        if curr_sample_idx == 100:
            rospy.logerr("Failed to sample a start and goal pair far enough apart, consider changing the map size")

        return start, goal


    def odom_callback(self, msg):
        self.odom_data = msg.pose.pose.position

    def sim_output_callback(self, msg): #ToDo: This also has odom inside, consider combine with previous callback 
        self.effort_temp += np.mean(msg.motor_rpm) #rpm
        self.effort_counter += 1

    def point_clouds_callback(self, msg):


        return

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
        start.header.stamp = rospy.Time.now()
        start.pose.position = msg.p_init.position
        start.pose.orientation.w = 1
        start.color.g = 1
        start.color.a = 1
        start.type = 2
        start.scale.x = start.scale.y = start.scale.z = 1
        goal = deepcopy(start)
        goal.pose.position = msg.p_final.position
        goal.id = 1
        goal.color.r = 1
        goal.color.g = 0
        start_and_goal.markers.append(start)
        start_and_goal.markers.append(goal)
        # self.path_pub.publish(msg)
        self.start_and_goal_pub.publish(start_and_goal) # viz
    def publisher(self):

        print("waiting for action server")
        for  i in range(self.num_planners):
            self.client_list[i].wait_for_server()

        for i in tqdm(range(self.num_trials)):
            if rospy.is_shutdown():
                break
            ######## CHANGE MAP ######
            random.seed(i)

            msg = Int32()
            msg.data = i
            self.change_map_pub.publish(msg) # this is only active when using structure map
            rospy.sleep(1.0) # maze map is still reading files sequentially
                # When change_map returns, the map is changed, but becuase delay, wait a little longer
            
            ####### DEFINE START #####
            # define start location not actually sending pos_msg since we using a tracker to get there
            if not use_odom_bool: # hopefully this is always the case, we can specify the start
                pos_msg = PositionCommand() # change position in simulator
                pos_msg.header.frame_id = "map"
                pos_msg.header.stamp = rospy.Time.now()

                if self.fix_start_end_location:
                    start = np.array([-9.5, -4, 1.0])
                    end = np.array([9.5, 4, 1.0])
                else:
                    start, end = self.sample_in_map()

                pos_msg.position.x = start[0]
                pos_msg.position.y = start[1]
                pos_msg.position.z = start[2]

                pos_msg.velocity.x = 0
                pos_msg.velocity.y = 0
                pos_msg.velocity.z = 0
                pos_msg.yaw = random.uniform(-np.pi,np.pi)

            for client_idx in range(self.num_planners):
                client = self.client_list[client_idx]
                ##### GO TO START #####
                if not use_odom_bool and self.wait_for_things:  #this needs to be done for every client 
                    traj_act_msg = LineTrackerGoal()
                    traj_act_msg.x = pos_msg.position.x
                    traj_act_msg.y = pos_msg.position.y
                    traj_act_msg.z = pos_msg.position.z
                    traj_act_msg.yaw = pos_msg.yaw
                    traj_act_msg.v_des = 0.0
                    traj_act_msg.a_des = 0.0
                    traj_act_msg.relative = False
                    traj_act_msg.t_start = rospy.Time.now()
                    traj_act_msg.duration = rospy.Duration(line_tracker_flight_time)
                    self.client_line_tracker.send_goal(traj_act_msg)# first change tracker goal msg
                    #wait while tracker's goal is not received
                    while True:
                        rospy.sleep(0.1)
                        state = self.client_line_tracker.get_state()
                        rospy.loginfo_throttle(f"Waiting for line tracker goal. Current State: {state}")
                        if state == 1:
                            rospy.loginfo("Line Tracker Goal Received")
                            break
                    # state = self.client_line_tracker.get_state() # make sure it received it
                    # print(f"After sent goal: Action State: {state}")
                    response = self.transition_tracker('kr_trackers/LineTrackerMinJerk')
                    # self.set_state_pub.publish(pos_msg) #then change state so no error remain

                    print(response)

                    self.client_line_tracker.wait_for_result(rospy.Duration.from_sec(line_tracker_flight_time + 3.0)) #flying
                    response = self.client_line_tracker.get_result()
                    if response is not None:
                        rospy.loginfo("Line Tracker Finished")
                    else:
                        rospy.logerr("Line Tracker Failed")


                ##### SET GOAL #####
                msg = PlanTwoPointGoal()
                if use_odom_bool:
                    msg.p_init.position = self.odom_data # if starting from current position
                    msg.p_final.position.z = self.odom_data.z # this is usually hardware, so z is more sensitive
                else:
                    msg.p_init.position = pos_msg.position # if starting from random position
                    msg.p_final.position.z = end[2] # this is not hardware, so set it to whatever
                # set goal to be random
                msg.p_final.position.x = end[0]
                msg.p_final.position.y = end[1]

                self.send_start_goal_viz(msg)

                client.send_goal(msg) #motion
                self.effort_temp = 0.0 # 
                self.effort_counter = 1 # to avoid divide by zero

                # Waits for the server to finish performing the action.
                if self.wait_for_things:
                    client.wait_for_result(rospy.Duration.from_sec(20.0)) 
                else:
                    client.wait_for_result(rospy.Duration.from_sec(4.0)) 
           
                # stop accumulating the effort
                self.effort[i,client_idx] = self.effort_temp / self.effort_counter
                result = client.get_result()

                #TODO(Laura) check if the path is collision free and feasible
                if result:
                    self.success[i,client_idx] = result.success
                    tqdm.write("Solve Status: trial "+ str(i) + " planner: " + str(client_idx) + " status: "+ str(result.policy_status))
                    self.success_detail[i,client_idx] = result.policy_status
                    # print(result.odom_pts) #@Yuwei: this should work, try this out! 
                    # Odom is also returned in result.odom_pts
                    if result.computation_time > 0:
                        self.traj_compute_time[i,client_idx] = result.computation_time
                        self.compute_time_front[i,client_idx] = result.compute_time_front_end
                        self.compute_time_back[i,client_idx] = result.compute_time_back_end
                        self.tracking_error[i,client_idx] = result.tracking_error
                    if result.success:
                        self.traj_time[i,client_idx] = result.traj.data[0].t_total
                        # self.traj_cost[i,client_idx] = self.computeCost(result.traj, self.rho)
                        self.traj_jerk[i,client_idx] = self.computeJerk(result.traj)

                else:
                    print("Action server failure trial" + str(i), "client" + str(client_idx))
                    self.success_detail[i,client_idx] = -1
            
            # input("Press Enter to continue...")

        print(self.success)
        print(self.success_detail)
        print("Traj Time", self.traj_time)
        print("Traj Cost",self.traj_cost)
        print("Jerk", self.traj_jerk)
        print("Compute Time", self.traj_compute_time)
        print("Compute Time Front", self.compute_time_front)
        print("Compute Time Back", self.compute_time_back)
        print("Tracking Error", self.tracking_error)
        print("Effort", self.effort)

        #save pickle with all the data, use date time as name
        with open('ECI_eval_data_'+str(rospy.get_time())+'.pkl', 'wb') as f:
            pickle.dump([self.success, self.success_detail, self.traj_time, self.traj_cost, self.traj_jerk, self.traj_compute_time, self.compute_time_front, self.compute_time_back, self.tracking_error, self.effort], f)
        

        print("success details: ", self.success_detail)
        print("success rate: " + str(np.sum(self.success)/self.success.size)+ " out of " + str(self.success.size))
        print("avg traj time(s): " + str(np.sum(self.traj_time[self.success]) / np.sum(self.success)))
        # print("avg traj cost(time + jerk): " + str(np.sum(self.traj_cost[self.success]) / np.sum(self.success)))
        print("avg traj jerk: " + str(np.sum(self.traj_jerk[self.success]) / np.sum(self.success)))
        print("avg compute time(ms): " + str(np.sum(self.traj_compute_time[self.success]) / np.sum(self.success)))
        print("avg compute time front(ms): " + str(np.sum(self.compute_time_front[self.success]) / np.sum(self.success)))
        print("avg compute time back(ms): " + str(np.sum(self.compute_time_back[self.success]) / np.sum(self.success)))
        print("avg tracking error(m): " + str(np.sum(self.tracking_error[self.success]) / np.sum(self.success)))
        print("avg effort(rpm): " + str(np.sum(self.effort) / self.effort.size))

def subscriber():
    rospy.init_node('evaluate_traj')
    Evaluater()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
