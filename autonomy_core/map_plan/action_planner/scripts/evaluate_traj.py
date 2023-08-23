#!/usr/bin/env python3
import rospy
# from kr_planning_msgs.msg import SplineTrajectory
from kr_planning_msgs.msg import PlanTwoPointAction, PlanTwoPointGoal
import numpy as np
# import matplotlib.pyplot as plt
import pandas as pd
from copy import deepcopy
from visualization_msgs.msg import MarkerArray, Marker
from actionlib import SimpleActionClient
from std_srvs.srv import Empty

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
        # self.path_pub = rospy.Publisher('/local_plan_server/plan_local_trajectory/goal', PlanTwoPointActionGoal, queue_size=10, latch=True)
        self.client = SimpleActionClient('/local_plan_server/plan_local_trajectory', PlanTwoPointAction)
        self.client2 = SimpleActionClient('/local_plan_server2/plan_local_trajectory', PlanTwoPointAction)
        self.client3 = SimpleActionClient('/local_plan_server3/plan_local_trajectory', PlanTwoPointAction)

        self.start_and_goal_pub = rospy.Publisher('/start_and_goal', MarkerArray, queue_size=10, latch=True)
        # rospy.Subscriber("/local_plan_server/trajectory", SplineTrajectory, self.callback)
        self.num_trials = 10
        self.success = np.zeros(self.num_trials, dtype=bool)
        self.traj_time = np.zeros(self.num_trials)
        self.traj_cost = np.zeros(self.num_trials)
        self.traj_jerk = np.zeros(self.num_trials)
        self.traj_compute_time = np.zeros(self.num_trials)
        self.compute_time_front = np.zeros(self.num_trials)
        self.compute_time_back = np.zeros(self.num_trials)
        self.rho = 50  # TODO(Laura) pull from param or somewhere

        self.publisher()

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

    def publisher(self):
        print("waiting for map server")
        rospy.wait_for_service('/image_to_map')
        map_client = rospy.ServiceProxy('/image_to_map', Empty)
        map_client()

        print("waiting for action server")
        self.client.wait_for_server()

        # for i in range(self.start_goals.shape[0]):
        #     if rospy.is_shutdown():
        #         break

        #     print(i)
        #     msg = PlanTwoPointGoal()
        #     msg.p_init.position.x = self.start_goals['xi'][i]
        #     msg.p_init.position.y = self.start_goals['yi'][i]
        #     msg.p_init.position.z = 5
        #     # msg.v_init.linear.x = 2
        #     # msg.v_init.linear.y = 2
        #     msg.p_final.position.x = self.start_goals['xf'][i]
        #     msg.p_final.position.y = self.start_goals['yf'][i]
        #     msg.p_final.position.z = 5

        for i in range(self.num_trials):
            print(i)
            if rospy.is_shutdown():
                break
            if (i > 0):
                map_client()
                #TODO(Laura): actually send map ?
                rospy.sleep(2)
            msg = PlanTwoPointGoal()
            msg.p_init.position.x = 1.25
            msg.p_init.position.y = 1.25
            msg.p_init.position.z = 5
            # msg.v_init.linear.x = 2
            # msg.v_init.linear.y = 2
            msg.p_final.position.x = 19.5
            msg.p_final.position.y = 10 - 1.25
            msg.p_final.position.z = 5



            # do you want velocity initial and final to be zero?

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
            self.start_and_goal_pub.publish(start_and_goal)
            self.client.send_goal(msg)
            # self.client2.send_goal(msg)
            # self.client3.send_goal(msg)
            # Waits for the server to finish performing the action.
            self.client.wait_for_result(rospy.Duration.from_sec(3.0))
            # self.client2.wait_for_result(rospy.Duration.from_sec(5.0))
            # self.client3.wait_for_result(rospy.Duration.from_sec(5.0))

            result = self.client.get_result()
            #TODO(Laura) check if the path is collision free and feasible
            if result:
                self.success[i] = result.success
                if 0 < result.computation_time < 1000:
                    self.traj_compute_time[i] = result.computation_time
                    self.compute_time_front[i] = result.compute_time_front_end
                    self.compute_time_back[i] = result.compute_time_back_end
                if result.success:
                    self.traj_time[i] = result.traj.data[0].t_total
                    self.traj_cost[i] = self.computeCost(result.traj, self.rho)
                    self.traj_jerk[i] = self.computeJerk(result.traj)

            else:
                print("Action server failure " + str(i))
            input("Press Enter to continue...")

        print(self.success)
        print("Traj Time", self.traj_time)
        print("Traj Cost",self.traj_cost)
        print("Jerk", self.traj_jerk)
        print("Compute Time", self.traj_compute_time)
        print("Compute Time Front", self.compute_time_front)
        print("Compute Time Back", self.compute_time_back)

        print("success rate: " + str(np.sum(self.success)/self.success.size))
        print("avg traj time: " + str(np.sum(self.traj_time[self.success]) / np.sum(self.success)))
        print("avg traj cost: " + str(np.sum(self.traj_cost[self.success]) / np.sum(self.success)))
        print("avg traj jerk: " + str(np.sum(self.traj_jerk[self.success]) / np.sum(self.success)))
        print("avg compute time: " + str(np.sum(self.traj_compute_time[self.success]) / np.sum(self.success)))
        print("avg compute time front: " + str(np.sum(self.compute_time_front[self.success]) / np.sum(self.success)))
        print("avg compute time back: " + str(np.sum(self.compute_time_back[self.success]) / np.sum(self.success)))


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
