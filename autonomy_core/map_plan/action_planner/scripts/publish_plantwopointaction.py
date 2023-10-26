#!/usr/bin/env python3
import rospy
from kr_planning_msgs.msg import PlanTwoPointActionGoal, VoxelMap
from visualization_msgs.msg import MarkerArray, Marker
from copy import deepcopy
from random import randrange, seed, uniform
import pandas as pd
import numpy as np
#filename = '/home/wyw/Code/yifei_ws/src/kr_autonomous_flight/autonomy_core/map_plan/action_planner/scripts/map_balls_start_goal.csv'
filename = "src/kr_autonomous_flight/autonomy_core/map_plan/action_planner/scripts/map_balls_start_goal.csv"



def semi_main():
    print("reading "+filename)
    start_goal = pd.read_csv(filename)
    path_pub = rospy.Publisher('/local_plan_server/plan_local_trajectory/goal', PlanTwoPointActionGoal, queue_size=10)
    #subscribe to voxel map
    # voxel_map = rospy.Subscriber('/mapper/local_voxel_map', VoxelMap, queue_size=1)
    rospy.init_node('publish_two_point_action')
    start_and_goal_pub = rospy.Publisher('/start_and_goal', MarkerArray, queue_size=10)
    rospy.init_node('publish_two_point_action')

    map_origin_x = rospy.get_param("/mapper/global/origin_x")
    map_origin_y = rospy.get_param("/mapper/global/origin_y")
    map_origin_z = rospy.get_param("/mapper/global/origin_z")

    map_range_x = rospy.get_param("/mapper/global/range_x")
    map_range_y = rospy.get_param("/mapper/global/range_y")
    map_range_z = rospy.get_param("/mapper/global/range_z")
    

    start_goal_dist_min = 0.8 * map_range_x

    def sample_in_map():

        curr_sample_idx = 0

        while curr_sample_idx < 100:
        
            rand_start_x = uniform(map_origin_x, map_range_x + map_origin_x)
            rand_start_y = uniform(map_origin_y, map_range_y + map_origin_y)
            rand_start_z = uniform(map_origin_z, map_range_z + map_origin_z)

            rand_goal_x = uniform(map_origin_x, map_range_x + map_origin_x)
            rand_goal_y = uniform(map_origin_y, map_range_y + map_origin_y)
            rand_goal_z = uniform(map_origin_z + 0.2, map_range_z + map_origin_z - 0.2)

            start = np.array([rand_start_x, rand_start_y, rand_start_z])
            goal = np.array([rand_goal_x, rand_goal_y, rand_goal_z])
            

            curr_sample_idx += 1
            dis = np.linalg.norm(start - goal)
            if dis < start_goal_dist_min or dis > 0.5 * (map_range_x + map_range_y):
                #print("start and goal are too close! Skipping...")
                continue

        return start, goal


    rate = rospy.Rate(1)  #TODO: Make this run everytime we receive a new map, not finished yet
    i = 0
    while True:
    # for i in range(start_goal.shape[0]):
        if rospy.is_shutdown():
            break
        print(i)
        msg = PlanTwoPointActionGoal()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()


        start, end = sample_in_map()

        msg.goal.p_init.position.x = start[0]
        msg.goal.p_init.position.y = start[1]
        msg.goal.p_init.position.z = start[2]
        #random initial velocity
        msg.goal.v_init.linear.x = uniform(0,1)
        msg.goal.v_init.linear.y = uniform(-1,1)
        msg.goal.v_init.linear.z = uniform(-1,1)
        msg.goal.a_init.linear.x = 0.0
        msg.goal.a_init.linear.y = 0.0
        msg.goal.a_init.linear.z = 0.0
        print("initial position")
        print(msg.goal.p_init.position)
        print("initial velocity")
        print(msg.goal.v_init.linear)

        msg.goal.p_final.position.x = end[0]
        msg.goal.p_final.position.y = end[1]
        msg.goal.p_final.position.z = end[2]
        
        #do you want velocity initial and final to be zero?

        start_and_goal = MarkerArray()
        start = Marker()
        start.header = msg.header
        start.pose.position = msg.goal.p_init.position
        start.pose.orientation.w = 1
        start.color.g = 1
        start.color.a = 1
        start.type = 2
        start.scale.x = start.scale.y = start.scale.z = 1
        goal = deepcopy(start)
        goal.pose.position = msg.goal.p_final.position
        goal.id = 1
        goal.color.r = 1
        goal.color.g = 0
        start_and_goal.markers.append(start)
        start_and_goal.markers.append(goal)
        path_pub.publish(msg)
        start_and_goal_pub.publish(start_and_goal)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        seed(237)
        semi_main()
    except rospy.ROSInterruptException:
        pass
