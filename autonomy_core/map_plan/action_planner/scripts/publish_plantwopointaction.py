#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kr_planning_msgs.action import PlanTwoPoint
from kr_planning_msgs.msg import VoxelMap
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
    node = rclpy.create_node('publish_two_point_action')
    # Action client replaces publishing to the old actionlib goal topic
    action_client = ActionClient(node, PlanTwoPoint, '/local_plan_server/plan_local_trajectory')
    #subscribe to voxel map
    # voxel_map = node.create_subscription(VoxelMap, '/mapper/local_voxel_map', cb, 1)
    start_and_goal_pub = node.create_publisher(MarkerArray, '/start_and_goal', 10)

    node.declare_parameter("/mapper/global/origin_x", 0.0)
    node.declare_parameter("/mapper/global/origin_y", 0.0)
    node.declare_parameter("/mapper/global/origin_z", 0.0)
    node.declare_parameter("/mapper/global/range_x", 0.0)
    node.declare_parameter("/mapper/global/range_y", 0.0)
    node.declare_parameter("/mapper/global/range_z", 0.0)

    map_origin_x = node.get_parameter("/mapper/global/origin_x").value
    map_origin_y = node.get_parameter("/mapper/global/origin_y").value
    map_origin_z = node.get_parameter("/mapper/global/origin_z").value

    map_range_x = node.get_parameter("/mapper/global/range_x").value
    map_range_y = node.get_parameter("/mapper/global/range_y").value
    map_range_z = node.get_parameter("/mapper/global/range_z").value


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


    # TODO: Make this run everytime we receive a new map, not finished yet
    sleep_period = 1.0
    i = 0
    while True:
    # for i in range(start_goal.shape[0]):
        if not rclpy.ok():
            break
        print(i)
        goal_msg = PlanTwoPoint.Goal()
        stamp_header_frame_id = "map"
        stamp_header_stamp = node.get_clock().now().to_msg()


        start, end = sample_in_map()

        goal_msg.p_init.position.x = start[0]
        goal_msg.p_init.position.y = start[1]
        goal_msg.p_init.position.z = start[2]
        #random initial velocity
        goal_msg.v_init.linear.x = uniform(0,1)
        goal_msg.v_init.linear.y = uniform(-1,1)
        goal_msg.v_init.linear.z = uniform(-1,1)
        goal_msg.a_init.linear.x = 0.0
        goal_msg.a_init.linear.y = 0.0
        goal_msg.a_init.linear.z = 0.0
        print("initial position")
        print(goal_msg.p_init.position)
        print("initial velocity")
        print(goal_msg.v_init.linear)

        goal_msg.p_final.position.x = end[0]
        goal_msg.p_final.position.y = end[1]
        goal_msg.p_final.position.z = end[2]

        #do you want velocity initial and final to be zero?

        start_and_goal = MarkerArray()
        start = Marker()
        start.header.frame_id = stamp_header_frame_id
        start.header.stamp = stamp_header_stamp
        start.pose.position = goal_msg.p_init.position
        start.pose.orientation.w = 1.0
        start.color.g = 1.0
        start.color.a = 1.0
        start.type = 2
        start.scale.x = start.scale.y = start.scale.z = 1.0
        goal = deepcopy(start)
        goal.pose.position = goal_msg.p_final.position
        goal.id = 1
        goal.color.r = 1.0
        goal.color.g = 0.0
        start_and_goal.markers.append(start)
        start_and_goal.markers.append(goal)
        # Send goal via the action client (was path_pub.publish(msg) previously)
        if action_client.wait_for_server(timeout_sec=1.0):
            action_client.send_goal_async(goal_msg)
        else:
            node.get_logger().warn("Action server /local_plan_server/plan_local_trajectory not available")
        start_and_goal_pub.publish(start_and_goal)
        time.sleep(sleep_period)
        i += 1

if __name__ == '__main__':
    rclpy.init()
    try:
        seed(237)
        semi_main()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
