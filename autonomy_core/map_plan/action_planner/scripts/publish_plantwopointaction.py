#!/usr/bin/env python3
import rospy
from kr_planning_msgs.msg import PlanTwoPointActionGoal
from visualization_msgs.msg import MarkerArray, Marker
from copy import deepcopy
from random import randrange, seed, uniform
import pandas as pd
filename = '/home/yifei/ws/src/kr_autonomous_flight/autonomy_core/map_plan/action_planner/scripts/map_balls_start_goal.csv'
def publisher():
    print("reading "+filename)
    start_goal = pd.read_csv(filename)
    path_pub = rospy.Publisher('/local_plan_server/plan_local_trajectory/goal', PlanTwoPointActionGoal, queue_size=10)
    rospy.init_node('publish_two_point_action')
    start_and_goal_pub = rospy.Publisher('/start_and_goal', MarkerArray, queue_size=10)
    rospy.init_node('publish_two_point_action')

    rate = rospy.Rate(0.5)  #
    for i in range(start_goal.shape[0]):
        if rospy.is_shutdown():
            break
        print(i)
        msg = PlanTwoPointActionGoal()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.goal.p_init.position.x = start_goal['xi'][i]
        msg.goal.p_init.position.y = start_goal['yi'][i]
        msg.goal.p_init.position.z = 1
        msg.goal.p_final.position.x = start_goal['xf'][i]
        msg.goal.p_final.position.y = start_goal['yf'][i]
        msg.goal.p_final.position.z = 1
        
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

if __name__ == '__main__':
    try:
        seed(237)
        publisher()
    except rospy.ROSInterruptException:
        pass
