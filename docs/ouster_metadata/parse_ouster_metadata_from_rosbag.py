#!/usr/bin/env python
# modified based on http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
import rospy
from std_msgs.msg import String
import json

def callback(data):
    x = json.loads(data.data)
    with open('ouster_metadata_parsed.json', 'w') as f:
        json.dump(x, f, ensure_ascii=False, indent=4)    
        #  json.dump(data.data, f)
    print("metadata saved")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/os_node/metadata", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
