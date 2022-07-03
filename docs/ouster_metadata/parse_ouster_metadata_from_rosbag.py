#!/usr/bin/env python
# modified based on http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
import rospy
from std_msgs.msg import String
import json

def callback(data):
    x = json.loads(data.data)
    with open('ouster_metadata_parsed.json', 'w') as f:
        # Dump metadata into json
        json.dump(x, f, ensure_ascii=False, indent=4)
    print("metadata saved")

def listener():
    rospy.init_node('parse_ouster_metadata', anonymous=True)
    rospy.Subscriber("/os_node/metadata", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
