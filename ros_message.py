#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("ros_tag_message")
    pub = rospy.Publisher("tag_message", String, queue_size=10)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "A"
        pub.publish(msg)
        rospy.loginfo("Publish : " + msg.data)
        msg.data = "B"
        pub.publish(msg)
        rospy.loginfo("Publish : " + msg.data)
        r.sleep()

if __name__ == "__main__":
    main()
