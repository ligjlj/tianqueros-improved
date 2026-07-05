#!/usr/bin/env python3
"""Fake odometry publisher — simulates a robot at origin for testing."""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3

def main():
    rospy.init_node('fake_odom')
    pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    rate = rospy.Rate(10)
    
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    msg.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    rospy.loginfo("Fake odometry publishing at origin.")
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
