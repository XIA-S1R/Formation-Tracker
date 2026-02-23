#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

rospy.init_node('trajectory_publisher')
pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=1)
rate = rospy.Rate(10)  # 10 Hz

for i in range(0, 361, 10):
    x = math.cos(math.radians(i))
    y = math.sin(math.radians(i))
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 1.0
    pose.pose.orientation.w = 1.0
    pub.publish(pose)
    rate.sleep()