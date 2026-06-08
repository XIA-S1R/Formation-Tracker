#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("delayed_trigger", anonymous=True)
    topic = rospy.get_param("~topic", "/triger")
    delay = float(rospy.get_param("~delay", 4.0))
    publish_hz = float(rospy.get_param("~publish_hz", 10.0))
    duration = float(rospy.get_param("~duration", 2.0))

    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    rospy.loginfo("[delayed_trigger] waiting %.2fs before publishing %s", delay, topic)
    rospy.sleep(max(0.0, delay))

    rate = rospy.Rate(max(1.0, publish_hz))
    end_t = rospy.Time.now() + rospy.Duration(max(0.1, duration))
    while not rospy.is_shutdown() and rospy.Time.now() < end_t:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo("[delayed_trigger] published %s", topic)


if __name__ == "__main__":
    main()
