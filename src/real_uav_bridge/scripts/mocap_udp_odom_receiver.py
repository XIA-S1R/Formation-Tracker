#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive mocap UDP JSON packets and publish ROS Odometry.

Expected packet fields match mocap_udp_relay.py:
  name, timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz
"""

import json
import socket
import threading

import rospy
from nav_msgs.msg import Odometry


def _get_required_float(data, key):
    if key not in data:
        raise KeyError(key)
    return float(data[key])


class MocapUdpOdomReceiver:
    def __init__(self):
        rospy.init_node("mocap_udp_odom_receiver", anonymous=False)

        self.listen_ip = rospy.get_param("~listen_ip", "0.0.0.0")
        self.listen_port = int(rospy.get_param("~listen_port", 5005))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.use_packet_name_as_child_frame = bool(
            rospy.get_param("~use_packet_name_as_child_frame", False)
        )
        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.tracker_name = rospy.get_param("~tracker_name", "")
        self.name_to_topic = rospy.get_param("~name_to_topic", {})
        self.publish_stale_warn_sec = float(rospy.get_param("~stale_warn_sec", 1.0))
        self.recv_timeout = float(rospy.get_param("~recv_timeout", 0.2))

        self.publishers = {}
        self.default_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=30)
        self.last_packet_time = rospy.Time(0)
        self.lock = threading.Lock()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(self.recv_timeout)

        rospy.Timer(rospy.Duration(0.5), self.warn_if_stale)
        rospy.loginfo(
            "[mocap_udp_odom_receiver] listening on %s:%d, default topic=%s",
            self.listen_ip,
            self.listen_port,
            self.odom_topic,
        )

    def get_publisher(self, name):
        if not self.name_to_topic:
            return self.default_pub
        topic = self.name_to_topic.get(name)
        if not topic:
            rospy.logwarn_throttle(
                2.0,
                "[mocap_udp_odom_receiver] drop packet: unmapped tracker name '%s'",
                name,
            )
            return None
        pub = self.publishers.get(topic)
        if pub is None:
            pub = rospy.Publisher(topic, Odometry, queue_size=30)
            self.publishers[topic] = pub
            rospy.loginfo("[mocap_udp_odom_receiver] mapped %s -> %s", name, topic)
        return pub

    def packet_to_odom(self, data):
        name = str(data.get("name", ""))
        if self.tracker_name and name and name != self.tracker_name:
            return None, None

        msg = Odometry()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = (
            name if self.use_packet_name_as_child_frame and name else self.child_frame_id
        )
        msg.header.stamp = rospy.Time.now()

        msg.pose.pose.position.x = _get_required_float(data, "x")
        msg.pose.pose.position.y = _get_required_float(data, "y")
        msg.pose.pose.position.z = _get_required_float(data, "z")
        msg.pose.pose.orientation.x = float(data.get("qx", 0.0))
        msg.pose.pose.orientation.y = float(data.get("qy", 0.0))
        msg.pose.pose.orientation.z = float(data.get("qz", 0.0))
        msg.pose.pose.orientation.w = float(data.get("qw", 1.0))

        msg.twist.twist.linear.x = float(data.get("vx", 0.0))
        msg.twist.twist.linear.y = float(data.get("vy", 0.0))
        msg.twist.twist.linear.z = float(data.get("vz", 0.0))

        return name, msg

    def handle_payload(self, payload, addr):
        try:
            data = json.loads(payload.decode("utf-8"))
            name, msg = self.packet_to_odom(data)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0,
                "[mocap_udp_odom_receiver] invalid packet from %s:%s: %s",
                addr[0],
                addr[1],
                exc,
            )
            return
        if msg is None:
            return

        pub = self.get_publisher(name)
        if pub is None:
            return
        pub.publish(msg)
        with self.lock:
            self.last_packet_time = rospy.Time.now()

    def warn_if_stale(self, _event):
        with self.lock:
            last = self.last_packet_time
        if last.is_zero():
            return
        age = (rospy.Time.now() - last).to_sec()
        if age > self.publish_stale_warn_sec:
            rospy.logwarn_throttle(
                1.0,
                "[mocap_udp_odom_receiver] no mocap UDP packet for %.2fs",
                age,
            )

    def spin(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not rospy.is_shutdown():
                    rospy.logerr("[mocap_udp_odom_receiver] socket error: %s", exc)
                break
            self.handle_payload(payload, addr)


if __name__ == "__main__":
    try:
        MocapUdpOdomReceiver().spin()
    except rospy.ROSInterruptException:
        pass
