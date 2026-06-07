#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline target maneuver for the measured real-field obstacle map."""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


WAYPOINTS = [
    (0.0, 0.00, 2.10, 1.20, 0.0),
    (2.0, 0.90, 2.10, 1.20, 0.0),
    (5.5, 2.55, 1.65, 1.25, math.radians(-25.0)),
    (9.5, 2.55, -0.35, 1.25, math.radians(-90.0)),
    (14.0, 1.10, -1.95, 1.20, math.radians(-140.0)),
    (17.5, -0.80, -1.95, 1.25, math.radians(180.0)),
    (21.5, -2.65, -1.10, 1.30, math.radians(155.0)),
    (26.0, -2.65, 1.25, 1.30, math.radians(90.0)),
    (29.0, -1.25, 1.65, 1.25, math.radians(20.0)),
    (32.5, 0.40, 2.10, 1.20, 0.0),
]


class RealFieldEvasion:
    def __init__(self):
        rospy.init_node("real_field_evasion", anonymous=True)

        self.publish_rate = float(rospy.get_param("~publish_rate", 100.0))
        self.hover_duration = float(rospy.get_param("~hover_duration", 2.0))
        self.timeout = float(rospy.get_param("~timeout", 20.0))
        self.start_tolerance = float(rospy.get_param("~start_tolerance", 0.6))
        self.max_cmd_speed = float(rospy.get_param("~max_cmd_speed", 0.65))
        self.max_cmd_acc = float(rospy.get_param("~max_cmd_acc", 1.2))
        self.traj_id = int(rospy.get_param("~trajectory_id", 20))
        self.wait_for_trigger = bool(rospy.get_param("~wait_for_trigger", False))
        self.trigger_topic = str(rospy.get_param("~trigger_topic", "/triger"))

        self.cmd_pub = rospy.Publisher("/target/position_cmd", PositionCommand, queue_size=10)
        self.target_pos = None
        self.trigger_received = False
        rospy.Subscriber("/target/odom", Odometry, self.odom_cb)
        if self.wait_for_trigger:
            rospy.Subscriber(self.trigger_topic, PoseStamped, self.trigger_cb)

        rospy.loginfo("[real_field_evasion] waiting for target odom...")
        t0 = rospy.Time.now()
        while self.target_pos is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > self.timeout:
                raise RuntimeError("target odom not ready")
            rospy.sleep(0.05)
        rospy.loginfo("[real_field_evasion] target odom ready")

        self.ref_t = np.array([p[0] for p in WAYPOINTS], dtype=float)
        self.ref_p = np.array([[p[1], p[2], p[3]] for p in WAYPOINTS], dtype=float)
        self.ref_yaw = np.unwrap(np.array([p[4] for p in WAYPOINTS], dtype=float))
        self.ref_v, self.ref_a = self.build_derivatives(self.ref_t, self.ref_p)
        self.ref_yaw_rate, _ = self.build_derivatives(self.ref_t, self.ref_yaw.reshape((-1, 1)))
        self.ref_yaw_rate = self.ref_yaw_rate[:, 0]

        start_ref = self.ref_p[0]
        dist0 = float(np.linalg.norm(self.target_pos - start_ref))
        if dist0 > self.start_tolerance:
            raise RuntimeError(
                "target initial position mismatch: dist=%.2f > tolerance=%.2f, "
                "odom=(%.2f, %.2f, %.2f), ref=(%.2f, %.2f, %.2f)"
                % (
                    dist0,
                    self.start_tolerance,
                    self.target_pos[0],
                    self.target_pos[1],
                    self.target_pos[2],
                    start_ref[0],
                    start_ref[1],
                    start_ref[2],
                )
            )

        self.total_duration = float(self.ref_t[-1])
        self.hover_point = self.ref_p[-1].copy()
        self.hover_yaw = float(self.ref_yaw[-1])

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z], dtype=float)

    def trigger_cb(self, _msg):
        self.trigger_received = True

    def wait_for_start_trigger(self):
        if not self.wait_for_trigger:
            return
        rospy.loginfo("[real_field_evasion] waiting for trigger on %s...", self.trigger_topic)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.trigger_received:
            rate.sleep()
        rospy.loginfo("[real_field_evasion] trigger received")

    @staticmethod
    def build_derivatives(ts, values):
        n = len(ts)
        vel = np.zeros_like(values)
        acc = np.zeros_like(values)
        for i in range(n):
            if i == 0:
                dt = max(1e-3, ts[i + 1] - ts[i])
                vel[i] = (values[i + 1] - values[i]) / dt
            elif i == n - 1:
                dt = max(1e-3, ts[i] - ts[i - 1])
                vel[i] = (values[i] - values[i - 1]) / dt
            else:
                dt = max(1e-3, ts[i + 1] - ts[i - 1])
                vel[i] = (values[i + 1] - values[i - 1]) / dt
        for i in range(n):
            if i == 0:
                dt = max(1e-3, ts[i + 1] - ts[i])
                acc[i] = (vel[i + 1] - vel[i]) / dt
            elif i == n - 1:
                dt = max(1e-3, ts[i] - ts[i - 1])
                acc[i] = (vel[i] - vel[i - 1]) / dt
            else:
                dt = max(1e-3, ts[i + 1] - ts[i - 1])
                acc[i] = (vel[i + 1] - vel[i - 1]) / dt
        return vel, acc

    @staticmethod
    def sample_linear(ts, values, t):
        if t <= ts[0]:
            return values[0].copy()
        if t >= ts[-1]:
            return values[-1].copy()
        idx = np.searchsorted(ts, t, side="right") - 1
        idx = max(0, min(idx, len(ts) - 2))
        t0 = ts[idx]
        t1 = ts[idx + 1]
        alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return (1.0 - alpha) * values[idx] + alpha * values[idx + 1]

    @staticmethod
    def clamp_vec(vec, max_norm):
        norm = float(np.linalg.norm(vec))
        if norm <= max_norm or norm < 1e-9:
            return vec
        return vec * (max_norm / norm)

    def build_cmd(self, p, v, a, yaw, yaw_rate):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = self.traj_id
        cmd.position.x = float(p[0])
        cmd.position.y = float(p[1])
        cmd.position.z = float(p[2])
        cmd.velocity.x = float(v[0])
        cmd.velocity.y = float(v[1])
        cmd.velocity.z = float(v[2])
        cmd.acceleration.x = float(a[0])
        cmd.acceleration.y = float(a[1])
        cmd.acceleration.z = float(a[2])
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        cmd.yaw = float(math.atan2(math.sin(yaw), math.cos(yaw)))
        cmd.yaw_dot = float(yaw_rate)
        cmd.kx[0] = 5.7
        cmd.kx[1] = 5.7
        cmd.kx[2] = 6.2
        cmd.kv[0] = 3.4
        cmd.kv[1] = 3.4
        cmd.kv[2] = 4.0
        return cmd

    def publish_hover(self, rate):
        cycles = max(1, int(round(self.hover_duration * self.publish_rate)))
        for _ in range(cycles):
            if rospy.is_shutdown():
                return
            self.cmd_pub.publish(
                self.build_cmd(self.hover_point, np.zeros(3), np.zeros(3), self.hover_yaw, 0.0)
            )
            rate.sleep()

    def run(self):
        rate = rospy.Rate(max(1.0, self.publish_rate))
        rospy.sleep(0.2)
        self.wait_for_start_trigger()
        start_wall = rospy.Time.now()
        rospy.loginfo(
            "[real_field_evasion] start replay, duration=%.2fs, waypoints=%d",
            self.total_duration,
            len(WAYPOINTS),
        )
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_wall).to_sec()
            if t >= self.total_duration:
                break
            p = self.sample_linear(self.ref_t, self.ref_p, t)
            v = self.clamp_vec(self.sample_linear(self.ref_t, self.ref_v, t), self.max_cmd_speed)
            a = self.clamp_vec(self.sample_linear(self.ref_t, self.ref_a, t), self.max_cmd_acc)
            yaw = float(self.sample_linear(self.ref_t, self.ref_yaw, t))
            yaw_rate = float(self.sample_linear(self.ref_t, self.ref_yaw_rate, t))
            self.cmd_pub.publish(self.build_cmd(p, v, a, yaw, yaw_rate))
            rate.sleep()
        rospy.loginfo("[real_field_evasion] replay finished, entering hover")
        self.publish_hover(rate)
        rospy.loginfo("[real_field_evasion] done")


if __name__ == "__main__":
    try:
        RealFieldEvasion().run()
    except rospy.ROSInterruptException:
        pass
