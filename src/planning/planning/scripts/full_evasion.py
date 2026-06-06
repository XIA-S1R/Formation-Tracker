#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Target 离线参考轨迹执行脚本。

直接向 /target/position_cmd 发布固定参考轨迹，不再经过 target 在线规划链路
或 traj_server。第一版使用一条从历史成功实验中提取的 top 机动轨迹骨架，
通过时间参数化插值在线采样为 PositionCommand。
"""

import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


# 从 experiment_results/本方法/run_011/tracking.bag 中按 1Hz 提取的 target 参考轨迹。
# 该 bag 对应当前固定 mockmap(seed=510) 下的一条已成功 target 机动。
REFERENCE_SAMPLES = [
    (0.00, -16.000, 0.000, 2.000),
    (1.00, -15.792, -0.039, 1.999),
    (1.98, -13.878, -0.400, 2.061),
    (2.99, -11.498, -0.893, 2.197),
    (4.00, -9.100, -1.202, 2.343),
    (4.98, -6.654, -1.533, 2.408),
    (6.01, -4.435, -2.232, 2.642),
    (6.99, -3.446, -4.267, 2.900),
    (8.01, -2.082, -5.873, 2.948),
    (9.01, 0.244, -5.896, 2.934),
    (10.00, 2.495, -5.641, 2.932),
    (11.01, 4.724, -5.523, 2.946),
    (12.01, 7.124, -5.816, 2.985),
    (13.00, 9.152, -5.765, 2.909),
    (14.01, 10.537, -3.976, 2.880),
    (15.00, 10.893, -1.684, 2.855),
    (16.02, 10.924, 0.415, 2.866),
    (16.98, 10.949, 2.646, 2.904),
    (18.02, 10.982, 4.924, 2.927),
    (19.02, 11.018, 7.033, 2.913),
    (19.99, 11.749, 8.113, 2.907),
    (21.00, 12.818, 8.980, 2.916),
    (21.99, 11.646, 7.907, 2.882),
    (23.02, 10.652, 5.691, 2.867),
    (23.99, 10.388, 3.476, 2.837),
    (24.99, 10.334, 1.188, 2.847),
    (25.99, 10.182, -0.977, 2.898),
    (26.99, 10.265, -3.203, 2.911),
    (27.98, 12.127, -4.626, 2.930),
    (29.00, 14.365, -5.370, 2.910),
    (30.00, 16.622, -5.799, 2.915),
    (31.01, 18.212, -4.613, 2.966),
    (32.01, 18.556, -2.382, 2.992),
    (33.00, 18.495, -0.125, 3.003),
    (34.01, 18.468, 2.253, 2.951),
    (34.99, 18.539, 3.646, 2.927),
    (36.00, 18.546, 1.739, 2.875),
    (36.99, 18.607, -0.513, 2.853),
    (37.99, 18.499, -2.824, 2.851),
    (39.02, 17.681, -5.041, 2.874),
    (39.99, 16.148, -6.717, 2.932),
    (41.00, 14.318, -8.169, 2.907),
    (42.02, 12.344, -9.551, 2.939),
    (42.99, 10.358, -10.674, 2.965),
    (43.98, 8.152, -11.574, 2.935),
    (44.99, 5.840, -12.204, 2.890),
    (45.99, 3.539, -12.721, 2.873),
    (47.00, 1.283, -13.497, 2.899),
    (48.02, -0.708, -14.872, 2.924),
    (48.98, -2.253, -16.436, 2.912),
    (49.99, -2.802, -15.568, 2.909),
    (51.00, -2.797, -13.178, 2.945),
    (51.98, -3.110, -10.944, 2.954),
    (53.00, -3.550, -8.554, 3.024),
    (54.00, -3.625, -6.280, 3.242),
    (55.00, -3.524, -3.855, 3.305),
    (55.98, -3.428, -1.468, 3.249),
    (57.00, -3.235, 0.781, 3.154),
    (57.98, -3.014, 3.038, 3.065),
    (59.00, -2.979, 5.437, 2.955),
    (59.99, -3.340, 7.700, 2.928),
    (60.99, -3.870, 10.054, 3.001),
    (62.00, -4.278, 12.409, 3.064),
    (63.01, -4.519, 14.750, 3.067),
]


class FullEvasionDirectCmd:
    def __init__(self):
        rospy.init_node('full_evasion', anonymous=True)

        self.publish_rate = float(rospy.get_param('~publish_rate', 100.0))
        self.hover_duration = float(rospy.get_param('~hover_duration', 2.0))
        self.timeout = float(rospy.get_param('~timeout', 20.0))
        self.start_tolerance = float(rospy.get_param('~start_tolerance', 1.0))
        self.max_cmd_speed = float(rospy.get_param('~max_cmd_speed', 2.5))
        self.max_cmd_acc = float(rospy.get_param('~max_cmd_acc', 5.0))
        self.traj_id = int(rospy.get_param('~trajectory_id', 1))
        self.yaw = float(rospy.get_param('~yaw', 0.0))
        self.hover_x = rospy.get_param('~hover_x', None)
        self.hover_y = rospy.get_param('~hover_y', None)
        self.hover_z = rospy.get_param('~hover_z', None)

        self.cmd_pub = rospy.Publisher('/target/position_cmd', PositionCommand, queue_size=10)
        self.target_pos = None

        rospy.Subscriber('/target/odom', Odometry, self.odom_cb)

        rospy.loginfo('[full_evasion] waiting for target odom...')
        t0 = rospy.Time.now()
        while self.target_pos is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > self.timeout:
                raise RuntimeError('target odom not ready')
            rospy.sleep(0.05)
        rospy.loginfo('[full_evasion] target odom ready')

        self.ref_t = np.array([p[0] for p in REFERENCE_SAMPLES], dtype=float)
        self.ref_p = np.array([[p[1], p[2], p[3]] for p in REFERENCE_SAMPLES], dtype=float)
        self.ref_v, self.ref_a = self.build_derivatives(self.ref_t, self.ref_p)

        start_ref = self.ref_p[0]
        dist0 = float(np.linalg.norm(self.target_pos - start_ref))
        if dist0 > self.start_tolerance:
            raise RuntimeError(
                'target initial position mismatch: dist=%.2f > tolerance=%.2f, odom=(%.2f, %.2f, %.2f), ref=(%.2f, %.2f, %.2f)'
                % (dist0,
                   self.start_tolerance,
                   self.target_pos[0], self.target_pos[1], self.target_pos[2],
                   start_ref[0], start_ref[1], start_ref[2]))

        self.total_duration = float(self.ref_t[-1])
        self.hover_point = np.array([
            self.ref_p[-1, 0] if self.hover_x is None else float(self.hover_x),
            self.ref_p[-1, 1] if self.hover_y is None else float(self.hover_y),
            self.ref_p[-1, 2] if self.hover_z is None else float(self.hover_z),
        ], dtype=float)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z], dtype=float)

    @staticmethod
    def build_derivatives(ts, ps):
        n = len(ts)
        vel = np.zeros_like(ps)
        acc = np.zeros_like(ps)
        for i in range(n):
            if i == 0:
                dt = max(1e-3, ts[i + 1] - ts[i])
                vel[i] = (ps[i + 1] - ps[i]) / dt
            elif i == n - 1:
                dt = max(1e-3, ts[i] - ts[i - 1])
                vel[i] = (ps[i] - ps[i - 1]) / dt
            else:
                dt = max(1e-3, ts[i + 1] - ts[i - 1])
                vel[i] = (ps[i + 1] - ps[i - 1]) / dt
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

    def sample_linear(self, ts, values, t):
        if t <= ts[0]:
            return values[0].copy()
        if t >= ts[-1]:
            return values[-1].copy()
        idx = np.searchsorted(ts, t, side='right') - 1
        idx = max(0, min(idx, len(ts) - 2))
        t0 = ts[idx]
        t1 = ts[idx + 1]
        alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return (1.0 - alpha) * values[idx] + alpha * values[idx + 1]

    def clamp_vec(self, vec, max_norm):
        norm = float(np.linalg.norm(vec))
        if norm <= max_norm or norm < 1e-9:
            return vec
        return vec * (max_norm / norm)

    def build_cmd(self, p, v, a):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'
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
        cmd.yaw = self.yaw
        cmd.yaw_dot = 0.0
        cmd.kx[0] = 5.7
        cmd.kx[1] = 5.7
        cmd.kx[2] = 6.2
        cmd.kv[0] = 3.4
        cmd.kv[1] = 3.4
        cmd.kv[2] = 4.0
        return cmd

    def publish_hover(self, rate):
        hover_cycles = max(1, int(round(self.hover_duration * self.publish_rate)))
        for _ in range(hover_cycles):
            if rospy.is_shutdown():
                return
            cmd = self.build_cmd(self.hover_point, np.zeros(3), np.zeros(3))
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def run(self):
        rate = rospy.Rate(max(1.0, self.publish_rate))
        rospy.sleep(0.2)
        start_wall = rospy.Time.now()
        rospy.loginfo('[full_evasion] start direct position_cmd replay, duration=%.2fs, samples=%d',
                      self.total_duration, len(self.ref_t))

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_wall).to_sec()
            if t >= self.total_duration:
                break
            p = self.sample_linear(self.ref_t, self.ref_p, t)
            v = self.clamp_vec(self.sample_linear(self.ref_t, self.ref_v, t), self.max_cmd_speed)
            a = self.clamp_vec(self.sample_linear(self.ref_t, self.ref_a, t), self.max_cmd_acc)
            cmd = self.build_cmd(p, v, a)
            self.cmd_pub.publish(cmd)
            rate.sleep()

        rospy.loginfo('[full_evasion] replay finished, entering hover for %.2fs', self.hover_duration)
        self.publish_hover(rate)
        rospy.loginfo('[full_evasion] done')


if __name__ == '__main__':
    try:
        FullEvasionDirectCmd().run()
    except rospy.ROSInterruptException:
        pass
