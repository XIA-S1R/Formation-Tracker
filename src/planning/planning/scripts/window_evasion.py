#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
穿窗机动脚本：让目标无人机在三面薄墙之间来回穿梭窗户，触发搜索模式。

地图约定 (来自 src/uav_simulator/mockamap/src/maps.cpp):
  - 3 面薄墙, x 位置分别为 9.0 / 13.0 / 17.0, 墙间距 4m
  - 薄墙 y 方向交错: 偶数号墙 wy=0.0, 奇数号墙 wy=3.0
  - 每面墙 2 个窗口, 沿 y 方向间距 3m, 中心对称于 wy
  - 窗口 z 中心 = win_z_base + win_height/2 = 1.5 + 0.75 = 2.25m

窗口中心 (x, y, z):
  墙0 (x=9.0,  wy=0.0): y = -1.5, +1.5
  墙1 (x=13.0, wy=3.0): y = +1.5, +4.5
  墙2 (x=17.0, wy=0.0): y = -1.5, +1.5

航线设计 (两段, 穿完所有 6 个窗口再回头):
  - 正向蛇形: 从 y=-1.5 切入, 依次穿 墙0(下窗) → 墙1(上窗) → 墙2(下窗);
    三面墙的三个不同 y 都要穿到, 每次过墙前在 y 方向做大幅机动以触发失锁.
  - 反向直线: 从 y=+1.5 切入, 依次穿 墙2(上窗) → 墙1(下窗) → 墙0(上窗);
    这条线上 三面墙 都在 y=+1.5 有窗, 几乎直线, 速度更快.

用法: rosrun planning window_evasion.py
参数: _arrive_dist:=2.0  _timeout:=30.0  _loop:=true
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 窗口 z 中心
WIN_Z = 2.25

# 每面墙的窗口中心 y
W0_BOT, W0_TOP = -1.5, 1.5   # 墙0 (x=9.0)
W1_BOT, W1_TOP =  1.5, 4.5   # 墙1 (x=13.0)
W2_BOT, W2_TOP = -1.5, 1.5   # 墙2 (x=17.0)

# 墙 x 位置
X_W0, X_W1, X_W2 = 9.0, 13.0, 17.0

# 每面墙前后用于定位的 x: 墙前 1.5m 摆拍, 墙后 1.5m 出窗
D = 1.5
X_BEFORE_W0 = X_W0 - D   # 7.5
X_BETWEEN_01_A = X_W0 + D   # 10.5  (刚出墙0)
X_BETWEEN_01_B = X_W1 - D   # 11.5  (要进墙1)
X_BETWEEN_12_A = X_W1 + D   # 14.5  (刚出墙1)
X_BETWEEN_12_B = X_W2 - D   # 15.5  (要进墙2)
X_AFTER_W2 = X_W2 + D    # 18.5

# 出入场 x (方便循环)
X_ENTRY = -5.0
X_RETURN = 21.0

WAYPOINTS = [
    # ======================================================
    # 第一段: 正向蛇形, 每墙走不同 y 的窗口, 一墙一点
    # 相邻航点间距 >=5m, 让目标能加速穿越
    # ======================================================
    (X_ENTRY,           W0_BOT),   # 入场对正墙 0 下窗 y=-1.5
    (X_BETWEEN_01_A,    W0_BOT),   # 穿墙 0 下窗 (x=10.5, y=-1.5)
    (X_BETWEEN_12_A,    W1_TOP),   # 侧移到 y=+4.5 并穿墙 1 上窗 (x=14.5)
    (X_AFTER_W2,        W2_BOT),   # 侧移到 y=-1.5 并穿墙 2 下窗 (x=18.5)
    (X_RETURN,          W2_BOT),   # 收场

    # ======================================================
    # 第二段: 反向直线, 走 y=+1.5 贯穿三墙
    # ======================================================
    (X_RETURN,          W2_TOP),   # 回头对正 y=+1.5
    (X_BETWEEN_12_B,    W2_TOP),   # 穿墙 2 上窗 (x=15.5)
    (X_BETWEEN_01_B,    W1_BOT),   # 穿墙 1 下窗 (x=11.5)
    (X_BEFORE_W0,       W0_TOP),   # 穿墙 0 上窗 (x=7.5)
    (X_ENTRY,           W0_TOP),   # 出场
]


class WindowEvasion:
    def __init__(self):
        rospy.init_node('window_evasion', anonymous=True)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=1)
        self.target_pos = None
        rospy.Subscriber('/target/odom', Odometry, self.odom_cb)

        # arrive_dist 要大于 planning 的 hover 判定距离 (tracking_dist+tolerance_d≈3.5m)，
        # 否则目标在墙前减速悬停、迟迟拿不到下一个航点。
        self.arrive_dist = rospy.get_param('~arrive_dist', 5.0)
        self.timeout = rospy.get_param('~timeout', 30.0)
        self.loop = rospy.get_param('~loop', False)
        # 穿窗时对齐精度要求更高，用更小的到达判定半径
        self.window_arrive_dist = rospy.get_param('~window_arrive_dist', 3.0)

        rospy.loginfo("[window_evasion] Waiting for target odom...")
        while self.target_pos is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        rospy.loginfo("[window_evasion] Got target odom, starting maneuver.")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])

    def publish_goal(self, x, y, z=WIN_Z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("[window_evasion] Goal: (%.1f, %.1f, %.1f)", x, y, z)

    def wait_arrive(self, x, y, arrive_dist):
        goal = np.array([x, y])
        t0 = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.target_pos is not None:
                dist = np.linalg.norm(self.target_pos[:2] - goal)
                if dist < arrive_dist:
                    rospy.loginfo(
                        "[window_evasion] Arrived (%.1f,%.1f) d=%.2f",
                        x, y, dist)
                    return True
            if (rospy.Time.now() - t0).to_sec() > self.timeout:
                rospy.logwarn(
                    "[window_evasion] Timeout at (%.1f, %.1f)", x, y)
                return False
            rate.sleep()
        return False

    @staticmethod
    def is_window_waypoint(x):
        # 墙前后 1.5m 的航点视为"对准窗户", 使用更小的到达半径
        return any(abs(x - xw) < (D + 0.3) for xw in (X_W0, X_W1, X_W2))

    def run(self):
        rospy.sleep(1.0)
        lap = 0
        while not rospy.is_shutdown():
            lap += 1
            rospy.loginfo("[window_evasion] === Lap %d ===", lap)
            for i, (wx, wy) in enumerate(WAYPOINTS):
                if rospy.is_shutdown():
                    break
                rospy.loginfo(
                    "[window_evasion] WP %d/%d", i + 1, len(WAYPOINTS))
                self.publish_goal(wx, wy)
                arrive_dist = (self.window_arrive_dist
                               if self.is_window_waypoint(wx)
                               else self.arrive_dist)
                self.wait_arrive(wx, wy, arrive_dist)
            if not self.loop:
                break
        rospy.loginfo("[window_evasion] Done")


if __name__ == '__main__':
    try:
        node = WindowEvasion()
        node.run()
    except rospy.ROSInterruptException:
        pass
