#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
自动摆脱机动脚本：让目标无人机绕薄墙边缘走回形(蛇形)路线，触发搜索模式。
地图42x40m, 薄墙区 x=[9,17], 3排墙间距4m, 交错偏移3m。

墙端点:
  墙0 (x=9,  wy=0): y in [-5, 5]
  墙1 (x=13, wy=3): y in [-2, 8]
  墙2 (x=17, wy=0): y in [-5, 5]

回形路线: 交替绕墙的上下边缘，形成蛇形穿梭。

用法: rosrun planning auto_evasion.py
参数: _arrive_dist:=2.0  _timeout:=30.0  _loop:=true
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 墙边缘坐标 (留1.5m余量)
W0_BOT = -6.5   # 墙0下边缘 y=-5, 留余量
W0_TOP =  6.5   # 墙0上边缘 y=5
W1_BOT = -3.5   # 墙1下边缘 y=-2
W1_TOP =  9.5   # 墙1上边缘 y=8
W2_BOT = -6.5   # 墙2下边缘 y=-5
W2_TOP =  6.5   # 墙2上边缘 y=5

# 墙前后的x坐标 (墙厚度很小，前后各留1.5m)
X_BEFORE_W0 = 7.0
X_BETWEEN_01 = 11.0
X_BETWEEN_12 = 15.0
X_AFTER_W2 = 19.0

# 回形路线航点 (x, y)
# 正向: 从下方绕墙0 → 从上方绕墙1 → 从下方绕墙2
# 反向: 从上方绕墙2 → 从下方绕墙1 → 从上方绕墙0
WAYPOINTS = [
    # === 正向穿越 (左→右) ===
    (X_BEFORE_W0,   W0_BOT),    # 墙0前方，走到下边缘
    (X_BETWEEN_01,  W0_BOT),    # 从墙0下方绕过
    (X_BETWEEN_01,  W1_TOP),    # 上行到墙1上边缘
    (X_BETWEEN_12,  W1_TOP),    # 从墙1上方绕过
    (X_BETWEEN_12,  W2_BOT),    # 下行到墙2下边缘
    (X_AFTER_W2,    W2_BOT),    # 从墙2下方绕过

    # === 反向穿越 (右→左) ===
    (X_AFTER_W2,    W2_TOP),    # 上行到墙2上边缘
    #(-5.0,-18.0),
    #(-5.0,18.0),
    (19.0, -10.0),
    (-5.0, -10.0)
]


class AutoEvasion:
    def __init__(self):
        rospy.init_node('auto_evasion', anonymous=True)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=1)
        self.target_pos = None
        rospy.Subscriber('/target/odom', Odometry, self.odom_cb)

        # arrive_dist要大于planning的hover判定距离(tracking_dist+tolerance_d=3.5m)
        # 这样目标还在飞行中就切换下一个航点，避免减速悬停
        self.arrive_dist = rospy.get_param('~arrive_dist', 5.0)
        self.timeout = rospy.get_param('~timeout', 30.0)
        self.loop = rospy.get_param('~loop', False)

        rospy.loginfo("[auto_evasion] Waiting for target odom...")
        while self.target_pos is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        rospy.loginfo("[auto_evasion] Got target odom, starting evasion")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])

    def publish_goal(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 3.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("[auto_evasion] Goal: (%.1f, %.1f)", x, y)

    def wait_arrive(self, x, y):
        goal = np.array([x, y])
        t0 = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.target_pos is not None:
                dist = np.linalg.norm(self.target_pos[:2] - goal)
                if dist < self.arrive_dist:
                    rospy.loginfo("[auto_evasion] Arrived (%.1f,%.1f) d=%.2f",
                                  x, y, dist)
                    return True
            if (rospy.Time.now() - t0).to_sec() > self.timeout:
                rospy.logwarn("[auto_evasion] Timeout at (%.1f, %.1f)", x, y)
                return False
            rate.sleep()
        return False

    def run(self):
        rospy.sleep(1.0)
        lap = 0
        while not rospy.is_shutdown():
            lap += 1
            rospy.loginfo("[auto_evasion] === Lap %d ===", lap)
            for i, (wx, wy) in enumerate(WAYPOINTS):
                if rospy.is_shutdown():
                    break
                rospy.loginfo("[auto_evasion] WP %d/%d", i+1, len(WAYPOINTS))
                self.publish_goal(wx, wy)
                self.wait_arrive(wx, wy)
                rospy.sleep(0.3)
            if not self.loop:
                break
        rospy.loginfo("[auto_evasion] Done")


if __name__ == '__main__':
    try:
        node = AutoEvasion()
        node.run()
    except rospy.ROSInterruptException:
        pass