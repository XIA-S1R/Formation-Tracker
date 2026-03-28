#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import datetime
import os
import re
import subprocess
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description="运行时自动抓取 RViz 窗口截图（不依赖 bag 回放）"
    )
    parser.add_argument(
        "--times",
        type=str,
        default="18.9,25.9,36.5,41.8,47.1",
        help="截图时刻（秒），相对机动开始时间，逗号分隔",
    )
    parser.add_argument(
        "--outdir",
        type=str,
        default="",
        help="输出目录（默认自动创建到 experiment_results/runtime_rviz_shots_时间戳）",
    )
    parser.add_argument(
        "--target-topic",
        type=str,
        default="/target/odom",
        help="用于判定机动开始与计时的目标里程计话题",
    )
    parser.add_argument(
        "--move-thresh",
        type=float,
        default=0.3,
        help="目标位移超过该阈值（米）判定机动开始",
    )
    parser.add_argument(
        "--window-pattern",
        type=str,
        default="RViz",
        help="窗口标题匹配关键字（默认 RViz）",
    )
    parser.add_argument(
        "--window-id",
        type=str,
        default="",
        help="手工指定窗口 ID（如 0x4a00007），可跳过自动查找",
    )
    parser.add_argument(
        "--poll-hz",
        type=float,
        default=30.0,
        help="主循环频率",
    )
    parser.add_argument(
        "--start-now",
        action="store_true",
        help="不等待目标移动，脚本启动后立刻开始计时",
    )
    return parser.parse_args()


def parse_times(raw):
    times = []
    for s in raw.split(","):
        s = s.strip()
        if not s:
            continue
        v = float(s)
        if v < 0:
            raise ValueError("times 中不能包含负数")
        times.append(v)
    if not times:
        raise ValueError("times 为空")
    return sorted(times)


def parse_window_candidates(xwininfo_text, pattern):
    candidates = []
    lines = xwininfo_text.splitlines()
    title_pat = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"(.*)"')
    geom_pat = re.compile(r"(\d+)x(\d+)\+")
    pattern_low = pattern.lower()

    for line in lines:
        m = title_pat.search(line)
        if not m:
            continue
        win_id = m.group(1)
        title = m.group(2)
        if pattern_low not in title.lower():
            continue
        g = geom_pat.search(line)
        if g:
            w = int(g.group(1))
            h = int(g.group(2))
            area = w * h
        else:
            w = 0
            h = 0
            area = 0
        candidates.append((area, win_id, title, w, h))
    candidates.sort(reverse=True, key=lambda x: x[0])
    return candidates


def find_window_id(pattern):
    try:
        ret = subprocess.run(
            ["xwininfo", "-root", "-tree"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
    except Exception as e:
        raise RuntimeError("调用 xwininfo 失败: {}".format(e))

    cands = parse_window_candidates(ret.stdout, pattern)
    if not cands:
        raise RuntimeError(
            '未找到包含 "{}" 的窗口。请先打开 RViz，并确保标题可见。'.format(pattern)
        )
    best = cands[0]
    return best[1], best[2]


def capture_window(window_id, out_png):
    cmd = ["import", "-window", window_id, out_png]
    ret = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if ret.returncode != 0:
        raise RuntimeError("截图失败: {}".format(ret.stderr.strip()))


class OdomClock(object):
    def __init__(self):
        self.first_pos = None
        self.last_pos = None
        self.last_time = None

    def cb(self, msg):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=float)
        t = msg.header.stamp.to_sec()
        if t <= 0.0:
            t = rospy.get_time()

        if self.first_pos is None:
            self.first_pos = pos.copy()
        self.last_pos = pos
        self.last_time = t


def main():
    args = parse_args()
    try:
        shot_times = parse_times(args.times)
    except ValueError as e:
        print("[error] {}".format(e))
        return 1

    if args.outdir:
        outdir = args.outdir
    else:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = os.path.join("experiment_results", "runtime_rviz_shots_{}".format(ts))
    os.makedirs(outdir, exist_ok=True)

    rospy.init_node("capture_rviz_runtime_shots", anonymous=True)
    clock = OdomClock()
    rospy.Subscriber(args.target_topic, Odometry, clock.cb, queue_size=1)

    if args.window_id:
        window_id = args.window_id
        window_title = "manual_window_id"
    else:
        window_id = None
        window_title = ""
        while not rospy.is_shutdown():
            try:
                window_id, window_title = find_window_id(args.window_pattern)
                break
            except RuntimeError as e:
                print("[wait] {} (1s后重试)".format(e))
                time.sleep(1.0)
        if rospy.is_shutdown():
            return 0

    print("[info] 输出目录: {}".format(outdir))
    print("[info] RViz窗口: {} ({})".format(window_title, window_id))
    print("[info] 截图时刻(秒): {}".format(", ".join("{:.2f}".format(t) for t in shot_times)))
    print("[info] 请保持 RViz 窗口不最小化，且位于可见屏幕区域。")

    rate = rospy.Rate(max(1.0, args.poll_hz))
    last_wait_print = 0.0
    while clock.last_pos is None and not rospy.is_shutdown():
        now_wall = time.time()
        if now_wall - last_wait_print > 1.0:
            print("[wait] 等待 {}".format(args.target_topic))
            last_wait_print = now_wall
        rate.sleep()

    if rospy.is_shutdown():
        return 0

    if args.start_now:
        if clock.last_time is not None:
            start_time = clock.last_time
        else:
            start_time = rospy.get_time()
        print("[info] 立刻开始计时")
    else:
        print("[wait] 等待目标开始运动，阈值 {:.2f} m".format(args.move_thresh))
        start_time = None
        while not rospy.is_shutdown():
            if clock.last_pos is not None and clock.first_pos is not None:
                moved = np.linalg.norm(clock.last_pos - clock.first_pos)
                if moved >= args.move_thresh:
                    start_time = clock.last_time if clock.last_time is not None else rospy.get_time()
                    print("[info] 检测到机动开始，位移 {:.3f} m".format(moved))
                    break
            rate.sleep()

    if rospy.is_shutdown():
        return 0

    idx = 0
    n = len(shot_times)
    while not rospy.is_shutdown() and idx < n:
        now_t = clock.last_time if clock.last_time is not None else rospy.get_time()
        elapsed = now_t - start_time
        target_t = shot_times[idx]
        if elapsed + 1e-6 >= target_t:
            time_tag = "{:05.2f}".format(target_t).replace(".", "p")
            fname = "rviz_t{}s.png".format(time_tag)
            out_png = os.path.join(outdir, fname)
            try:
                capture_window(window_id, out_png)
                print("[shot] t={:.2f}s -> {}".format(elapsed, out_png))
            except RuntimeError as e:
                print("[warn] {}，尝试重新查找窗口一次".format(e))
                try:
                    window_id, window_title = find_window_id(args.window_pattern)
                    capture_window(window_id, out_png)
                    print("[shot] t={:.2f}s -> {} (recovered)".format(elapsed, out_png))
                except Exception as e2:
                    print("[error] 截图失败，已跳过该时刻: {}".format(e2))
            idx += 1
        else:
            rate.sleep()

    print("[done] 已完成 {} 张截图".format(idx))
    return 0


if __name__ == "__main__":
    sys.exit(main())
