#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import select
import subprocess
import sys
import termios
import tty
from pathlib import Path

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description="手动选时刻：按键截图并记录相对时间（相对目标开始运动）"
    )
    parser.add_argument(
        "--target-topic",
        default="/target/odom",
        help="用于判定开始运动与计时的目标里程计话题",
    )
    parser.add_argument(
        "--move-thresh",
        type=float,
        default=0.3,
        help="目标位移超过该阈值(m)后开始计时",
    )
    parser.add_argument(
        "--window-pattern",
        default="RViz",
        help="自动查找窗口标题关键字（默认 RViz）",
    )
    parser.add_argument(
        "--window-id",
        default="",
        help="手动指定窗口ID（如 0x4a00007），可跳过自动查找",
    )
    parser.add_argument(
        "--outdir",
        default="experiment_results/manual_rviz_shots",
        help="截图输出目录",
    )
    parser.add_argument(
        "--times-output",
        default="/tmp/manual_snapshot_times.txt",
        help="记录的相对时间输出文件",
    )
    return parser.parse_args()


def find_window_id_by_pattern(pattern):
    try:
        ret = subprocess.run(
            ["xwininfo", "-root", "-tree"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
    except Exception as e:
        raise RuntimeError(f"xwininfo 调用失败: {e}")

    lines = ret.stdout.splitlines()
    pat_title = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"(.*)"')
    pat_geom = re.compile(r"(\d+)x(\d+)\+")
    key = pattern.lower()

    cands = []
    for line in lines:
        m = pat_title.search(line)
        if not m:
            continue
        wid = m.group(1)
        title = m.group(2)
        if key not in title.lower():
            continue
        g = pat_geom.search(line)
        if g:
            area = int(g.group(1)) * int(g.group(2))
        else:
            area = 0
        cands.append((area, wid, title))

    if not cands:
        return None, None
    cands.sort(reverse=True, key=lambda x: x[0])
    _, wid, title = cands[0]
    return wid, title


def capture_window(window_id, png_path):
    ret = subprocess.run(
        ["import", "-window", window_id, str(png_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return ret.returncode == 0, ret.stderr.strip()


class TargetClock:
    def __init__(self):
        self.first_pos = None
        self.start_time = None
        self.latest_time = None
        self.latest_pos = None

    def cb(self, msg):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=float)
        t = msg.header.stamp.to_sec()
        if t <= 0.0:
            t = rospy.get_time()

        self.latest_pos = pos
        self.latest_time = t
        if self.first_pos is None:
            self.first_pos = pos.copy()


def main():
    args = parse_args()

    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    times_out = Path(args.times_output).expanduser().resolve()
    times_out.parent.mkdir(parents=True, exist_ok=True)

    if args.window_id:
        window_id = args.window_id
        window_title = "manual_window_id"
    else:
        window_id, window_title = find_window_id_by_pattern(args.window_pattern)
        if not window_id:
            print(
                f"[error] 未找到标题包含 '{args.window_pattern}' 的窗口。"
                "请先打开 RViz，或手动传 --window-id。"
            )
            return 1

    rospy.init_node("manual_rviz_mark_and_shot", anonymous=True)
    clock = TargetClock()
    rospy.Subscriber(args.target_topic, Odometry, clock.cb, queue_size=1)

    print(f"[info] 输出目录: {outdir}")
    print(f"[info] 时间文件: {times_out}")
    print(f"[info] RViz窗口: {window_title} ({window_id})")
    print(f"[info] 监听话题: {args.target_topic}")
    print("[keys] s=截图+记时, 空格=仅记时, p=打印当前时间, q=结束")

    try:
        tty_in = open("/dev/tty", "r")
    except Exception as e:
        print(f"[error] 无法打开 /dev/tty: {e}")
        print("[hint] 请在系统终端中运行，不要在无TTY的面板运行。")
        return 1

    fd = tty_in.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    marks = []
    start_announced = False
    move_thresh = max(0.0, float(args.move_thresh))

    try:
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            if (
                not start_announced
                and clock.first_pos is not None
                and clock.latest_pos is not None
                and clock.latest_time is not None
            ):
                moved = float(np.linalg.norm(clock.latest_pos - clock.first_pos))
                if moved >= move_thresh:
                    clock.start_time = clock.latest_time
                    start_announced = True
                    print(
                        f"\n[start] motion_start={clock.start_time:.3f}s "
                        f"(moved={moved:.3f}m)"
                    )

            if select.select([tty_in], [], [], 0)[0]:
                ch = tty_in.read(1)
                if ch.lower() == "q":
                    break

                if clock.start_time is None or clock.latest_time is None:
                    print("[wait] 还未检测到目标开始运动，按键忽略。")
                    continue

                rel = float(clock.latest_time - clock.start_time)

                if ch.lower() == "p":
                    print(f"[time] rel={rel:.3f}s")
                    continue

                if ch == " ":
                    marks.append(rel)
                    print(f"[mark] rel={rel:.3f}s")
                    continue

                if ch.lower() == "s":
                    marks.append(rel)
                    tag = f"{rel:.3f}".replace(".", "p")
                    png = outdir / f"rviz_rel_{tag}s.png"
                    ok, err = capture_window(window_id, png)
                    if ok:
                        print(f"[shot] rel={rel:.3f}s -> {png}")
                    else:
                        print(f"[warn] 截图失败: {err}，但已记时 rel={rel:.3f}s")
                    continue

            rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        tty_in.close()

    marks = sorted(marks)
    times_out.write_text(",".join(f"{t:.3f}" for t in marks), encoding="utf-8")
    print(f"[done] 已记录 {len(marks)} 个时刻")
    print(f"[done] 保存时间到: {times_out}")
    if marks:
        print("[done] times:", ", ".join(f"{t:.3f}" for t in marks))
    return 0


if __name__ == "__main__":
    sys.exit(main())
