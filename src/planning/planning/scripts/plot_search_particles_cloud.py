#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Visualize real particle clouds after target loss from bag topic:
  /droneX/droneX_target_dpf/search_particles_vis

Usage:
  python3 plot_search_particles_cloud.py \
    --bag /path/to/tracking.bag \
    --loss_idx 1 \
    --duration 12 \
    --n_frames 8 \
    --out_dir ./search_particles_out
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("Need rosbag environment: source devel/setup.bash")
    sys.exit(1)


DRONE_COLORS = ["#e74c3c", "#2ecc71", "#3498db"]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--loss_idx", type=int, default=1, help="loss event index starting from 1")
    p.add_argument("--duration", type=float, default=12.0, help="seconds after loss start")
    p.add_argument("--n_frames", type=int, default=8, help="frames in output panel")
    p.add_argument("--out_dir", default="./search_particles_out")
    p.add_argument("--point_size", type=float, default=10.0)
    p.add_argument("--obstacle_max_points", type=int, default=40000,
                   help="max obstacle points used for top-view background")
    return p.parse_args()


def nearest(records, t_query):
    if not records:
        return None
    ts = [r[0] for r in records]
    idx = int(np.searchsorted(ts, t_query))
    if idx <= 0:
        return records[0]
    if idx >= len(records):
        return records[-1]
    if abs(ts[idx] - t_query) < abs(ts[idx - 1] - t_query):
        return records[idx]
    return records[idx - 1]


def find_loss_intervals(search_states):
    d0 = sorted(search_states[0])
    intervals = []
    prev = False
    t0 = None
    for t, s in d0:
        cur = bool(s)
        if cur and (not prev):
            t0 = t
        elif (not cur) and prev and t0 is not None:
            intervals.append((t0, t, t - t0))
            t0 = None
        prev = cur
    return intervals


def load_bag(path, obstacle_max_points):
    bag = rosbag.Bag(path)
    t0 = bag.get_start_time()

    search_states = {d: [] for d in range(3)}
    particle_clouds = {d: [] for d in range(3)}  # (rel_t, Nx4[x,y,z,intensity])
    drone_odom = {d: [] for d in range(3)}       # (rel_t, x, y, z)
    target_odom = []                              # (rel_t, x, y, z)
    obstacle_xy = None                            # (N,2) from /global_map

    topics = (
        [f"/drone{d}/drone{d}_target_dpf/search_state" for d in range(3)] +
        [f"/drone{d}/drone{d}_target_dpf/search_particles_vis" for d in range(3)] +
        [f"/drone{d}/odom" for d in range(3)] +
        ["/target/odom", "/global_map"]
    )

    for topic, msg, t in bag.read_messages(topics=topics):
        rel = t.to_sec() - t0
        parts = topic.split("/")
        d = -1
        if len(parts) > 1 and parts[1].startswith("drone"):
            try:
                d = int(parts[1].replace("drone", ""))
            except Exception:
                d = -1

        if "search_state" in topic and d >= 0:
            search_states[d].append((rel, bool(msg.data)))
        elif "search_particles_vis" in topic and d >= 0:
            field_names = [f.name for f in msg.fields]
            if all(k in field_names for k in ("x", "y", "z", "intensity")):
                pts = list(pc2.read_points(
                    msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
                if pts:
                    arr = np.asarray(pts, dtype=np.float64)
                    particle_clouds[d].append((rel, arr))
            elif all(k in field_names for k in ("x", "y", "z")):
                pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                if pts:
                    arr3 = np.asarray(pts, dtype=np.float64)
                    inten = np.ones((arr3.shape[0], 1), dtype=np.float64)
                    arr = np.hstack([arr3, inten])
                    particle_clouds[d].append((rel, arr))
        elif topic == "/target/odom":
            p = msg.pose.pose.position
            target_odom.append((rel, p.x, p.y, p.z))
        elif topic == "/global_map" and obstacle_xy is None:
            pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if pts:
                arr = np.asarray(pts, dtype=np.float64)
                if obstacle_max_points > 0 and arr.shape[0] > obstacle_max_points:
                    idx = np.random.choice(arr.shape[0], obstacle_max_points, replace=False)
                    arr = arr[idx]
                obstacle_xy = arr[:, :2]
        elif "/odom" in topic and "target" not in topic and d >= 0:
            p = msg.pose.pose.position
            drone_odom[d].append((rel, p.x, p.y, p.z))

    bag.close()
    print("particle_cloud records:", [len(particle_clouds[d]) for d in range(3)])
    print("obstacle points:", 0 if obstacle_xy is None else obstacle_xy.shape[0])
    return search_states, particle_clouds, drone_odom, target_odom, obstacle_xy


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    search_states, particle_clouds, drone_odom, target_odom, obstacle_xy = load_bag(
        args.bag, args.obstacle_max_points)
    total_cloud_msgs = sum(len(particle_clouds[d]) for d in range(3))
    if total_cloud_msgs == 0:
        print("No search_particles_vis data in bag. This bag cannot visualize real particle clouds.")
        print("Please re-record with topics /droneX/droneX_target_dpf/search_particles_vis enabled.")
        return 2
    intervals = find_loss_intervals(search_states)
    if not intervals:
        print("No loss interval found from search_state.")
        return 1
    if args.loss_idx < 1 or args.loss_idx > len(intervals):
        print("loss_idx out of range: {} (total {})".format(args.loss_idx, len(intervals)))
        return 1

    t_loss_start, t_loss_end, loss_dur = intervals[args.loss_idx - 1]
    t_plot_end = t_loss_start + args.duration
    frame_times = np.linspace(t_loss_start, t_plot_end, args.n_frames)
    print("Loss #{}: t={:.3f}s dur={:.3f}s plot_end={:.3f}s".format(
        args.loss_idx, t_loss_start, loss_dur, t_plot_end))

    # Fixed visualization bounds for consistent comparisons across runs
    xlim = (-30.0, 30.0)
    ylim = (-30.0, 30.0)

    cols = 4
    rows = int(np.ceil(args.n_frames / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.4 * cols, 4.8 * rows))
    axes = np.asarray(axes).reshape(-1)
    fig.patch.set_facecolor("#05071a")
    fig.suptitle(
        "Real Particle Clouds After Loss | loss#{} (t={:.2f}s, dur={:.2f}s)".format(
            args.loss_idx, t_loss_start, loss_dur),
        color="white", fontsize=18)

    for i, t_q in enumerate(frame_times):
        ax = axes[i]
        ax.set_facecolor("#1a1a2e")
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)

        in_search = (t_loss_start <= t_q <= t_loss_end)
        status = "SEARCH +{:.1f}s".format(t_q - t_loss_start) if in_search else "TRACKING"
        ax.set_title("t={:.2f}s\n{}".format(t_q, status), color="white", fontsize=13)

        # obstacle top-view background
        if obstacle_xy is not None and obstacle_xy.size > 0:
            ax.scatter(obstacle_xy[:, 0], obstacle_xy[:, 1],
                       s=1.2, c="#7f8c8d", alpha=0.20, edgecolors="none", zorder=1)

        for d in range(3):
            entry = nearest(particle_clouds[d], t_q)
            if entry is None:
                continue
            cloud = entry[1]
            if cloud.size == 0:
                continue
            inten = cloud[:, 3]
            inten = np.clip(inten, 0.0, 1.0)
            sizes = args.point_size * (0.35 + 0.9 * inten)
            ax.scatter(
                cloud[:, 0], cloud[:, 1], s=sizes,
                c=DRONE_COLORS[d], alpha=0.75, edgecolors="none", zorder=3
            )
            # weighted center for quick perception
            w = np.clip(inten, 1e-3, None)
            cx = float(np.average(cloud[:, 0], weights=w))
            cy = float(np.average(cloud[:, 1], weights=w))
            ax.plot(cx, cy, marker="D", ms=9, color=DRONE_COLORS[d],
                    markeredgecolor="white", markeredgewidth=1.0, zorder=4)

        # drone odom
        for d in range(3):
            od = nearest(drone_odom[d], t_q)
            if od is not None:
                ax.plot(od[1], od[2], marker="^", ms=11, color=DRONE_COLORS[d],
                        markeredgecolor="white", markeredgewidth=1.2, zorder=5)

        # target truth
        tg = nearest(target_odom, t_q)
        if tg is not None:
            tgt_color = "#ff6b6b" if in_search else "#ffd43b"
            ax.plot(tg[1], tg[2], marker="*", ms=20, color=tgt_color,
                    markeredgecolor="white", markeredgewidth=1.0, zorder=6)

        ax.tick_params(colors="gray", labelsize=8)
        ax.set_xlabel("X (m)", color="gray", fontsize=9)
        ax.set_ylabel("Y (m)", color="gray", fontsize=9)
        for sp in ax.spines.values():
            sp.set_color("#444444")

    for j in range(len(frame_times), len(axes)):
        axes[j].axis("off")

    legend = [
        Line2D([0], [0], marker="o", color="w", label="Obstacle map",
               markerfacecolor="#7f8c8d", markersize=7, alpha=0.8),
        Line2D([0], [0], marker="o", color="w", label="Particle Cloud (real)",
               markerfacecolor="#4cc9f0", markersize=8, alpha=0.7),
        Line2D([0], [0], marker="D", color="w", label="Weighted cloud center",
               markerfacecolor="#4cc9f0", markersize=8),
        Line2D([0], [0], marker="^", color="w", label="Drone odom",
               markerfacecolor="gray", markersize=8),
        Line2D([0], [0], marker="*", color="w", label="Target truth",
               markerfacecolor="#ff6b6b", markersize=14),
    ]
    lg = fig.legend(handles=legend, loc="lower center", ncol=5,
                    frameon=True, facecolor="#1a1a2e", edgecolor="#444444",
                    fontsize=11)
    for txt in lg.get_texts():
        txt.set_color("white")
    fig.tight_layout(rect=[0, 0.06, 1, 0.95])

    out_png = os.path.join(args.out_dir, "search_particles_loss{}.png".format(args.loss_idx))
    fig.savefig(out_png, dpi=180)
    plt.close(fig)
    print("Saved:", out_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
