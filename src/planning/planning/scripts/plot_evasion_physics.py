#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从一个 tracking.bag 里画 4 张物理合理性验证图：
  v.png       三架追踪者 |v|
  a.png       三架追踪者 |a|  (IMU linear_acceleration)
  fm.png      三架追踪者 |f|/m (so3cmd.force 归一化)
  newton.png  |a_xy| vs g*tan(tilt) 散点

用法:
  rosrun planning plot_evasion_physics.py --bag tracking.bag --out out_dir
"""
import argparse
import os
import numpy as np

try:
    import rosbag
except ImportError:
    print("rosbag required, source ROS env first.")
    raise

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.81
MASS_DEFAULT = 0.98

ODOM_FMT = "/drone{d}/odom"
IMU_FMT  = "/drone{d}/imu"
SO3_FMT  = "/drone{d}/so3cmd"

COLORS = ["#2c7fb8", "#f16913", "#2ca25f"]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--drones", default="0,1,2")
    p.add_argument("--mass", type=float, default=MASS_DEFAULT)
    return p.parse_args()


def q_to_R(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def tilt_from_q(w, x, y, z):
    R = q_to_R(w, x, y, z)
    # z_body 在世界系
    z_body_world = R[:, 2]
    return float(np.arccos(max(-1.0, min(1.0, z_body_world[2]))))


def read_bag(bag_path, drones):
    data = {d: {"t_odom": [], "v": [],
                "t_imu": [], "a": [], "tilt": [],
                "t_so3": [], "f": []} for d in drones}

    topics = []
    for d in drones:
        topics += [ODOM_FMT.format(d=d), IMU_FMT.format(d=d), SO3_FMT.format(d=d)]

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            for d in drones:
                if topic == ODOM_FMT.format(d=d):
                    vx = msg.twist.twist.linear.x
                    vy = msg.twist.twist.linear.y
                    vz = msg.twist.twist.linear.z
                    data[d]["t_odom"].append(ts)
                    data[d]["v"].append(np.sqrt(vx * vx + vy * vy + vz * vz))
                    break
                if topic == IMU_FMT.format(d=d):
                    ax = msg.linear_acceleration.x
                    ay = msg.linear_acceleration.y
                    az = msg.linear_acceleration.z
                    q = msg.orientation
                    data[d]["t_imu"].append(ts)
                    data[d]["a"].append([ax, ay, az])
                    data[d]["tilt"].append(tilt_from_q(q.w, q.x, q.y, q.z)
                                           if not (q.w == 0 and q.x == 0 and q.y == 0 and q.z == 0)
                                           else 0.0)
                    break
                if topic == SO3_FMT.format(d=d):
                    fx = msg.force.x
                    fy = msg.force.y
                    fz = msg.force.z
                    data[d]["t_so3"].append(ts)
                    data[d]["f"].append(np.sqrt(fx * fx + fy * fy + fz * fz))
                    break

    for d in drones:
        data[d]["t_odom"] = np.asarray(data[d]["t_odom"])
        data[d]["v"]      = np.asarray(data[d]["v"])
        data[d]["t_imu"]  = np.asarray(data[d]["t_imu"])
        data[d]["a"]      = np.asarray(data[d]["a"]).reshape(-1, 3) if data[d]["a"] else np.zeros((0, 3))
        data[d]["tilt"]   = np.asarray(data[d]["tilt"])
        data[d]["t_so3"]  = np.asarray(data[d]["t_so3"])
        data[d]["f"]      = np.asarray(data[d]["f"])
    return data


def rebase_time(data):
    t0 = min([x["t_odom"][0] for x in data.values() if x["t_odom"].size > 0],
             default=0.0)
    for d in data:
        for k in ("t_odom", "t_imu", "t_so3"):
            if data[d][k].size > 0:
                data[d][k] = data[d][k] - t0
    return data


def plot_v(data, out_path):
    fig, ax = plt.subplots(figsize=(9.0, 4.2))
    for i, d in enumerate(sorted(data.keys())):
        if data[d]["t_odom"].size == 0:
            continue
        ax.plot(data[d]["t_odom"], data[d]["v"],
                color=COLORS[i % len(COLORS)], lw=1.4, label=f"drone{d}")
    ax.axhline(3.0, color="red", ls="--", lw=1.0, alpha=0.7, label="max_speed=3.0")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|v| (m/s)")
    ax.set_title("Tracker speed magnitude")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_a(data, out_path):
    fig, ax = plt.subplots(figsize=(9.0, 4.2))
    for i, d in enumerate(sorted(data.keys())):
        a = data[d]["a"]
        if a.shape[0] == 0:
            continue
        mag = np.linalg.norm(a, axis=1)
        ax.plot(data[d]["t_imu"], mag,
                color=COLORS[i % len(COLORS)], lw=1.2, label=f"drone{d}")
    ax.axhline(6.0, color="red", ls="--", lw=1.0, alpha=0.7, label="max_accel=6.0")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|a| (m/s^2)")
    ax.set_title("Tracker acceleration magnitude (IMU)")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_fm(data, mass, out_path):
    fig, ax = plt.subplots(figsize=(9.0, 4.2))
    for i, d in enumerate(sorted(data.keys())):
        if data[d]["t_so3"].size == 0:
            continue
        fm = data[d]["f"] / max(mass, 1e-6)
        ax.plot(data[d]["t_so3"], fm,
                color=COLORS[i % len(COLORS)], lw=1.2, label=f"drone{d}")
    ax.axhline(G, color="black", ls=":", lw=1.0, alpha=0.7, label="g=9.81")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|f|/m (m/s^2)")
    ax.set_title("Mass-normalized thrust (so3cmd.force / mass)")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_newton(data, out_path):
    fig, ax = plt.subplots(figsize=(6.0, 6.0))
    x_all, y_all = [], []
    for i, d in enumerate(sorted(data.keys())):
        a = data[d]["a"]
        tilt = data[d]["tilt"]
        if a.shape[0] == 0 or tilt.size == 0:
            continue
        a_xy = np.linalg.norm(a[:, :2], axis=1)
        n = min(a_xy.size, tilt.size)
        x = G * np.tan(tilt[:n])
        y = a_xy[:n]
        ax.scatter(x, y, s=3, alpha=0.35,
                   color=COLORS[i % len(COLORS)], label=f"drone{d}")
        x_all.append(x)
        y_all.append(y)
    if x_all:
        xm = np.concatenate(x_all)
        ym = np.concatenate(y_all)
        lim = max(float(xm.max(initial=0)), float(ym.max(initial=0))) * 1.05 + 0.1
        ax.plot([0, lim], [0, lim], color="red", ls="--", lw=1.2, label="y = x")
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
    ax.set_xlabel("g * tan(tilt)  (m/s^2)")
    ax.set_ylabel("|a_xy|  (m/s^2)")
    ax.set_title("Newton check: ideal quadrotor a_xy = g*tan(tilt)")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main():
    args = parse_args()
    drones = [int(x) for x in args.drones.split(",") if x.strip()]
    os.makedirs(args.out, exist_ok=True)

    print(f"[plot_evasion_physics] reading {args.bag} drones={drones}")
    data = read_bag(args.bag, drones)
    data = rebase_time(data)
    for d in drones:
        print(f"  drone{d}: odom={data[d]['t_odom'].size} "
              f"imu={data[d]['t_imu'].size} so3={data[d]['t_so3'].size}")

    plot_v(data, os.path.join(args.out, "v.png"))
    plot_a(data, os.path.join(args.out, "a.png"))
    plot_fm(data, args.mass, os.path.join(args.out, "fm.png"))
    plot_newton(data, os.path.join(args.out, "newton.png"))
    print(f"Wrote 4 PNGs into {args.out}")


if __name__ == "__main__":
    main()
