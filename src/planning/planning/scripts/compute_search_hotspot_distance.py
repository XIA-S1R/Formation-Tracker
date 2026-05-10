#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Plot the minimum distance between selected search hotspots and target GT.

For each search episode, this script reads:
  - /target/odom for ground truth
  - /drone{i}/.../search_state to detect search episodes
  - /drone{i}/.../search_targets to get the assigned hotspot for drone i

It then computes, for each time bin after loss, the minimum hotspot-to-GT
distance among all drones in that bin, and aggregates the curve over a batch.
"""
"""
python3 src/planning/planning/scripts/compute_search_hotspot_distance.py \
  --summary-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/summary.csv \
  --only-ok \
  --curve-max-sec 15 \
  --out-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/hotspot_dist_curve.csv \
  --sample-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/hotspot_dist_samples.csv \
  --curve-png /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/hotspot_dist_curve.png
"""

import argparse
import csv
import os
import re
import sys
from collections import defaultdict

import numpy as np

try:
    import rosbag
    import geometry_msgs.msg
except ImportError:
    print("rosbag is required. Run inside a ROS environment (source devel/setup.bash).")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required for plotting.")
    sys.exit(1)


TARGET_TOPIC = "/target/odom"
SEARCH_STATE_FMT = "/drone{d}/drone{d}_target_dpf/search_state"
SEARCH_TARGET_FMT = "/drone{d}/drone{d}_target_dpf/search_targets"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", action="append", default=[],
                   help="tracking bag path (repeatable)")
    p.add_argument("--summary-csv", default="",
                   help="batch summary csv with bag_path column")
    p.add_argument("--only-ok", action="store_true",
                   help="when reading summary.csv, keep only status==ok")
    p.add_argument("--drone-ids", default="",
                   help="comma separated drone ids, e.g. 0,1,2; empty=auto detect")
    p.add_argument("--max-gt-dt", type=float, default=0.10,
                   help="max time diff to nearest /target/odom")
    p.add_argument("--curve-bin-sec", type=float, default=0.2,
                   help="time bin size after loss")
    p.add_argument("--curve-max-sec", type=float, default=15.0,
                   help="max seconds since loss to keep")
    p.add_argument("--out-csv", default="",
                   help="optional aggregated curve csv")
    p.add_argument("--sample-csv", default="",
                   help="optional raw sample csv")
    p.add_argument("--curve-png", default="",
                   help="optional plot png")
    return p.parse_args()


def ensure_parent_dir(path):
    if not path:
        return
    d = os.path.dirname(os.path.abspath(path))
    if d:
      os.makedirs(d, exist_ok=True)


def normalize_path(path, base_dir):
    if not path:
        return ""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


def unique_paths(paths):
    seen = set()
    out = []
    for p in paths:
        if not p:
            continue
        pa = os.path.abspath(p)
        if pa in seen:
            continue
        seen.add(pa)
        out.append(pa)
    return out


def candidate_bag_paths(raw_bag, run_id, base_dir):
    cands = []
    raw_bag = str(raw_bag or "").strip()
    run_id = str(run_id or "").strip()

    if raw_bag:
        cands.append(normalize_path(raw_bag, base_dir))
        raw_dir = os.path.basename(os.path.dirname(raw_bag))
        if re.match(r"^run_\d+$", raw_dir):
            cands.append(os.path.join(base_dir, raw_dir, "tracking.bag"))

    if run_id.isdigit():
        cands.append(os.path.join(base_dir, "run_{:03d}".format(int(run_id)), "tracking.bag"))
        cands.append(os.path.join(base_dir, "run_{}".format(run_id), "tracking.bag"))

    return unique_paths(cands)


def collect_bag_jobs(args):
    jobs = []
    seen = set()
    missing = []

    for b in args.bag:
        b_abs = os.path.abspath(b)
        if not os.path.isfile(b_abs):
            missing.append({"source": "arg", "raw_bag": str(b), "tried": [b_abs]})
            continue
        if b_abs not in seen:
            jobs.append({"bag_path": b_abs, "run_id": "", "status": ""})
            seen.add(b_abs)

    if args.summary_csv:
        summary_csv = os.path.abspath(args.summary_csv)
        if not os.path.isfile(summary_csv):
            raise FileNotFoundError("summary csv not found: {}".format(summary_csv))

        with open(summary_csv, "r", newline="") as f:
            reader = csv.DictReader(f)
            base_dir = os.path.dirname(summary_csv)
            for row in reader:
                status = str(row.get("status", "")).strip().lower()
                if args.only_ok and status != "ok":
                    continue
                raw_bag = str(row.get("bag_path", "")).strip()
                run_id = str(row.get("run_id", "")).strip()
                cands = candidate_bag_paths(raw_bag, run_id, base_dir)
                bag_abs = ""
                for c in cands:
                    if os.path.isfile(c):
                        bag_abs = c
                        break
                if not bag_abs:
                    missing.append({
                        "source": "summary",
                        "run_id": run_id,
                        "raw_bag": raw_bag,
                        "tried": cands,
                    })
                    continue
                if bag_abs in seen:
                    continue
                jobs.append({"bag_path": bag_abs, "run_id": run_id, "status": status})
                seen.add(bag_abs)

    return jobs, missing


def parse_drone_ids_arg(text):
    if not text:
        return []
    out = []
    for tok in text.split(","):
        tok = tok.strip()
        if tok:
            out.append(int(tok))
    return sorted(set(out))


def detect_drone_ids(topic_info):
    ids = set()
    pat_state = re.compile(r"^/drone(\d+)/.*?/search_state$")
    pat_target = re.compile(r"^/drone(\d+)/.*?/search_targets$")
    for topic in topic_info.keys():
        m = pat_state.match(topic)
        if m:
            ids.add(int(m.group(1)))
            continue
        m = pat_target.match(topic)
        if m:
            ids.add(int(m.group(1)))
    return sorted(ids)


def build_search_episodes_from_events(events):
    if not events:
        return []
    events = sorted(events, key=lambda x: x[0])
    episodes = []
    in_search = False
    cur_start = None
    cur_id = 0

    for ts, st in events:
        st = bool(st)
        if st and not in_search:
            in_search = True
            cur_start = float(ts)
            cur_id += 1
        elif (not st) and in_search:
            episodes.append({"episode_id": cur_id, "start": float(cur_start), "end": float(ts)})
            in_search = False
            cur_start = None

    if in_search and cur_start is not None:
        episodes.append({"episode_id": cur_id, "start": float(cur_start), "end": np.inf})
    return episodes


def locate_episode(episodes, ts):
    if not episodes:
        return None
    t = float(ts)
    lo = 0
    hi = len(episodes) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        ep = episodes[mid]
        if t < ep["start"]:
            hi = mid - 1
        elif t >= ep["end"]:
            lo = mid + 1
        else:
            return int(ep["episode_id"]), float(ep["start"])
    return None


def nearest_target_pos(target_t, target_xyz, query_t, max_dt):
    if target_t.size == 0:
        return None
    idx = int(np.searchsorted(target_t, query_t))
    cand = []
    if idx < target_t.size:
        cand.append(idx)
    if idx > 0:
        cand.append(idx - 1)
    best = None
    best_dt = 1e18
    for i in cand:
        dt = abs(float(target_t[i] - query_t))
        if dt < best_dt:
            best_dt = dt
            best = i
    if best is None or best_dt > max_dt:
        return None
    return target_xyz[best]


def search_topic_for_drone(d):
    return SEARCH_TARGET_FMT.format(d=d)


def state_topic_for_drone(d):
    return SEARCH_STATE_FMT.format(d=d)


def topic_drone_id(topic):
    m = re.match(r"^/drone(\d+)/.*?/(search_state|search_targets)$", topic)
    if not m:
        return None
    return int(m.group(1))


def load_bag_samples(bag_path, drone_ids, max_gt_dt, curve_max_sec):
    samples = []

    with rosbag.Bag(bag_path, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        if not drone_ids:
            drone_ids = detect_drone_ids(topic_info)

        if not drone_ids:
            return samples, drone_ids

        target_t = []
        target_xyz = []
        for _, msg, t in bag.read_messages(topics=[TARGET_TOPIC]):
            target_t.append(float(t.to_sec()))
            target_xyz.append([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ])
        if not target_t:
            return samples, drone_ids

        target_t = np.asarray(target_t, dtype=np.float64)
        target_xyz = np.asarray(target_xyz, dtype=np.float64)

        state_topics = [state_topic_for_drone(d) for d in drone_ids]
        state_events = {d: [] for d in drone_ids}
        for topic, msg, t in bag.read_messages(topics=state_topics):
            d = topic_drone_id(topic)
            if d is None or d not in state_events:
                continue
            state_events[d].append((float(t.to_sec()), bool(msg.data)))

        episodes_by_drone = {d: build_search_episodes_from_events(state_events[d]) for d in drone_ids}

        target_topics = [search_topic_for_drone(d) for d in drone_ids]
        for topic, msg, t in bag.read_messages(topics=target_topics):
            d = topic_drone_id(topic)
            if d is None:
                continue
            if d >= len(msg.poses):
                continue
            ep = locate_episode(episodes_by_drone.get(d, []), float(t.to_sec()))
            if ep is None:
                continue
            ep_id, ep_start = ep
            tau = float(t.to_sec()) - ep_start
            if tau < 0.0:
                tau = 0.0
            if curve_max_sec > 0.0 and tau > curve_max_sec:
                continue
            y = nearest_target_pos(target_t, target_xyz, float(t.to_sec()), max_gt_dt)
            if y is None:
                continue

            p = msg.poses[d].position
            dist = float(np.linalg.norm(np.asarray([p.x, p.y, p.z], dtype=np.float64) - y))
            samples.append({
                "bag_path": bag_path,
                "drone_id": d,
                "episode_id": ep_id,
                "t_since_loss_sec": tau,
                "dist": dist,
            })

    return samples, drone_ids


def aggregate_curve(samples, bin_sec):
    if not samples:
        return []
    if bin_sec <= 0.0:
        bin_sec = 0.2

    per_episode_bin_min = {}
    for s in samples:
        key = (s["bag_path"], s["drone_id"], s["episode_id"], int(np.floor(s["t_since_loss_sec"] / bin_sec)))
        dist = float(s["dist"])
        if key not in per_episode_bin_min or dist < per_episode_bin_min[key]:
            per_episode_bin_min[key] = dist

    bin_values = defaultdict(list)
    for (_, _, _, bin_idx), dist in per_episode_bin_min.items():
        bin_values[int(bin_idx)].append(dist)

    rows = []
    for bin_idx in sorted(bin_values.keys()):
        vals = np.asarray(bin_values[bin_idx], dtype=np.float64)
        rows.append({
            "t_since_loss_sec": bin_idx * bin_sec,
            "dist_min_mean": float(np.mean(vals)),
            "dist_min_median": float(np.median(vals)),
            "dist_min_p25": float(np.percentile(vals, 25)),
            "dist_min_p75": float(np.percentile(vals, 75)),
            "n_samples": int(vals.size),
        })
    return rows


def write_csv(path, rows, fieldnames):
    if not path:
        return
    ensure_parent_dir(path)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_curve(rows, png_path):
    if not png_path:
        return
    if not rows:
        return
    ensure_parent_dir(png_path)
    x = np.asarray([r["t_since_loss_sec"] for r in rows], dtype=np.float64)
    y = np.asarray([r["dist_min_mean"] for r in rows], dtype=np.float64)
    y0 = np.asarray([r["dist_min_p25"] for r in rows], dtype=np.float64)
    y1 = np.asarray([r["dist_min_p75"] for r in rows], dtype=np.float64)
    n = np.asarray([r["n_samples"] for r in rows], dtype=np.int32)

    plt.figure(figsize=(9, 4.8))
    plt.plot(x, y, color="#1f77b4", lw=2.0, label="mean of episode-wise min dist")
    plt.fill_between(x, y0, y1, color="#1f77b4", alpha=0.18, label="IQR")
    plt.xlabel("Time since loss (s)")
    plt.ylabel("Min hotspot-to-GT distance (m)")
    plt.grid(True, linestyle="--", alpha=0.35)
    plt.legend(loc="best")
    plt.title("Search hotspot coverage vs time since loss")
    plt.tight_layout()
    plt.savefig(png_path, dpi=180)
    plt.close()


def main():
    args = parse_args()
    jobs, missing = collect_bag_jobs(args)
    if missing:
        print("[warn] missing bag entries: {}".format(len(missing)))

    user_drone_ids = parse_drone_ids_arg(args.drone_ids)
    all_samples = []
    used_bags = 0
    used_drone_ids = set(user_drone_ids)

    for job in jobs:
        bag_path = job["bag_path"]
        try:
            samples, auto_ids = load_bag_samples(
                bag_path=bag_path,
                drone_ids=user_drone_ids if user_drone_ids else [],
                max_gt_dt=args.max_gt_dt,
                curve_max_sec=args.curve_max_sec,
            )
        except Exception as e:
            print("[warn] skip {}: {}".format(bag_path, e))
            continue
        if auto_ids:
            used_drone_ids.update(auto_ids)
        if not samples:
            print("[warn] no valid samples in {}".format(bag_path))
            continue
        used_bags += 1
        all_samples.extend(samples)
        print("[ok] {}: samples={}".format(os.path.basename(os.path.dirname(bag_path)), len(samples)))

    if not all_samples:
        print("no samples collected")
        return 1

    curve_rows = aggregate_curve(all_samples, args.curve_bin_sec)

    print("bags used: {}".format(used_bags))
    print("drone ids: {}".format(",".join(str(d) for d in sorted(used_drone_ids)) if used_drone_ids else "auto"))
    print("raw samples: {}".format(len(all_samples)))
    print("curve bins: {}".format(len(curve_rows)))

    if args.sample_csv:
        write_csv(args.sample_csv, all_samples,
                  ["bag_path", "drone_id", "episode_id", "t_since_loss_sec", "dist"])
    if args.out_csv:
        write_csv(args.out_csv, curve_rows,
                  ["t_since_loss_sec", "dist_min_mean", "dist_min_median",
                   "dist_min_p25", "dist_min_p75", "n_samples"])
    if args.curve_png:
        plot_curve(curve_rows, args.curve_png)

    return 0


if __name__ == "__main__":
    sys.exit(main())
