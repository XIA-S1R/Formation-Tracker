#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Evaluate whether the initial search particle cloud covers GT and stays compact.

Outputs per time bin after loss:
  - min particle distance to GT
  - weighted mean distance to GT
  - unweighted hit ratio within r_cover
  - weighted mass within r_cover
  - cloud spread radius around weighted mean
  - covariance trace

This is intended to support the claim that the early particle cloud is close to GT
and not overly dispersed.
"""
"""
python3 src/planning/planning/scripts/compute_search_initial_cloud_quality.py \
  --summary-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/summary.csv \
  --only-ok \
  --curve-max-sec 5 \
  --out-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/init_cloud_quality.csv \
  --sample-csv /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/init_cloud_quality_samples.csv \
  --curve-png /home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/full_evasion_batch_20260510_144815/init_cloud_quality.png
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
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("rosbag + sensor_msgs.point_cloud2 are required. Run inside ROS env.")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required.")
    sys.exit(1)


TARGET_TOPIC = "/target/odom"
STATE_TOPIC_FMT = "/drone{d}/drone{d}_target_dpf/search_state"
PARTICLE_TOPIC_CANDS = [
    "/drone{d}/drone{d}_target_dpf/search_particles_vis_pre_prune",
    "/drone{d}/drone{d}_target_dpf/search_particles_vis_post_prune",
    "/drone{d}/drone{d}_target_dpf/search_particles_vis",
]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", action="append", default=[], help="tracking bag path (repeatable)")
    p.add_argument("--summary-csv", default="", help="batch summary csv with bag_path column")
    p.add_argument("--only-ok", action="store_true", help="when reading summary.csv, keep only status==ok")
    p.add_argument("--drone-ids", default="", help="comma separated drone ids, empty=auto detect")
    p.add_argument("--topic-mode", default="auto", choices=["auto", "pre_prune", "post_prune", "vis"],
                   help="which particle topic to read")
    p.add_argument("--max-gt-dt", type=float, default=0.10, help="max time diff to nearest GT")
    p.add_argument("--cover-radius", type=float, default=1.0, help="radius for coverage metrics")
    p.add_argument("--curve-bin-sec", type=float, default=0.2, help="time bin size after loss")
    p.add_argument("--curve-max-sec", type=float, default=5.0, help="max seconds since loss to keep")
    p.add_argument("--out-csv", default="", help="optional aggregated curve csv")
    p.add_argument("--sample-csv", default="", help="optional raw sample csv")
    p.add_argument("--curve-png", default="", help="optional plot png")
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
                bag_abs = ""
                for c in candidate_bag_paths(raw_bag, run_id, base_dir):
                    if os.path.isfile(c):
                        bag_abs = c
                        break
                if not bag_abs:
                    missing.append({
                        "source": "summary",
                        "run_id": run_id,
                        "raw_bag": raw_bag,
                        "tried": candidate_bag_paths(raw_bag, run_id, base_dir),
                    })
                    continue
                if bag_abs in seen:
                    continue
                jobs.append({"bag_path": bag_abs, "run_id": run_id, "status": status})
                seen.add(bag_abs)
    return jobs, missing


def parse_drone_ids(text):
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
    pat_particle = re.compile(r"^/drone(\d+)/.*?/search_particles_vis(?:_pre_prune|_post_prune)?$")
    for topic in topic_info.keys():
        m = pat_state.match(topic)
        if m:
            ids.add(int(m.group(1)))
            continue
        m = pat_particle.match(topic)
        if m:
            ids.add(int(m.group(1)))
    return sorted(ids)


def choose_particle_topic(topic_info, d, mode):
    candidates = []
    if mode == "pre_prune":
        candidates = [PARTICLE_TOPIC_CANDS[0].format(d=d)]
    elif mode == "post_prune":
        candidates = [PARTICLE_TOPIC_CANDS[1].format(d=d)]
    elif mode == "vis":
        candidates = [PARTICLE_TOPIC_CANDS[2].format(d=d)]
    else:
        candidates = [cand.format(d=d) for cand in PARTICLE_TOPIC_CANDS]

    for topic in candidates:
        if topic in topic_info:
            return topic
    return ""


def build_search_episodes(events):
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
    best_i = -1
    best_dt = 1e18
    for i in cand:
        dt = abs(float(target_t[i] - query_t))
        if dt < best_dt:
            best_dt = dt
            best_i = i
    if best_i < 0 or best_dt > max_dt:
        return None
    return target_xyz[best_i]


def weighted_quantile(values, weights, q):
    vals = np.asarray(values, dtype=np.float64).reshape(-1)
    w = np.asarray(weights, dtype=np.float64).reshape(-1)
    if vals.size == 0 or w.size != vals.size:
        return 0.0
    mask = np.isfinite(vals) & np.isfinite(w) & (w > 0.0)
    if not np.any(mask):
        return float(np.median(vals)) if vals.size > 0 else 0.0
    vals = vals[mask]
    w = w[mask]
    order = np.argsort(vals)
    vals = vals[order]
    w = w[order]
    cdf = np.cumsum(w)
    if cdf[-1] <= 0.0:
        return float(vals[-1])
    cdf /= cdf[-1]
    qv = np.clip(float(q), 0.0, 1.0)
    idx = int(np.searchsorted(cdf, qv, side="left"))
    idx = min(max(idx, 0), vals.size - 1)
    return float(vals[idx])


def decode_cloud(msg):
    field_names = [f.name for f in msg.fields]
    if not (("x" in field_names) and ("y" in field_names) and ("z" in field_names)):
        return None
    has_i = "intensity" in field_names
    fn = ("x", "y", "z", "intensity") if has_i else ("x", "y", "z")
    rows = list(pc2.read_points(msg, field_names=fn, skip_nans=True))
    if not rows:
        return None
    arr = np.asarray(rows, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        return None
    xyz = arr[:, :3]
    if has_i and arr.shape[1] >= 4:
        w = np.asarray(arr[:, 3], dtype=np.float64)
        w[~np.isfinite(w)] = 0.0
        w[w < 0.0] = 0.0
    else:
        w = np.ones((xyz.shape[0],), dtype=np.float64)
    wsum = float(np.sum(w))
    if not (wsum > 1e-18):
        w = np.ones((xyz.shape[0],), dtype=np.float64)
        wsum = float(np.sum(w))
    w /= wsum
    return {"points": xyz, "weights": w}


def evaluate_cloud_metrics(pcloud, y_vec, cover_radius):
    pts = pcloud["points"]
    w = pcloud["weights"]
    if pts.size == 0 or w.size == 0:
        return None
    y = np.asarray(y_vec, dtype=np.float64).reshape(1, 3)
    d = np.linalg.norm(pts - y, axis=1)
    mu = np.sum(pts * w[:, None], axis=0)
    centered = pts - mu.reshape(1, 3)
    cov = np.sum(w[:, None, None] * centered[:, :, None] * centered[:, None, :], axis=0)
    spread_rms = float(np.sqrt(max(0.0, np.trace(cov))))
    gt_rms = float(np.sqrt(np.sum(w * d * d)))
    mean_dist = float(np.sum(w * d))
    hit_mass = float(np.sum(w * (d <= float(cover_radius)).astype(np.float64)))
    hit_ratio = float(np.mean(d <= float(cover_radius)))
    min_dist = float(np.min(d))
    p90_dist = weighted_quantile(d, w, 0.90)
    gt_err = float(np.linalg.norm(mu - y.reshape(3)))
    return {
        "min_dist": min_dist,
        "mean_dist": mean_dist,
        "hit_ratio": hit_ratio,
        "hit_mass": hit_mass,
        "spread_rms": spread_rms,
        "cov_trace": float(np.trace(cov)),
        "gt_rms": gt_rms,
        "p90_dist": p90_dist,
        "gt_err": gt_err,
    }


def load_bag_samples(bag_path, drone_ids, topic_mode, max_gt_dt, cover_radius, curve_max_sec):
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

        state_topics = [STATE_TOPIC_FMT.format(d=d) for d in drone_ids]
        state_events = {d: [] for d in drone_ids}
        for topic, msg, t in bag.read_messages(topics=state_topics):
            m = re.match(r"^/drone(\d+)/", topic)
            if not m:
                continue
            d = int(m.group(1))
            if d in state_events:
                state_events[d].append((float(t.to_sec()), bool(msg.data)))
        episodes_by_drone = {d: build_search_episodes(state_events[d]) for d in drone_ids}

        topic_map = {}
        all_particle_topics = []
        for d in drone_ids:
            tp = choose_particle_topic(topic_info, d, topic_mode)
            topic_map[d] = tp
            if tp:
                all_particle_topics.append(tp)
        all_particle_topics = sorted(set(all_particle_topics))
        for topic, msg, t in bag.read_messages(topics=all_particle_topics):
            m = re.match(r"^/drone(\d+)/", topic)
            if not m:
                continue
            d = int(m.group(1))
            if d not in topic_map:
                continue
            if topic != topic_map[d]:
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
            cloud = decode_cloud(msg)
            if cloud is None:
                continue
            met = evaluate_cloud_metrics(cloud, y, cover_radius)
            if met is None:
                continue
            met.update({
                "bag_path": bag_path,
                "drone_id": d,
                "episode_id": ep_id,
                "t_since_loss_sec": tau,
                "topic": topic,
            })
            samples.append(met)
    return samples, drone_ids


def aggregate_curve(samples, bin_sec):
    if not samples:
        return []
    if bin_sec <= 0.0:
        bin_sec = 0.2
    bin_map = defaultdict(list)
    for s in samples:
        b = int(np.floor(float(s["t_since_loss_sec"]) / bin_sec))
        bin_map[b].append(s)

    rows = []
    for b in sorted(bin_map.keys()):
        rows_s = bin_map[b]
        def vals(key):
            return np.asarray([float(r[key]) for r in rows_s], dtype=np.float64)
        rows.append({
            "t_since_loss_sec": b * bin_sec,
            "min_dist_mean": float(np.mean(vals("min_dist"))),
            "min_dist_median": float(np.median(vals("min_dist"))),
            "mean_dist_mean": float(np.mean(vals("mean_dist"))),
            "hit_ratio_mean": float(np.mean(vals("hit_ratio"))),
            "hit_mass_mean": float(np.mean(vals("hit_mass"))),
            "spread_rms_mean": float(np.mean(vals("spread_rms"))),
            "cov_trace_mean": float(np.mean(vals("cov_trace"))),
            "gt_rms_mean": float(np.mean(vals("gt_rms"))),
            "gt_err_mean": float(np.mean(vals("gt_err"))),
            "p90_dist_mean": float(np.mean(vals("p90_dist"))),
            "n_samples": int(len(rows_s)),
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
    if not png_path or not rows:
        return
    ensure_parent_dir(png_path)
    x = np.asarray([r["t_since_loss_sec"] for r in rows], dtype=np.float64)
    min_d = np.asarray([r["min_dist_mean"] for r in rows], dtype=np.float64)
    mean_d = np.asarray([r["mean_dist_mean"] for r in rows], dtype=np.float64)
    hit = np.asarray([r["hit_ratio_mean"] for r in rows], dtype=np.float64)
    spread = np.asarray([r["spread_rms_mean"] for r in rows], dtype=np.float64)
    gt_rms = np.asarray([r["gt_rms_mean"] for r in rows], dtype=np.float64)
    n = np.asarray([r["n_samples"] for r in rows], dtype=np.int32)

    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    fig.patch.set_facecolor("white")

    ax = axes[0]
    ax.plot(x, min_d, label="min dist", lw=2.0)
    ax.plot(x, mean_d, label="weighted mean dist", lw=2.0)
    ax.plot(x, gt_rms, label="weighted RMS dist to GT", lw=1.8)
    ax.set_ylabel("Distance to GT (m)")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(loc="best")

    ax = axes[1]
    ax.plot(x, hit, color="#2ca02c", lw=2.0, label="unweighted hit ratio")
    ax.set_ylabel("Hit ratio")
    ax.set_ylim(-0.02, 1.02)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(loc="best")

    ax = axes[2]
    ax.plot(x, spread, color="#d62728", lw=2.0, label="cloud spread radius")
    ax.set_ylabel("Spread (m)")
    ax.set_xlabel("Time since loss (s)")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(loc="best")

    for ax in axes:
        ax2 = ax.twinx()
        ax2.plot(x, n, color="#7f7f7f", alpha=0.20, lw=1.0)
        ax2.set_ylabel("samples", color="#7f7f7f")
        ax2.tick_params(axis="y", labelcolor="#7f7f7f")

    fig.suptitle("Initial particle cloud quality after loss", fontsize=16)
    fig.tight_layout()
    fig.savefig(png_path, dpi=180)
    plt.close(fig)


def main():
    args = parse_args()
    jobs, missing = collect_bag_jobs(args)
    if missing:
        print("[warn] missing bag entries: {}".format(len(missing)))

    user_drone_ids = parse_drone_ids(args.drone_ids)
    all_samples = []
    used_bags = 0
    used_drone_ids = set(user_drone_ids)

    for job in jobs:
        bag_path = job["bag_path"]
        try:
            samples, auto_ids = load_bag_samples(
                bag_path=bag_path,
                drone_ids=user_drone_ids if user_drone_ids else [],
                topic_mode=args.topic_mode,
                max_gt_dt=args.max_gt_dt,
                cover_radius=args.cover_radius,
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
        write_csv(args.sample_csv, all_samples, [
            "bag_path", "drone_id", "episode_id", "t_since_loss_sec", "topic",
            "min_dist", "mean_dist", "hit_ratio", "hit_mass", "spread_rms",
            "cov_trace", "gt_rms", "gt_err", "p90_dist",
        ])
    if args.out_csv:
        write_csv(args.out_csv, curve_rows, [
            "t_since_loss_sec", "min_dist_mean", "min_dist_median",
            "mean_dist_mean", "hit_ratio_mean", "hit_mass_mean",
            "spread_rms_mean", "cov_trace_mean", "gt_rms_mean",
            "gt_err_mean", "p90_dist_mean", "n_samples",
        ])
    if args.curve_png:
        plot_curve(curve_rows, args.curve_png)

    return 0


if __name__ == "__main__":
    sys.exit(main())
