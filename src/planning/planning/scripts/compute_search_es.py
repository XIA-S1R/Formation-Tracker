#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Compute search-mode estimation metrics from tracking bags.

Inputs:
  1) --bag <path> [--bag <path> ...]
  2) --summary-csv <summary.csv> (batch mode, reads bag_path column)

Outputs:
  - Prints per-bag/per-drone statistics for:
      * Energy Score (ES)
      * Hit Probability within radius r_hit
      * HPD Coverage (indicator in highest density region with mass alpha)
      * Average Deviation (weighted mean distance to GT)
  - Optionally writes CSV with --out-csv.

ES definition (Monte Carlo):
  ES(F, y) = E||X - y|| - 0.5 * E||X - X'||
where X, X' are i.i.d. from predictive distribution F.

Hit Probability (Monte Carlo):
  P_hit = P(||X - y|| <= r_hit)

HPD Coverage (Monte Carlo):
  1{ y in HPD_alpha(F) }, where HPD_alpha(F) is the highest-density region
  containing probability mass alpha.

Average Deviation (particle-weighted):
  AD = sum_i w_i * ||x_i - y||, where sum_i w_i = 1
"""
"""
python3 src/planning/planning/scripts/compute_search_es.py \
  --summary-csv /你的实验目录/summary.csv \
  --only-ok \
  --mc-samples 256 \
  --hit-radius 1.0 \
  --hpd-alpha 0.90 \
  --hpd-samples 512 \
  --curve-samples-csv /你的输出/out_samples.csv \
  --curve-binned-csv /你的输出/out_curve.csv \
  --curve-bin-sec 0.2 \
  --curve-max-sec 20 \
  --curve-png /你的输出/out_curve.png
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
    print("rosbag + sensor_msgs.point_cloud2 are required. Run in ROS env (e.g. source devel/setup.bash).")
    sys.exit(1)


TARGET_TOPIC = "/target/odom"
GMM_TOPIC_FMT = "/drone{d}/drone{d}_target_dpf/search_pos_gmm"
PARTICLE_TOPIC_FMT = "/drone{d}/drone{d}_target_dpf/search_particles_vis"
STATE_TOPIC_FMT = "/drone{d}/drone{d}_target_dpf/search_state"


def percentile(values, q):
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def mean_or_zero(values):
    if not values:
        return 0.0
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", action="append", default=[],
                        help="tracking.bag path (can be repeated)")
    parser.add_argument("--summary-csv", default="",
                        help="batch input CSV with bag_path column")
    parser.add_argument("--out-csv", default="",
                        help="optional output CSV path")
    parser.add_argument("--only-ok", action="store_true",
                        help="when reading summary.csv, keep only status==ok")
    parser.add_argument("--drone-ids", default="",
                        help="comma separated drone ids, e.g. 0,1,2; empty=auto detect")
    parser.add_argument("--mc-samples", type=int, default=256,
                        help="Monte Carlo samples per ES evaluation")
    parser.add_argument("--hit-radius", type=float, default=1.0,
                        help="hit radius in meters for hit probability P(||X-y||<=r)")
    parser.add_argument("--hpd-alpha", type=float, default=0.90,
                        help="HPD mass alpha in (0,1), e.g. 0.90")
    parser.add_argument("--hpd-samples", type=int, default=512,
                        help="Monte Carlo samples to estimate HPD threshold")
    parser.add_argument("--hpd-voxel-size", type=float, default=1.0,
                        help="voxel size (m) for particle HPD coverage")
    parser.add_argument("--metric-source", default="particles", choices=["particles", "gmm", "auto"],
                        help="distribution source for metrics: particles(voxel-HPD), gmm, or auto")
    parser.add_argument("--max-gt-dt", type=float, default=0.10,
                        help="max time diff (sec) to nearest /target/odom")
    parser.add_argument("--seed", type=int, default=42,
                        help="random seed for Monte Carlo")
    parser.add_argument("--curve-samples-csv", default="",
                        help="optional per-sample curve CSV (includes es, avg_dev, hit_prob, hpd_cover)")
    parser.add_argument("--curve-binned-csv", default="",
                        help="optional binned curve CSV")
    parser.add_argument("--curve-bin-sec", type=float, default=0.2,
                        help="curve bin size in seconds")
    parser.add_argument("--curve-max-sec", type=float, default=0.0,
                        help="max seconds since loss for curve (<=0 means no limit)")
    parser.add_argument("--curve-per-drone", action="store_true",
                        help="when exporting binned curve, split by drone_id")
    parser.add_argument("--curve-png", default="",
                        help="optional png path for ES/Hit/HPD-vs-time curves")
    return parser.parse_args()


def normalize_path(path, base_dir):
    if not path:
        return ""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


def ensure_parent_dir(file_path):
    if not file_path:
        return
    out_dir = os.path.dirname(os.path.abspath(file_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)


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
            missing.append({
                "source": "arg",
                "run_id": "",
                "raw_bag": str(b),
                "tried": [b_abs],
            })
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
                jobs.append({
                    "bag_path": bag_abs,
                    "run_id": run_id,
                    "status": status,
                })
                seen.add(bag_abs)

    return jobs, missing


def detect_drone_ids(topic_info):
    ids = set()
    pat_gmm = re.compile(r"^/drone(\d+)/drone\1_target_dpf/search_pos_gmm$")
    pat_particle = re.compile(r"^/drone(\d+)/drone\1_target_dpf/search_particles_vis$")
    for topic in topic_info.keys():
        m1 = pat_gmm.match(topic)
        if m1:
            ids.add(int(m1.group(1)))
            continue
        m2 = pat_particle.match(topic)
        if m2:
            ids.add(int(m2.group(1)))
    return sorted(ids)


def parse_drone_ids_arg(arg_text):
    if not arg_text:
        return []
    out = []
    for tok in arg_text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        out.append(int(tok))
    return sorted(set(out))


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


def build_search_episodes_from_events(events):
    # events: list of (timestamp_sec, bool_state)
    if not events:
        return []
    events = sorted(events, key=lambda x: x[0])
    episodes = []
    in_search = False
    cur_start = None
    cur_eid = 0

    for ts, st in events:
        st = bool(st)
        if st and (not in_search):
            in_search = True
            cur_start = float(ts)
            cur_eid += 1
        elif (not st) and in_search:
            episodes.append({
                "episode_id": cur_eid,
                "start": float(cur_start),
                "end": float(ts),
            })
            in_search = False
            cur_start = None

    if in_search and cur_start is not None:
        episodes.append({
            "episode_id": cur_eid,
            "start": float(cur_start),
            "end": np.inf,
        })
    return episodes


def locate_episode(episodes, ts):
    # return (episode_id, start_ts) if ts in any [start, end), else None
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


def robust_cholesky(cov):
    # cov should be symmetric; add jitter if needed
    eye = np.eye(3, dtype=np.float64)
    sym = 0.5 * (cov + cov.T)
    jitter = 1e-8
    for _ in range(8):
        try:
            return np.linalg.cholesky(sym + jitter * eye)
        except np.linalg.LinAlgError:
            jitter *= 10.0
    d = np.maximum(np.diag(sym), 1e-6)
    return np.linalg.cholesky(np.diag(d))


def decode_search_gmm_3d(msg):
    c_num = int(msg.num_components)
    dim = int(msg.state_dim) if int(msg.state_dim) > 0 else 6
    if c_num <= 0 or dim < 3:
        return None
    if len(msg.zeta_alpha) < c_num:
        return None
    if len(msg.zeta_a) < c_num * dim:
        return None
    if len(msg.zeta_b) < c_num * dim * dim:
        return None

    alphas = np.asarray(msg.zeta_alpha[:c_num], dtype=np.float64)
    valid_idx = [i for i, a in enumerate(alphas) if np.isfinite(a) and a > 1e-12]
    if not valid_idx:
        return None

    weights = []
    means = []
    chols = []

    for i in valid_idx:
        alpha = float(alphas[i])
        a0 = i * dim
        a1 = (i + 1) * dim
        b0 = i * dim * dim
        b1 = (i + 1) * dim * dim

        a_vec = np.asarray(msg.zeta_a[a0:a1], dtype=np.float64)
        b_mat = np.asarray(msg.zeta_b[b0:b1], dtype=np.float64).reshape(dim, dim)

        if not np.all(np.isfinite(a_vec)) or not np.all(np.isfinite(b_mat)):
            continue

        mu = a_vec / alpha
        cov = b_mat / alpha
        mu3 = mu[:3]
        cov3 = cov[:3, :3]

        if not np.all(np.isfinite(mu3)) or not np.all(np.isfinite(cov3)):
            continue

        l_mat = robust_cholesky(cov3)
        weights.append(alpha)
        means.append(mu3)
        chols.append(l_mat)

    if not weights:
        return None

    weights = np.asarray(weights, dtype=np.float64)
    wsum = float(weights.sum())
    if wsum <= 1e-18:
        return None
    weights /= wsum

    return {
        "weights": weights,
        "means": np.asarray(means, dtype=np.float64),
        "chols": chols,
    }


def sample_from_gmm(gmm, n_samples, rng):
    weights = gmm["weights"]
    means = gmm["means"]
    chols = gmm["chols"]
    k_num = weights.shape[0]
    comp_idx = rng.choice(k_num, size=n_samples, p=weights)
    out = np.empty((n_samples, 3), dtype=np.float64)

    for k in range(k_num):
        sel = np.where(comp_idx == k)[0]
        if sel.size == 0:
            continue
        noise = rng.standard_normal((sel.size, 3))
        out[sel, :] = means[k] + noise @ chols[k].T
    return out


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


def decode_particle_cloud_3d(msg):
    # search_particles_vis fields: x, y, z, intensity(weight normalized by w_max)
    field_names = [f.name for f in msg.fields]
    if not (("x" in field_names) and ("y" in field_names) and ("z" in field_names)):
        return None
    has_i = ("intensity" in field_names)
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
    return {
        "points": xyz,
        "weights": w,
    }


def evaluate_metrics_from_particles(particle_dist, y_vec, hit_radius, hpd_alpha, hpd_voxel_size):
    pts = particle_dist["points"]
    w = particle_dist["weights"]
    if pts.size == 0 or w.size == 0:
        return None
    y = np.asarray(y_vec, dtype=np.float64).reshape(1, 3)
    d = np.linalg.norm(pts - y, axis=1)

    # ES for empirical weighted distribution: second term estimated by pairwise matrix.
    t1 = float(np.sum(w * d))
    avg_dev = t1
    pair = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    t2 = float(np.sum((w[:, None] * w[None, :]) * pair))
    es = float(t1 - 0.5 * t2)

    if hit_radius > 0.0:
        hit_prob = float(np.sum(w * (d <= float(hit_radius)).astype(np.float64)))
    else:
        hit_prob = 0.0

    # Voxel-HPD: aggregate weighted particles into voxels, sort voxels by mass,
    # keep top-alpha cumulative mass voxels, and test whether GT voxel is included.
    alpha = float(np.clip(hpd_alpha, 0.0, 1.0))
    vox = float(hpd_voxel_size)
    if alpha <= 0.0 or alpha >= 1.0:
        hpd_cover = 1.0
    elif not (vox > 0.0):
        hpd_cover = 0.0
    else:
        # voxel id: integer tuple in world-aligned grid
        vxyz = np.floor(pts / vox).astype(np.int64)
        mass_map = {}
        for i in range(vxyz.shape[0]):
            key = (int(vxyz[i, 0]), int(vxyz[i, 1]), int(vxyz[i, 2]))
            mass_map[key] = mass_map.get(key, 0.0) + float(w[i])
        if not mass_map:
            hpd_cover = 0.0
        else:
            items = sorted(mass_map.items(), key=lambda kv: kv[1], reverse=True)
            keep = set()
            cum = 0.0
            for key, mass in items:
                keep.add(key)
                cum += mass
                if cum >= alpha:
                    break
            y_key = tuple(np.floor(y.reshape(3) / vox).astype(np.int64).tolist())
            hpd_cover = 1.0 if y_key in keep else 0.0

    return es, hit_prob, hpd_cover, avg_dev


def logsumexp(a, axis=None):
    arr = np.asarray(a, dtype=np.float64)
    amax = np.max(arr, axis=axis, keepdims=True)
    shifted = arr - amax
    out = amax + np.log(np.sum(np.exp(shifted), axis=axis, keepdims=True))
    if axis is None:
        return float(out.reshape(-1)[0])
    return np.squeeze(out, axis=axis)


def gmm_logpdf_points(gmm, points):
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, 3)
    n_pts = pts.shape[0]
    if n_pts == 0:
        return np.zeros((0,), dtype=np.float64)

    weights = gmm["weights"]
    means = gmm["means"]
    chols = gmm["chols"]
    k_num = int(weights.shape[0])

    log_comp = np.empty((k_num, n_pts), dtype=np.float64)
    d = 3.0
    c0 = -0.5 * d * np.log(2.0 * np.pi)

    for k in range(k_num):
        l_mat = chols[k]
        mu = means[k]
        # diff: [3, N]
        diff = (pts - mu.reshape(1, 3)).T
        try:
            sol = np.linalg.solve(l_mat, diff)
        except np.linalg.LinAlgError:
            # very rare fallback
            cov = l_mat @ l_mat.T + np.eye(3, dtype=np.float64) * 1e-8
            sol = np.linalg.solve(np.linalg.cholesky(cov), diff)
        q = np.sum(sol * sol, axis=0)
        diag_l = np.clip(np.diag(l_mat), 1e-12, None)
        log_det_half = np.sum(np.log(diag_l))
        log_comp[k, :] = np.log(max(float(weights[k]), 1e-300)) + c0 - log_det_half - 0.5 * q

    return np.asarray(logsumexp(log_comp, axis=0), dtype=np.float64)


def hpd_coverage_from_gmm(gmm, y_vec, alpha, n_samples, rng):
    alpha = float(alpha)
    if alpha <= 0.0:
        return 1.0
    if alpha >= 1.0:
        return 1.0
    n_samples = max(16, int(n_samples))

    x = sample_from_gmm(gmm, n_samples, rng)
    logp_x = gmm_logpdf_points(gmm, x)
    # HPD threshold: retain top-alpha mass => density >= q_{1-alpha}(p(X))
    thr = float(np.percentile(logp_x, 100.0 * (1.0 - alpha)))
    logp_y = float(gmm_logpdf_points(gmm, np.asarray(y_vec, dtype=np.float64).reshape(1, 3))[0])
    return 1.0 if logp_y >= thr else 0.0


def evaluate_metrics_from_gmm(gmm, y_vec, n_samples, hit_radius, hpd_alpha, hpd_samples, rng):
    n_samples = max(2, int(n_samples))
    x = sample_from_gmm(gmm, n_samples, rng)
    xp = sample_from_gmm(gmm, n_samples, rng)
    diff = x - y_vec.reshape(1, 3)
    dist = np.linalg.norm(diff, axis=1)

    t1 = dist.mean()
    avg_dev = float(t1)
    t2 = np.linalg.norm(x - xp, axis=1).mean()
    es = float(t1 - 0.5 * t2)

    if hit_radius > 0.0:
        hit_prob = float(np.mean(dist <= float(hit_radius)))
    else:
        hit_prob = 0.0

    hpd_cover = hpd_coverage_from_gmm(gmm, y_vec, hpd_alpha, hpd_samples, rng)
    return es, hit_prob, hpd_cover, avg_dev


def compute_bag_es(
    bag_path,
    selected_drone_ids,
    n_samples,
    max_gt_dt,
    hit_radius,
    hpd_alpha,
    hpd_samples,
    hpd_voxel_size,
    metric_source,
    rng,
):
    with rosbag.Bag(bag_path, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        auto_ids = detect_drone_ids(topic_info)
        if selected_drone_ids:
            drone_ids = [d for d in selected_drone_ids if d in auto_ids]
        else:
            drone_ids = auto_ids
        has_particle_topic = {
            d: (PARTICLE_TOPIC_FMT.format(d=d) in topic_info) for d in drone_ids
        }

        if not drone_ids:
            return {
                "bag_path": bag_path,
                "drone_ids": [],
                "per_drone": {},
                "all_es": [],
                "all_hit_prob": [],
                "all_hpd_cover": [],
                "all_avg_dev": [],
                "curve_samples": [],
            }

        # Pass 1: target trajectory
        target_t = []
        target_xyz = []
        for _, msg, t in bag.read_messages(topics=[TARGET_TOPIC]):
            p = msg.pose.pose.position
            target_t.append(float(t.to_sec()))
            target_xyz.append((float(p.x), float(p.y), float(p.z)))

        if not target_t:
            return {
                "bag_path": bag_path,
                "drone_ids": drone_ids,
                "per_drone": {
                    d: {
                        "n_episodes": 0,
                        "n_gmm": 0, "n_particle": 0,
                        "n_search": 0, "n_used": 0, "n_skip_no_gt": 0,
                        "n_skip_bad_gmm": 0, "n_skip_bad_particle": 0, "n_skip_not_search": 0,
                        "n_fallback_to_gmm": 0,
                        "es_values": [], "hit_prob_values": [], "hpd_cover_values": [], "avg_dev_values": []
                    } for d in drone_ids
                },
                "all_es": [],
                "all_hit_prob": [],
                "all_hpd_cover": [],
                "all_avg_dev": [],
                "curve_samples": [],
            }

        target_t = np.asarray(target_t, dtype=np.float64)
        target_xyz = np.asarray(target_xyz, dtype=np.float64)

        # Pass 2: read search_state events and build strict search episodes
        state_topics = [STATE_TOPIC_FMT.format(d=d) for d in drone_ids]
        state_events = {d: [] for d in drone_ids}
        for topic, msg, t in bag.read_messages(topics=state_topics):
            ts = float(t.to_sec())
            for d in drone_ids:
                if topic == STATE_TOPIC_FMT.format(d=d):
                    state_events[d].append((ts, bool(msg.data)))
                    break
        episodes_by_drone = {
            d: build_search_episodes_from_events(state_events[d]) for d in drone_ids
        }

        # Pass 3: particle/gmm samples (strictly filter by search episodes)
        metrics = {}
        topics = []
        for d in drone_ids:
            metrics[d] = {
                "n_episodes": len(episodes_by_drone.get(d, [])),
                "n_gmm": 0,
                "n_particle": 0,
                "n_search": 0,
                "n_used": 0,
                "n_skip_no_gt": 0,
                "n_skip_bad_gmm": 0,
                "n_skip_bad_particle": 0,
                "n_skip_not_search": 0,
                "n_fallback_to_gmm": 0,
                "es_values": [],
                "hit_prob_values": [],
                "hpd_cover_values": [],
                "avg_dev_values": [],
            }
            if metric_source == "particles":
                topics.append(PARTICLE_TOPIC_FMT.format(d=d))
            elif metric_source == "gmm":
                topics.append(GMM_TOPIC_FMT.format(d=d))
            else:
                # auto: per-drone prefer particles if available, else gmm
                if has_particle_topic.get(d, False):
                    topics.append(PARTICLE_TOPIC_FMT.format(d=d))
                else:
                    topics.append(GMM_TOPIC_FMT.format(d=d))

        curve_samples = []

        for topic, msg, t in bag.read_messages(topics=topics):
            ts = float(t.to_sec())
            d_hit = None
            src = ""
            for d in drone_ids:
                if topic == PARTICLE_TOPIC_FMT.format(d=d):
                    d_hit = d
                    src = "particle"
                    break
                if topic == GMM_TOPIC_FMT.format(d=d):
                    d_hit = d
                    src = "gmm"
                    break
            if d_hit is None:
                continue

            m = metrics[d_hit]
            if src == "particle":
                m["n_particle"] += 1
            else:
                m["n_gmm"] += 1

            ep = locate_episode(episodes_by_drone.get(d_hit, []), ts)
            if ep is None:
                m["n_skip_not_search"] += 1
                continue
            ep_id, ep_start = ep
            m["n_search"] += 1

            y = nearest_target_pos(target_t, target_xyz, ts, max_gt_dt)
            if y is None:
                m["n_skip_no_gt"] += 1
                continue

            if metric_source == "particles":
                if src != "particle":
                    continue
                pd = decode_particle_cloud_3d(msg)
                if pd is None:
                    m["n_skip_bad_particle"] += 1
                    continue
                out = evaluate_metrics_from_particles(
                    particle_dist=pd,
                    y_vec=y,
                    hit_radius=hit_radius,
                    hpd_alpha=hpd_alpha,
                    hpd_voxel_size=hpd_voxel_size,
                )
                if out is None:
                    m["n_skip_bad_particle"] += 1
                    continue
                es, hit_prob, hpd_cover, avg_dev = out
            elif metric_source == "gmm":
                if src != "gmm":
                    continue
                gmm = decode_search_gmm_3d(msg)
                if gmm is None:
                    m["n_skip_bad_gmm"] += 1
                    continue
                es, hit_prob, hpd_cover, avg_dev = evaluate_metrics_from_gmm(
                    gmm=gmm,
                    y_vec=y,
                    n_samples=n_samples,
                    hit_radius=hit_radius,
                    hpd_alpha=hpd_alpha,
                    hpd_samples=hpd_samples,
                    rng=rng,
                )
            else:  # auto
                if src == "particle":
                    pd = decode_particle_cloud_3d(msg)
                    if pd is None:
                        m["n_skip_bad_particle"] += 1
                        continue
                    out = evaluate_metrics_from_particles(
                        particle_dist=pd,
                        y_vec=y,
                        hit_radius=hit_radius,
                        hpd_alpha=hpd_alpha,
                        hpd_voxel_size=hpd_voxel_size,
                    )
                    if out is None:
                        m["n_skip_bad_particle"] += 1
                        continue
                    es, hit_prob, hpd_cover, avg_dev = out
                elif src == "gmm":
                    gmm = decode_search_gmm_3d(msg)
                    if gmm is None:
                        m["n_skip_bad_gmm"] += 1
                        continue
                    es, hit_prob, hpd_cover, avg_dev = evaluate_metrics_from_gmm(
                        gmm=gmm,
                        y_vec=y,
                        n_samples=n_samples,
                        hit_radius=hit_radius,
                        hpd_alpha=hpd_alpha,
                        hpd_samples=hpd_samples,
                        rng=rng,
                    )
                    m["n_fallback_to_gmm"] += 1
                else:
                    continue

            m["es_values"].append(es)
            m["hit_prob_values"].append(hit_prob)
            m["hpd_cover_values"].append(hpd_cover)
            m["avg_dev_values"].append(avg_dev)
            m["n_used"] += 1

            tau = ts - ep_start
            if tau < 0.0:
                tau = 0.0
            curve_samples.append({
                "bag_path": bag_path,
                "drone_id": d_hit,
                "episode_id": ep_id,
                "t_since_loss_sec": tau,
                "es": es,
                "avg_dev": avg_dev,
                "hit_prob": hit_prob,
                "hpd_cover": hpd_cover,
                "source": src,
            })

        all_es = []
        all_hit = []
        all_hpd = []
        all_avg_dev = []
        for d in drone_ids:
            all_es.extend(metrics[d]["es_values"])
            all_hit.extend(metrics[d]["hit_prob_values"])
            all_hpd.extend(metrics[d]["hpd_cover_values"])
            all_avg_dev.extend(metrics[d]["avg_dev_values"])

        return {
            "bag_path": bag_path,
            "drone_ids": drone_ids,
            "per_drone": metrics,
            "all_es": all_es,
            "all_hit_prob": all_hit,
            "all_hpd_cover": all_hpd,
            "all_avg_dev": all_avg_dev,
            "curve_samples": curve_samples,
        }


def summarize_bag_result(bag_result):
    rows = []
    bag_path = bag_result["bag_path"]
    for d in bag_result["drone_ids"]:
        m = bag_result["per_drone"][d]
        values = m["es_values"]
        row = {
            "drone_id": d,
            "bag_path": bag_path,
            "n_episodes": m.get("n_episodes", 0),
            "n_gmm": m["n_gmm"],
            "n_particle": m.get("n_particle", 0),
            "n_search": m["n_search"],
            "n_used": m["n_used"],
            "n_skip_no_gt": m["n_skip_no_gt"],
            "n_skip_bad_gmm": m["n_skip_bad_gmm"],
            "n_skip_bad_particle": m.get("n_skip_bad_particle", 0),
            "n_skip_not_search": m["n_skip_not_search"],
            "n_fallback_to_gmm": m.get("n_fallback_to_gmm", 0),
            "es_mean": mean_or_zero(values),
            "es_median": percentile(values, 50),
            "es_p90": percentile(values, 90),
            "es_p95": percentile(values, 95),
            "hit_prob_mean": mean_or_zero(m["hit_prob_values"]),
            "hit_prob_median": percentile(m["hit_prob_values"], 50),
            "hit_prob_p10": percentile(m["hit_prob_values"], 10),
            "hit_prob_p90": percentile(m["hit_prob_values"], 90),
            "hpd_coverage_mean": mean_or_zero(m["hpd_cover_values"]),
            "avg_dev_mean": mean_or_zero(m["avg_dev_values"]),
            "avg_dev_median": percentile(m["avg_dev_values"], 50),
            "avg_dev_p90": percentile(m["avg_dev_values"], 90),
            "avg_dev_p95": percentile(m["avg_dev_values"], 95),
        }
        rows.append(row)
    return rows


def print_bag_summary(run_tag, rows, all_es_values, all_hit_values, all_hpd_values, all_avg_dev_values):
    print("\n=== {} ===".format(run_tag))
    if not rows:
        print("no valid drone data")
        return
    for r in rows:
        print(
            "drone{d}: episodes={e}, ES mean={m:.4f}, p95={p95:.4f}, AD mean={ad:.4f}, Hit mean={hit:.3f}, HPDcov mean={hpd:.3f}, used={u}/{s} (particle={p}, gmm={g}, fb_gmm={fbg})".format(
                d=r["drone_id"], m=r["es_mean"], p95=r["es_p95"],
                e=r.get("n_episodes", 0),
                ad=r["avg_dev_mean"],
                hit=r["hit_prob_mean"], hpd=r["hpd_coverage_mean"],
                u=r["n_used"], s=r["n_search"],
                p=r.get("n_particle", 0), g=r["n_gmm"], fbg=r.get("n_fallback_to_gmm", 0))
        )
    if all_es_values:
        print(
            "aggregate(within bag, weighted by samples): ES mean={:.4f}, ES median={:.4f}, ES p95={:.4f}, AD mean={:.4f}, Hit mean={:.3f}, HPDcov mean={:.3f}".format(
                mean_or_zero(all_es_values),
                percentile(all_es_values, 50),
                percentile(all_es_values, 95),
                mean_or_zero(all_avg_dev_values),
                mean_or_zero(all_hit_values),
                mean_or_zero(all_hpd_values),
            )
        )


def build_curve_binned_rows(samples, bin_sec, max_sec, per_drone):
    if bin_sec <= 0.0:
        raise ValueError("curve bin_sec must be > 0")

    buckets = {}
    for s in samples:
        tau = float(s["t_since_loss_sec"])
        if tau < 0.0:
            continue
        if max_sec > 0.0 and tau > max_sec:
            continue
        bin_idx = int(np.floor(tau / bin_sec))
        d_key = int(s["drone_id"]) if per_drone else -1
        bag_key = str(s.get("bag_path", ""))
        key = (d_key, bin_idx)
        if key not in buckets:
            buckets[key] = {}
        if bag_key not in buckets[key]:
            buckets[key][bag_key] = {
                "es": [],
                "hit_prob": [],
                "hpd_cover": [],
                "avg_dev": [],
            }
        buckets[key][bag_key]["es"].append(float(s["es"]))
        if "hit_prob" in s:
            buckets[key][bag_key]["hit_prob"].append(float(s["hit_prob"]))
        if "hpd_cover" in s:
            buckets[key][bag_key]["hpd_cover"].append(float(s["hpd_cover"]))
        if "avg_dev" in s:
            buckets[key][bag_key]["avg_dev"].append(float(s["avg_dev"]))

    rows = []
    for (d_key, bin_idx), by_bag in sorted(buckets.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        # 每个bag在该时间桶内先求均值，再跨bag等权统计
        es_vals = [mean_or_zero(v["es"]) for v in by_bag.values() if v["es"]]
        hit_vals = [mean_or_zero(v["hit_prob"]) for v in by_bag.values() if v["hit_prob"]]
        hpd_vals = [mean_or_zero(v["hpd_cover"]) for v in by_bag.values() if v["hpd_cover"]]
        avg_dev_vals = [mean_or_zero(v["avg_dev"]) for v in by_bag.values() if v["avg_dev"]]
        n_samples = int(sum(len(v["es"]) for v in by_bag.values()))
        n_bags = len(es_vals)
        left = bin_idx * bin_sec
        right = left + bin_sec
        center = left + 0.5 * bin_sec
        rows.append({
            "drone_id": d_key if per_drone else "",
            "bin_idx": bin_idx,
            "t_left_sec": left,
            "t_center_sec": center,
            "t_right_sec": right,
            "n": n_bags,
            "n_samples": n_samples,
            "n_bags": n_bags,
            "es_mean": mean_or_zero(es_vals),
            "es_median": percentile(es_vals, 50),
            "es_p10": percentile(es_vals, 10),
            "es_p25": percentile(es_vals, 25),
            "es_p75": percentile(es_vals, 75),
            "es_p90": percentile(es_vals, 90),
            "es_p95": percentile(es_vals, 95),
            "avg_dev_mean": mean_or_zero(avg_dev_vals),
            "avg_dev_median": percentile(avg_dev_vals, 50),
            "avg_dev_p10": percentile(avg_dev_vals, 10),
            "avg_dev_p25": percentile(avg_dev_vals, 25),
            "avg_dev_p75": percentile(avg_dev_vals, 75),
            "avg_dev_p90": percentile(avg_dev_vals, 90),
            "avg_dev_p95": percentile(avg_dev_vals, 95),
            "hit_prob_mean": mean_or_zero(hit_vals),
            "hit_prob_p10": percentile(hit_vals, 10),
            "hit_prob_p25": percentile(hit_vals, 25),
            "hit_prob_p75": percentile(hit_vals, 75),
            "hit_prob_p90": percentile(hit_vals, 90),
            "hpd_coverage_mean": mean_or_zero(hpd_vals),
            "hpd_coverage_p10": percentile(hpd_vals, 10),
            "hpd_coverage_p25": percentile(hpd_vals, 25),
            "hpd_coverage_p75": percentile(hpd_vals, 75),
            "hpd_coverage_p90": percentile(hpd_vals, 90),
        })
    return rows


def maybe_plot_curve(rows, per_drone, png_path):
    if not png_path:
        return
    if not rows:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("Warning: matplotlib not available, skip curve plot.")
        return

    ensure_parent_dir(png_path)

    metric_defs = [
        {
            "key_mean": "es_mean",
            "key_p25": "es_p25",
            "key_p75": "es_p75",
            "ylabel": "Energy Score (m)",
            "title": "ES Curve (lower is better)",
            "ylim": None,
            "color": "#2c7fb8",
        },
        {
            "key_mean": "avg_dev_mean",
            "key_p25": "avg_dev_p25",
            "key_p75": "avg_dev_p75",
            "ylabel": "Avg Deviation (m)",
            "title": "Average Deviation Curve (lower is better)",
            "ylim": None,
            "color": "#f16913",
        },
        {
            "key_mean": "hit_prob_mean",
            "key_p25": "hit_prob_p25",
            "key_p75": "hit_prob_p75",
            "ylabel": "Hit Probability",
            "title": "Hit Probability Curve",
            "ylim": (-0.02, 1.02),
            "color": "#2ca25f",
        },
        {
            "key_mean": "hpd_coverage_mean",
            "key_p25": "hpd_coverage_p25",
            "key_p75": "hpd_coverage_p75",
            "ylabel": "HPD Coverage",
            "title": "HPD Coverage Curve",
            "ylim": (-0.02, 1.02),
            "color": "#dd1c77",
        },
    ]

    fig, axes = plt.subplots(4, 1, figsize=(8.8, 12.8), sharex=True)

    by_drone = {}
    if per_drone:
        for r in rows:
            d = int(r["drone_id"])
            by_drone.setdefault(d, []).append(r)

    for ax, md in zip(axes, metric_defs):
        if per_drone:
            for d, rs in sorted(by_drone.items()):
                rs = sorted(rs, key=lambda x: x["t_center_sec"])
                x = np.asarray([r["t_center_sec"] for r in rs], dtype=np.float64)
                y = np.asarray([r.get(md["key_mean"], 0.0) for r in rs], dtype=np.float64)
                p25 = np.asarray([r.get(md["key_p25"], 0.0) for r in rs], dtype=np.float64)
                p75 = np.asarray([r.get(md["key_p75"], 0.0) for r in rs], dtype=np.float64)
                ax.plot(x, y, lw=1.8, label="drone{}".format(d))
                ax.fill_between(x, p25, p75, alpha=0.15)
        else:
            rs = sorted(rows, key=lambda x: x["t_center_sec"])
            x = np.asarray([r["t_center_sec"] for r in rs], dtype=np.float64)
            y = np.asarray([r.get(md["key_mean"], 0.0) for r in rs], dtype=np.float64)
            p25 = np.asarray([r.get(md["key_p25"], 0.0) for r in rs], dtype=np.float64)
            p75 = np.asarray([r.get(md["key_p75"], 0.0) for r in rs], dtype=np.float64)
            ax.plot(x, y, lw=2.2, color=md["color"], label="all drones")
            ax.fill_between(x, p25, p75, color=md["color"], alpha=0.20, label="p25-p75")

        if md["ylim"] is not None:
            ax.set_ylim(md["ylim"])
        ax.set_ylabel(md["ylabel"])
        ax.set_title(md["title"])
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend(loc="best")

    axes[-1].set_xlabel("Time Since Loss Start (s)")
    fig.suptitle("Search Estimation Curves")
    fig.tight_layout()
    fig.savefig(png_path, dpi=180)
    plt.close(fig)


def main():
    args = parse_args()
    if not (0.0 < float(args.hpd_alpha) < 1.0):
        print("Invalid --hpd-alpha: must be in (0, 1)")
        return 1
    if int(args.hpd_samples) <= 0:
        print("Invalid --hpd-samples: must be > 0")
        return 1
    if float(args.hpd_voxel_size) <= 0.0:
        print("Invalid --hpd-voxel-size: must be > 0")
        return 1
    if float(args.hit_radius) <= 0.0:
        print("Invalid --hit-radius: must be > 0")
        return 1

    jobs, missing = collect_bag_jobs(args)
    if missing:
        print("Bag resolve: {} found, {} missing.".format(len(jobs), len(missing)))
        for m in missing[:8]:
            tried = m.get("tried", [])
            tried_text = "; ".join(tried[:3]) if tried else "(none)"
            print("  missing run_id={} raw='{}' tried: {}".format(
                m.get("run_id", ""), m.get("raw_bag", ""), tried_text))
        if len(missing) > 8:
            print("  ... {} more missing entries".format(len(missing) - 8))
    if not jobs:
        print("No bag files found. Check summary.csv bag_path or restore tracking.bag files.")
        return 1

    selected_ids = parse_drone_ids_arg(args.drone_ids)
    rng = np.random.default_rng(args.seed)

    out_rows = []
    bag_level_global = []
    curve_samples_all = []

    for i, job in enumerate(jobs, start=1):
        bag_path = job["bag_path"]
        run_id = job.get("run_id", "")
        run_tag = "run {} ({}/{})".format(run_id or i, i, len(jobs))
        print("[{}] processing {}".format(i, bag_path))

        bag_result = compute_bag_es(
            bag_path=bag_path,
            selected_drone_ids=selected_ids,
            n_samples=args.mc_samples,
            max_gt_dt=args.max_gt_dt,
            hit_radius=args.hit_radius,
            hpd_alpha=args.hpd_alpha,
            hpd_samples=args.hpd_samples,
            hpd_voxel_size=args.hpd_voxel_size,
            metric_source=args.metric_source,
            rng=rng,
        )
        rows = summarize_bag_result(bag_result)
        print_bag_summary(
            run_tag,
            rows,
            bag_result["all_es"],
            bag_result["all_hit_prob"],
            bag_result["all_hpd_cover"],
            bag_result["all_avg_dev"],
        )

        for r in rows:
            r2 = dict(r)
            r2["run_id"] = run_id if run_id else str(i)
            r2["status"] = job.get("status", "")
            out_rows.append(r2)
        if bag_result["all_es"]:
            bag_level_global.append({
                "es_mean": mean_or_zero(bag_result["all_es"]),
                "hit_mean": mean_or_zero(bag_result["all_hit_prob"]),
                "hpd_mean": mean_or_zero(bag_result["all_hpd_cover"]),
                "avg_dev_mean": mean_or_zero(bag_result["all_avg_dev"]),
            })
        for s in bag_result["curve_samples"]:
            s2 = dict(s)
            s2["run_id"] = run_id if run_id else str(i)
            s2["status"] = job.get("status", "")
            curve_samples_all.append(s2)

    if bag_level_global:
        es_bag = [b["es_mean"] for b in bag_level_global]
        hit_bag = [b["hit_mean"] for b in bag_level_global]
        hpd_bag = [b["hpd_mean"] for b in bag_level_global]
        avg_dev_bag = [b["avg_dev_mean"] for b in bag_level_global]
        print("\n=== Overall ===")
        print("ES mean={:.4f}, median={:.4f}, p95={:.4f}, AD mean={:.4f}, Hit mean={:.3f}, HPDcov mean={:.3f}, bag_equal_n={}".format(
            mean_or_zero(es_bag),
            percentile(es_bag, 50),
            percentile(es_bag, 95),
            mean_or_zero(avg_dev_bag),
            mean_or_zero(hit_bag),
            mean_or_zero(hpd_bag),
            len(es_bag),
        ))

    if args.out_csv:
        out_csv = os.path.abspath(args.out_csv)
        ensure_parent_dir(out_csv)
        fields = [
            "run_id", "status", "bag_path", "drone_id",
            "n_episodes",
            "n_gmm", "n_particle", "n_fallback_to_gmm",
            "n_search", "n_used",
            "n_skip_no_gt", "n_skip_bad_gmm", "n_skip_bad_particle", "n_skip_not_search",
            "es_mean", "es_median", "es_p90", "es_p95",
            "avg_dev_mean", "avg_dev_median", "avg_dev_p90", "avg_dev_p95",
            "hit_prob_mean", "hit_prob_median", "hit_prob_p10", "hit_prob_p90",
            "hpd_coverage_mean",
        ]
        with open(out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in out_rows:
                writer.writerow(r)
        print("\nWrote: {}".format(out_csv))

    if args.curve_samples_csv:
        curve_samples_csv = os.path.abspath(args.curve_samples_csv)
        ensure_parent_dir(curve_samples_csv)
        fields = [
            "run_id", "status", "bag_path", "drone_id", "episode_id",
            "t_since_loss_sec", "es", "avg_dev", "hit_prob", "hpd_cover", "source",
        ]
        with open(curve_samples_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in curve_samples_all:
                writer.writerow(r)
        print("Wrote: {}".format(curve_samples_csv))

    curve_rows = []
    need_curve = bool(args.curve_binned_csv or args.curve_png)
    if need_curve:
        curve_rows = build_curve_binned_rows(
            samples=curve_samples_all,
            bin_sec=args.curve_bin_sec,
            max_sec=args.curve_max_sec,
            per_drone=args.curve_per_drone,
        )
    if args.curve_binned_csv:
        curve_binned_csv = os.path.abspath(args.curve_binned_csv)
        ensure_parent_dir(curve_binned_csv)
        fields = [
            "drone_id", "bin_idx", "t_left_sec", "t_center_sec", "t_right_sec",
            "n", "n_samples", "n_bags",
            "es_mean", "es_median", "es_p10", "es_p25", "es_p75", "es_p90", "es_p95",
            "avg_dev_mean", "avg_dev_median", "avg_dev_p10", "avg_dev_p25", "avg_dev_p75", "avg_dev_p90", "avg_dev_p95",
            "hit_prob_mean", "hit_prob_p10", "hit_prob_p25", "hit_prob_p75", "hit_prob_p90",
            "hpd_coverage_mean", "hpd_coverage_p10", "hpd_coverage_p25", "hpd_coverage_p75", "hpd_coverage_p90",
        ]
        with open(curve_binned_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in curve_rows:
                writer.writerow(r)
        print("Wrote: {}".format(curve_binned_csv))

    if args.curve_png:
        maybe_plot_curve(curve_rows, args.curve_per_drone, os.path.abspath(args.curve_png))
        print("Wrote: {}".format(os.path.abspath(args.curve_png)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
