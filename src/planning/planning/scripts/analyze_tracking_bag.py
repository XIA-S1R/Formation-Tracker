#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import defaultdict

import rosbag
import sensor_msgs.point_cloud2 as pc2


def percentile(values, q):
    if not values:
        return 0.0
    if len(values) == 1:
        return float(values[0])
    vals = sorted(values)
    idx = (len(vals) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(vals[lo])
    w = idx - lo
    return float(vals[lo] * (1.0 - w) + vals[hi] * w)


def vec_norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def pairwise_dist(a, b):
    return vec_norm3(a[0] - b[0], a[1] - b[1], a[2] - b[2])


def pairwise_dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return dx * dx + dy * dy + dz * dz


def mean_or_zero(values):
    return float(statistics.mean(values)) if values else 0.0


def build_voxel_index(points_xyz, voxel_size):
    index = defaultdict(list)
    if voxel_size <= 1e-6:
        return index
    inv = 1.0 / voxel_size
    for x, y, z in points_xyz:
        key = (
            int(math.floor(x * inv)),
            int(math.floor(y * inv)),
            int(math.floor(z * inv)),
        )
        index[key].append((x, y, z))
    return index


def query_min_dist_in_radius(pos, voxel_index, voxel_size, radius):
    if not voxel_index or voxel_size <= 1e-6 or radius <= 0.0:
        return None
    inv = 1.0 / voxel_size
    cx = int(math.floor(pos[0] * inv))
    cy = int(math.floor(pos[1] * inv))
    cz = int(math.floor(pos[2] * inv))
    rn = int(math.ceil(radius * inv))
    r2 = radius * radius
    best2 = None
    for ix in range(cx - rn, cx + rn + 1):
        for iy in range(cy - rn, cy + rn + 1):
            for iz in range(cz - rn, cz + rn + 1):
                pts = voxel_index.get((ix, iy, iz), ())
                for px, py, pz in pts:
                    dx = px - pos[0]
                    dy = py - pos[1]
                    dz = pz - pos[2]
                    d2 = dx * dx + dy * dy + dz * dz
                    if d2 <= r2 and (best2 is None or d2 < best2):
                        best2 = d2
    if best2 is None:
        return None
    return math.sqrt(best2)


def get_desired_formation_nodes(formation_type):
    # Keep consistent with traj_opt::TrajOpt::setDesiredFormation
    if formation_type == 1:
        # regular hexagon ring (6 drones)
        return [
            (1.7321, -1.0000, 0.0),
            (0.0000, -2.0000, 0.0),
            (-1.7321, -1.0000, 0.0),
            (-1.7321, 1.0000, 0.0),
            (0.0000, 2.0000, 0.0),
            (1.7321, 1.0000, 0.0),
        ]
    if formation_type == 2:
        # equilateral triangle (3 drones)
        return [
            (0.0, 1.1547, 0.0),
            (1.0, -0.5774, 0.0),
            (-1.0, -0.5774, 0.0),
        ]
    if formation_type == 3:
        # square (4 drones)
        return [
            (1.0, 1.0, 0.0),
            (1.0, -1.0, 0.0),
            (-1.0, -1.0, 0.0),
            (-1.0, 1.0, 0.0),
        ]
    if formation_type == 4:
        # regular pentagon (5 drones)
        return [
            (1.7013, 0.0000, 0.0),
            (0.5257, 1.6180, 0.0),
            (-1.3764, 1.0000, 0.0),
            (-1.3764, -1.0000, 0.0),
            (0.5257, -1.6180, 0.0),
        ]
    return []


def calc_snl_matrix(nodes):
    n = len(nodes)
    if n <= 0:
        return None

    A = [[0.0 for _ in range(n)] for __ in range(n)]
    D = [0.0 for _ in range(n)]

    for i in range(n):
        for j in range(n):
            A[i][j] = pairwise_dist2(nodes[i], nodes[j])
            D[i] += A[i][j]

    for di in D:
        if di <= 1e-12:
            return None

    Lhat = [[0.0 for _ in range(n)] for __ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                Lhat[i][j] = 1.0
            else:
                Lhat[i][j] = -A[i][j] / math.sqrt(D[i] * D[j])
    return Lhat


def calc_formation_graph_jf(nodes, desired_nodes):
    if len(nodes) != len(desired_nodes) or len(nodes) <= 0:
        return None
    L = calc_snl_matrix(nodes)
    Ld = calc_snl_matrix(desired_nodes)
    if L is None or Ld is None:
        return None

    n = len(nodes)
    jf = 0.0
    for i in range(n):
        for j in range(n):
            d = L[i][j] - Ld[i][j]
            jf += d * d
    return float(jf)


def parse_drone_id_from_topic(topic):
    # /drone0/odom
    if topic.startswith('/drone'):
        rest = topic[6:]
        num = ''
        for ch in rest:
            if ch.isdigit():
                num += ch
            else:
                break
        if num:
            return int(num)
    return None


def integrate_single_drone_visibility_from_search(events, t_max):
    """
    events: [(ts, search_state_bool), ...]
    visible := not search_state
    until_first_loss means:
      from first valid timestamp to the end of first loss episode
      (if never recovered from first loss, use t_max).
    """
    if not events:
        return {
            'visibility_rate_full': 0.0,
            'visibility_rate_until_first_loss': 0.0,
            'first_loss_time_sec': None,
            'valid_duration_full_sec': 0.0,
            'valid_duration_until_first_loss_sec': 0.0,
        }

    events = sorted(events, key=lambda x: x[0])
    start_t = events[0][0]
    last_t = start_t
    prev_search = bool(events[0][1])
    first_loss_start_t = start_t if prev_search else None

    visible_full = 0.0
    visible_until_first_loss = 0.0
    first_loss_end_t = None
    until_window_end_t = None

    for ts, cur_search in events[1:]:
        if ts < last_t:
            continue
        dt = ts - last_t
        if not prev_search:
            visible_full += dt
        # integrate until-first-loss window:
        # from start to first loss end (or t_max if unresolved).
        if until_window_end_t is None and first_loss_start_t is None and (not prev_search):
            visible_until_first_loss += dt

        if first_loss_start_t is None and (not prev_search) and bool(cur_search):
            first_loss_start_t = ts
        elif first_loss_start_t is not None and first_loss_end_t is None and prev_search and (not bool(cur_search)):
            first_loss_end_t = ts
            until_window_end_t = ts

        prev_search = bool(cur_search)
        last_t = ts

    if t_max >= last_t:
        dt = t_max - last_t
        if not prev_search:
            visible_full += dt
        if until_window_end_t is None and first_loss_start_t is None and (not prev_search):
            visible_until_first_loss += dt

    if until_window_end_t is None:
        # no loss or unresolved first loss both end at t_max
        until_window_end_t = t_max

    full_dur = max(0.0, t_max - start_t)
    until_loss_dur = max(0.0, until_window_end_t - start_t)

    # first_loss_time_sec keeps existing meaning: first loss start timestamp
    return {
        'visibility_rate_full': (visible_full / full_dur) if full_dur > 1e-6 else 0.0,
        'visibility_rate_until_first_loss': (
            (visible_until_first_loss / until_loss_dur) if until_loss_dur > 1e-6 else 0.0
        ),
        'first_loss_time_sec': first_loss_start_t,
        'first_loss_end_time_sec': first_loss_end_t,
        'valid_duration_full_sec': full_dur,
        'valid_duration_until_first_loss_sec': until_loss_dur,
    }


def integrate_single_drone_tracking_visibility(search_events, obs_events, t_max):
    """
    Integrate per-drone visibility while the drone is in tracking mode.
    tracking mode: search_state == False
    visibility: has_observation from local_stats; if unavailable, fallback to search_state-derived visibility.
    """
    if not search_events:
        return {
            'visibility_rate_tracking': 0.0,
            'tracking_duration_sec': 0.0,
            'valid_duration_sec': 0.0,
            'visible_duration_sec': 0.0,
            'source': 'none',
        }

    use_obs = bool(obs_events)
    events = [(ts, 'search', bool(st)) for ts, st in search_events]
    if use_obs:
        events.extend((ts, 'obs', bool(st)) for ts, st in obs_events)
    events.sort(key=lambda x: (x[0], 0 if x[1] == 'search' else 1))

    search_state = None
    obs_state = None
    last_t = None

    tracking_duration_sec = 0.0
    valid_duration_sec = 0.0
    visible_duration_sec = 0.0

    for ts, kind, st in events:
        if last_t is not None and ts >= last_t:
            dt = ts - last_t
            if search_state is not None and (not search_state):
                tracking_duration_sec += dt
                if use_obs:
                    if obs_state is not None:
                        valid_duration_sec += dt
                        if obs_state:
                            visible_duration_sec += dt
                else:
                    valid_duration_sec += dt
                    visible_duration_sec += dt

        if kind == 'search':
            search_state = st
        else:
            obs_state = st
        last_t = ts

    if last_t is not None and t_max >= last_t:
        dt = t_max - last_t
        if search_state is not None and (not search_state):
            tracking_duration_sec += dt
            if use_obs:
                if obs_state is not None:
                    valid_duration_sec += dt
                    if obs_state:
                        visible_duration_sec += dt
            else:
                valid_duration_sec += dt
                visible_duration_sec += dt

    return {
        'visibility_rate_tracking': (visible_duration_sec / valid_duration_sec) if valid_duration_sec > 1e-6 else 0.0,
        'tracking_duration_sec': tracking_duration_sec,
        'valid_duration_sec': valid_duration_sec,
        'visible_duration_sec': visible_duration_sec,
        'source': 'local_stats' if use_obs else 'search_state_fallback',
    }


def integrate_single_drone_visibility_over_total(search_events, obs_events, t_start, t_end):
    """
    Integrate per-drone visibility over full experiment duration:
      visibility_rate_total = visible_duration / (t_end - t_start)
    visible is from local_stats.has_observation when available,
    otherwise fallback to (not search_state).
    """
    total_dur = max(0.0, float(t_end) - float(t_start))
    if total_dur <= 1e-9:
        return {
            'visibility_rate_total': 0.0,
            'visible_duration_sec': 0.0,
            'total_duration_sec': total_dur,
            'source': 'none',
        }

    use_obs = bool(obs_events)
    if use_obs:
        events = sorted((float(ts), bool(st)) for ts, st in obs_events)
        visible_when_true = True
        source = 'local_stats'
    else:
        events = sorted((float(ts), bool(st)) for ts, st in search_events)
        visible_when_true = False  # search_state=False means visible
        source = 'search_state_fallback'

    if not events:
        return {
            'visibility_rate_total': 0.0,
            'visible_duration_sec': 0.0,
            'total_duration_sec': total_dur,
            'source': source,
        }

    # Seed with latest state at or before t_start.
    state = None
    idx_start = 0
    for i, (ts, st) in enumerate(events):
        if ts <= t_start:
            state = st
            idx_start = i + 1
        else:
            break

    visible_dur = 0.0
    last_t = float(t_start)

    for ts, st in events[idx_start:]:
        if ts < t_start:
            continue
        if ts > t_end:
            break
        dt = ts - last_t
        if dt > 0.0 and state is not None:
            visible = (state if visible_when_true else (not state))
            if visible:
                visible_dur += dt
        state = st
        last_t = ts

    if t_end > last_t and state is not None:
        dt = t_end - last_t
        visible = (state if visible_when_true else (not state))
        if visible:
            visible_dur += dt

    return {
        'visibility_rate_total': (visible_dur / total_dur) if total_dur > 1e-9 else 0.0,
        'visible_duration_sec': visible_dur,
        'total_duration_sec': total_dur,
        'source': source,
    }


def find_target_maneuver_start(target_samples, speed_thresh, dist_thresh, confirm_sec):
    """
    target_samples: [(ts, speed, (x,y,z)), ...]
    Return first timestamp considered as target maneuver start.
    Rule: speed >= speed_thresh AND displacement from initial position >= dist_thresh,
    and this state lasts for confirm_sec (time accumulated over continuous samples).
    """
    if not target_samples:
        return None

    samples = sorted(target_samples, key=lambda x: float(x[0]))
    p0 = samples[0][2]
    speed_thresh = max(0.0, float(speed_thresh))
    dist_thresh = max(0.0, float(dist_thresh))
    confirm_sec = max(0.0, float(confirm_sec))

    run_start = None
    run_dur = 0.0
    prev_ts = None

    for ts, speed, pos in samples:
        ts = float(ts)
        moved = pairwise_dist(pos, p0) >= dist_thresh
        moving = (float(speed) >= speed_thresh) and moved

        if moving:
            if run_start is None:
                run_start = ts
                run_dur = 0.0
            if prev_ts is not None and ts >= prev_ts:
                dt = ts - prev_ts
                # Discontinuous samples break continuity.
                if dt > 1.0:
                    run_start = ts
                    run_dur = 0.0
                else:
                    run_dur += dt
            if run_dur >= confirm_sec:
                return run_start
        else:
            run_start = None
            run_dur = 0.0

        prev_ts = ts

    # Fallback: first sample with enough displacement.
    for ts, speed, pos in samples:
        if pairwise_dist(pos, p0) >= dist_thresh:
            return float(ts)

    return None


def analyze_bag(
    bag_path,
    formation_side_length=2.0,
    formation_type=2,
    collision_distance=0.0,
    collision_release_distance=0.0,
    obstacle_collision_distance=0.0,
    obstacle_collision_release_distance=0.0,
    reacq_timeout_sec=10.0,
    hovering_fail_duration_sec=5.0,
    dyn_vmax_limit=3.0,
    dyn_amax_limit=6.0,
    speed_tail_trim_sec=11.0,
    target_motion_speed_thresh=0.2,
    target_motion_dist_thresh=0.5,
    target_motion_confirm_sec=1.0,
):
    collision_distance = max(0.0, float(collision_distance))
    collision_release_distance = max(collision_distance, float(collision_release_distance))
    obstacle_collision_distance = max(0.0, float(obstacle_collision_distance))
    obstacle_collision_release_distance = max(
        obstacle_collision_distance, float(obstacle_collision_release_distance)
    )
    hovering_fail_duration_sec = max(0.0, float(hovering_fail_duration_sec))
    speed_tail_trim_sec = max(0.0, float(speed_tail_trim_sec))

    drone_positions = {}  # drone_id -> (x,y,z)
    drone_speeds = defaultdict(list)
    drone_speeds_tracking = defaultdict(list)
    drone_speeds_search = defaultdict(list)
    drone_speed_samples = defaultdict(list)
    drone_speed_samples_tracking = defaultdict(list)
    drone_speed_samples_search = defaultdict(list)
    desired_nodes = get_desired_formation_nodes(int(formation_type))
    formation_jf_samples = []
    formation_jf_samples_tracking = []
    formation_jf_samples_search = []
    target_speed_samples = []  # [(ts, speed, (x,y,z))]
    drone_last_replan_state = {}
    drone_last_replan_ts = {}
    replan_failed_hovering_count = 0
    replan_hovering_total_duration_by_drone = defaultdict(float)
    replan_hovering_max_continuous_by_drone = defaultdict(float)
    hovering_episode_start_by_drone = {}
    emergency_stop_count = 0

    # search_state timeline
    search_state_now = {}
    search_state_events = []  # (t, drone_id, state)
    obs_state_now = {}
    obs_state_events = []  # (t, drone_id, has_observation)

    # odom-driven evaluation timeline
    eval_times = []
    formation_err_samples = []
    formation_err_samples_tracking = []
    formation_err_samples_search = []
    min_pair_dist_samples = []
    inter_drone_collision_count = 0
    inter_drone_collision_active = False
    # dynamics legality
    last_odom_state = {}  # drone_id -> (ts, vx, vy, vz, speed)
    dyn_total_dur_by_drone = defaultdict(float)
    dyn_speed_over_dur_by_drone = defaultdict(float)
    dyn_acc_over_dur_by_drone = defaultdict(float)
    dyn_any_over_dur_by_drone = defaultdict(float)

    # obstacle collision from /global_map
    obstacle_map_points_count = 0
    obstacle_voxel_index = {}
    obstacle_voxel_size = 0.0
    obstacle_query_radius = max(obstacle_collision_release_distance, obstacle_collision_distance)
    obstacle_collision_count = 0
    obstacle_collision_active = defaultdict(bool)  # drone_id -> active
    min_obstacle_dist_samples = []

    t_min = None
    t_max = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            ts = t.to_sec()
            if t_min is None or ts < t_min:
                t_min = ts
            if t_max is None or ts > t_max:
                t_max = ts

            # Static obstacle map
            if topic == '/global_map' and obstacle_query_radius > 0.0 and obstacle_map_points_count == 0:
                pts = []
                for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    pts.append((float(x), float(y), float(z)))
                obstacle_map_points_count = len(pts)
                if obstacle_map_points_count > 0:
                    # Spatial hash for fast nearest lookup in a local radius
                    obstacle_voxel_size = max(0.1, obstacle_query_radius * 0.5)
                    obstacle_voxel_index = build_voxel_index(pts, obstacle_voxel_size)
                continue

            # Target odom
            if topic == '/target/odom':
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                target_speed = vec_norm3(v.x, v.y, v.z)
                target_speed_samples.append((ts, target_speed, (float(p.x), float(p.y), float(p.z))))
                continue

            # Drone odom: /droneX/odom
            if topic.endswith('/odom') and topic.startswith('/drone') and topic.count('/') == 2:
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                speed = vec_norm3(v.x, v.y, v.z)
                drone_positions[drone_id] = (p.x, p.y, p.z)
                drone_speeds[drone_id].append(speed)
                drone_speed_samples[drone_id].append((ts, speed))

                # dynamics legality from odom (time weighted)
                prev = last_odom_state.get(drone_id, None)
                if prev is not None:
                    ts_prev, vx_prev, vy_prev, vz_prev, speed_prev = prev
                    dt = ts - ts_prev
                    if 1e-4 < dt < 1.0:
                        acc = vec_norm3((v.x - vx_prev) / dt, (v.y - vy_prev) / dt, (v.z - vz_prev) / dt)
                        speed_eval = max(speed, speed_prev)
                        speed_over = speed_eval > dyn_vmax_limit
                        acc_over = acc > dyn_amax_limit
                        any_over = speed_over or acc_over
                        dyn_total_dur_by_drone[drone_id] += dt
                        if speed_over:
                            dyn_speed_over_dur_by_drone[drone_id] += dt
                        if acc_over:
                            dyn_acc_over_dur_by_drone[drone_id] += dt
                        if any_over:
                            dyn_any_over_dur_by_drone[drone_id] += dt
                last_odom_state[drone_id] = (ts, v.x, v.y, v.z, speed)
                # 速度按本机模式拆分
                if drone_id in search_state_now:
                    if search_state_now[drone_id]:
                        drone_speeds_search[drone_id].append(speed)
                        drone_speed_samples_search[drone_id].append((ts, speed))
                    else:
                        drone_speeds_tracking[drone_id].append(speed)
                        drone_speed_samples_tracking[drone_id].append((ts, speed))

                # use every odom update as eval tick when all drones are available
                eval_times.append(ts)
                if len(drone_positions) >= 2:
                    ids = sorted(drone_positions.keys())
                    dists = []
                    for i in range(len(ids)):
                        for j in range(i + 1, len(ids)):
                            d = pairwise_dist(drone_positions[ids[i]], drone_positions[ids[j]])
                            dists.append(d)
                    if dists:
                        min_d = min(dists)
                        min_pair_dist_samples.append(min_d)

                        # formation error: RMS deviation from nominal side length
                        errs = [(d - formation_side_length) ** 2 for d in dists]
                        formation_err = math.sqrt(sum(errs) / len(errs))
                        formation_err_samples.append(formation_err)
                        # 队形误差按全队模式拆分（全员search_state==true 判定为搜索模式）
                        if ids and all(i in search_state_now for i in ids):
                            all_search_now = all(search_state_now[i] for i in ids)
                            if all_search_now:
                                formation_err_samples_search.append(formation_err)
                            else:
                                formation_err_samples_tracking.append(formation_err)

                        # inter-drone collision event with hysteresis
                        if (not inter_drone_collision_active) and min_d <= collision_distance:
                            inter_drone_collision_count += 1
                            inter_drone_collision_active = True
                        elif inter_drone_collision_active and min_d >= collision_release_distance:
                            inter_drone_collision_active = False

                # graph formation metric J_f = ||Lhat - Lhat_des||_F^2
                if desired_nodes:
                    req_ids = list(range(len(desired_nodes)))
                    if all(i in drone_positions for i in req_ids):
                        swarm_nodes = [drone_positions[i] for i in req_ids]
                        jf = calc_formation_graph_jf(swarm_nodes, desired_nodes)
                        if jf is not None:
                            formation_jf_samples.append(jf)
                            if req_ids and all(i in search_state_now for i in req_ids):
                                all_search_now = all(search_state_now[i] for i in req_ids)
                                if all_search_now:
                                    formation_jf_samples_search.append(jf)
                                else:
                                    formation_jf_samples_tracking.append(jf)

                # obstacle collision event with hysteresis (per drone)
                if obstacle_query_radius > 0.0 and obstacle_voxel_index:
                    pos = (p.x, p.y, p.z)
                    min_obs_d = query_min_dist_in_radius(
                        pos,
                        obstacle_voxel_index,
                        obstacle_voxel_size,
                        obstacle_query_radius,
                    )
                    if min_obs_d is not None:
                        min_obstacle_dist_samples.append(min_obs_d)
                        if (not obstacle_collision_active[drone_id]) and min_obs_d <= obstacle_collision_distance:
                            obstacle_collision_count += 1
                            obstacle_collision_active[drone_id] = True
                        elif obstacle_collision_active[drone_id] and min_obs_d >= obstacle_collision_release_distance:
                            obstacle_collision_active[drone_id] = False
                    else:
                        if obstacle_collision_active[drone_id]:
                            obstacle_collision_active[drone_id] = False
                continue

            # search_state: /droneX/droneX_target_dpf/search_state
            if topic.endswith('/search_state') and topic.startswith('/drone'):
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                state = bool(msg.data)
                search_state_now[drone_id] = state
                search_state_events.append((ts, drone_id, state))
                continue

            # local_stats: /droneX/droneX_target_dpf/local_stats
            if topic.endswith('/local_stats') and topic.startswith('/drone'):
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                has_obs = bool(getattr(msg, 'has_observation', False))
                prev_obs = obs_state_now.get(drone_id, None)
                if prev_obs is None or prev_obs != has_obs:
                    obs_state_now[drone_id] = has_obs
                    obs_state_events.append((ts, drone_id, has_obs))
                continue

            # replanState: /droneX/replanState or /droneX/planning/replanState
            if topic.endswith('/replanState') and topic.startswith('/drone'):
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                st = int(msg.state)
                prev = drone_last_replan_state.get(drone_id, None)
                prev_ts = drone_last_replan_ts.get(drone_id, None)

                # integrate previous state duration
                if prev == 1 and prev_ts is not None and ts >= prev_ts:
                    replan_hovering_total_duration_by_drone[drone_id] += (ts - prev_ts)

                if st == 1 and prev != 1:
                    replan_failed_hovering_count += 1
                    hovering_episode_start_by_drone[drone_id] = ts
                elif prev == 1 and st != 1:
                    start_t = hovering_episode_start_by_drone.pop(drone_id, None)
                    if start_t is not None and ts >= start_t:
                        dur = ts - start_t
                        if dur > replan_hovering_max_continuous_by_drone[drone_id]:
                            replan_hovering_max_continuous_by_drone[drone_id] = dur

                if st == 2 and prev != 2:
                    emergency_stop_count += 1
                drone_last_replan_state[drone_id] = st
                drone_last_replan_ts[drone_id] = ts
                continue

    if t_min is None or t_max is None:
        raise RuntimeError('bag is empty: {}'.format(bag_path))

    # close replan hovering episodes at bag end
    for drone_id, st in drone_last_replan_state.items():
        if st != 1:
            continue
        prev_ts = drone_last_replan_ts.get(drone_id, None)
        if prev_ts is not None and t_max >= prev_ts:
            replan_hovering_total_duration_by_drone[drone_id] += (t_max - prev_ts)
        start_t = hovering_episode_start_by_drone.get(drone_id, None)
        if start_t is not None and t_max >= start_t:
            dur = t_max - start_t
            if dur > replan_hovering_max_continuous_by_drone[drone_id]:
                replan_hovering_max_continuous_by_drone[drone_id] = dur

    duration = max(0.0, t_max - t_min)

    # visibility/loss/reacq computed from all-drone search_state timeline
    visibility_rate = 0.0
    view_loss_count = 0
    view_loss_duration_list = []
    reacq_time_list = []
    reacq_failure_count = 0

    # only valid after every drone has at least one search_state sample
    drone_ids_search = sorted(search_state_now.keys())
    if drone_ids_search and search_state_events:
        # sort events by time
        search_state_events.sort(key=lambda x: x[0])

        # replay and integrate all_search duration
        known = {}
        all_search_prev = None
        last_t = None
        in_loss = False
        loss_start_t = None
        failure_counted_in_current_loss = False
        valid_duration = 0.0
        visible_duration = 0.0

        for ts, drone_id, st in search_state_events:
            # integrate previous interval
            if last_t is not None and all_search_prev is not None and ts >= last_t:
                dt = ts - last_t
                valid_duration += dt
                if not all_search_prev:
                    visible_duration += dt

                if in_loss and (not failure_counted_in_current_loss):
                    if (ts - loss_start_t) >= reacq_timeout_sec:
                        reacq_failure_count += 1
                        failure_counted_in_current_loss = True

            known[drone_id] = st
            # all_search only meaningful when all known
            if len(known) == len(drone_ids_search):
                all_search_now = all(known[d] for d in drone_ids_search)

                if all_search_prev is None:
                    all_search_prev = all_search_now
                    if all_search_now:
                        in_loss = True
                        loss_start_t = ts
                        view_loss_count += 1
                        failure_counted_in_current_loss = False
                else:
                    if (not all_search_prev) and all_search_now:
                        in_loss = True
                        loss_start_t = ts
                        view_loss_count += 1
                        failure_counted_in_current_loss = False
                    elif all_search_prev and (not all_search_now):
                        if in_loss and loss_start_t is not None:
                            loss_dur = ts - loss_start_t
                            view_loss_duration_list.append(loss_dur)
                            reacq_time_list.append(loss_dur)
                        in_loss = False
                        loss_start_t = None
                        failure_counted_in_current_loss = False
                    all_search_prev = all_search_now

            last_t = ts

        # tail interval until bag end
        if last_t is not None and all_search_prev is not None and t_max >= last_t:
            dt = t_max - last_t
            valid_duration += dt
            if not all_search_prev:
                visible_duration += dt
            if in_loss and (not failure_counted_in_current_loss):
                if (t_max - loss_start_t) >= reacq_timeout_sec:
                    reacq_failure_count += 1
                    failure_counted_in_current_loss = True

        if in_loss and loss_start_t is not None:
            # unresolved loss at end still记入一次loss duration
            view_loss_duration_list.append(t_max - loss_start_t)

        visibility_rate = (visible_duration / valid_duration) if valid_duration > 1e-6 else 0.0

    # single-drone visibility
    search_events_by_drone = defaultdict(list)
    for ts, drone_id, st in search_state_events:
        search_events_by_drone[drone_id].append((ts, bool(st)))
    obs_events_by_drone = defaultdict(list)
    for ts, drone_id, st in obs_state_events:
        obs_events_by_drone[drone_id].append((ts, bool(st)))

    single_vis_rate_by_drone = {}
    single_vis_source_by_drone = {}
    single_vis_tracking_rate_by_drone = {}
    single_vis_tracking_source_by_drone = {}
    single_vis_tracking_valid_duration_by_drone = {}
    single_tracking_mode_duration_by_drone = {}
    for drone_id in sorted(drone_speeds.keys()):
        search_ev = search_events_by_drone.get(drone_id, [])
        obs_ev = obs_events_by_drone.get(drone_id, [])
        if (not search_ev) and (not obs_ev):
            continue
        s_total = integrate_single_drone_visibility_over_total(
            search_ev,
            obs_ev,
            t_min,
            t_max,
        )
        if search_ev:
            t = integrate_single_drone_tracking_visibility(search_ev, obs_ev, t_max)
        else:
            t = {
                'visibility_rate_tracking': 0.0,
                'tracking_duration_sec': 0.0,
                'valid_duration_sec': 0.0,
                'visible_duration_sec': 0.0,
                'source': 'none',
            }
        single_vis_rate_by_drone[str(drone_id)] = float(s_total['visibility_rate_total'])
        single_vis_source_by_drone[str(drone_id)] = s_total['source']
        single_vis_tracking_rate_by_drone[str(drone_id)] = float(t['visibility_rate_tracking'])
        single_vis_tracking_source_by_drone[str(drone_id)] = t['source']
        single_vis_tracking_valid_duration_by_drone[str(drone_id)] = float(t['valid_duration_sec'])
        single_tracking_mode_duration_by_drone[str(drone_id)] = float(t['tracking_duration_sec'])

    single_vis_rates = list(single_vis_rate_by_drone.values())
    single_vis_tracking_rates = list(single_vis_tracking_rate_by_drone.values())

    # dynamics legality summary
    dyn_total_duration_sec = float(sum(dyn_total_dur_by_drone.values()))
    dyn_speed_over_sec = float(sum(dyn_speed_over_dur_by_drone.values()))
    dyn_acc_over_sec = float(sum(dyn_acc_over_dur_by_drone.values()))
    dyn_any_over_sec = float(sum(dyn_any_over_dur_by_drone.values()))
    dyn_speed_over_ratio = (dyn_speed_over_sec / dyn_total_duration_sec) if dyn_total_duration_sec > 1e-6 else 0.0
    dyn_acc_over_ratio = (dyn_acc_over_sec / dyn_total_duration_sec) if dyn_total_duration_sec > 1e-6 else 0.0
    dyn_any_over_ratio = (dyn_any_over_sec / dyn_total_duration_sec) if dyn_total_duration_sec > 1e-6 else 0.0
    dyn_legal_ratio = 1.0 - dyn_any_over_ratio if dyn_total_duration_sec > 1e-6 else 0.0
    dyn_any_over_ratio_by_drone = {}
    for drone_id, total_dur in dyn_total_dur_by_drone.items():
        if total_dur > 1e-6:
            dyn_any_over_ratio_by_drone[str(drone_id)] = float(dyn_any_over_dur_by_drone[drone_id] / total_dur)
        else:
            dyn_any_over_ratio_by_drone[str(drone_id)] = 0.0

    # speed stats (strict window):
    # target maneuver start -> (bag end - tail trim).
    maneuver_start_ts = find_target_maneuver_start(
        target_speed_samples,
        speed_thresh=target_motion_speed_thresh,
        dist_thresh=target_motion_dist_thresh,
        confirm_sec=target_motion_confirm_sec,
    )
    if maneuver_start_ts is None:
        speed_eval_start_ts = float(t_min)
        speed_eval_basis = 'fallback_no_target_motion'
    else:
        speed_eval_start_ts = max(float(t_min), float(maneuver_start_ts))
        speed_eval_basis = 'target_maneuver_window'

    speed_eval_end_ts = max(speed_eval_start_ts, float(t_max) - speed_tail_trim_sec)
    if speed_eval_end_ts <= speed_eval_start_ts + 1e-6:
        speed_eval_start_ts = float(t_min)
        speed_eval_end_ts = float(t_max)
        speed_eval_basis = speed_eval_basis + '_fallback_full_duration'

    def cut_speed_samples(samples_by_drone):
        cut = {}
        for drone_id, samples in samples_by_drone.items():
            cut[drone_id] = [
                spd for ts, spd in samples
                if (float(ts) >= speed_eval_start_ts and float(ts) <= speed_eval_end_ts)
            ]
        return cut

    drone_speeds_cut = cut_speed_samples(drone_speed_samples)
    drone_speeds_tracking_cut = cut_speed_samples(drone_speed_samples_tracking)
    drone_speeds_search_cut = cut_speed_samples(drone_speed_samples_search)

    speed_metrics = {}
    speed_metrics_tracking = {}
    speed_metrics_search = {}
    all_speed_samples = []
    all_speed_samples_tracking = []
    all_speed_samples_search = []
    for drone_id in sorted(drone_speed_samples.keys()):
        s = drone_speeds_cut.get(drone_id, [])
        s_tracking = drone_speeds_tracking_cut.get(drone_id, [])
        s_search = drone_speeds_search_cut.get(drone_id, [])
        all_speed_samples.extend(s)
        all_speed_samples_tracking.extend(s_tracking)
        all_speed_samples_search.extend(s_search)
        speed_metrics[str(drone_id)] = {
            'mean': mean_or_zero(s),
            'p95': percentile(s, 0.95),
            'max': max(s) if s else 0.0,
        }
        speed_metrics_tracking[str(drone_id)] = {
            'mean': mean_or_zero(s_tracking),
            'p95': percentile(s_tracking, 0.95),
            'max': max(s_tracking) if s_tracking else 0.0,
        }
        speed_metrics_search[str(drone_id)] = {
            'mean': mean_or_zero(s_search),
            'p95': percentile(s_search, 0.95),
            'max': max(s_search) if s_search else 0.0,
        }

    target_loss_durations_sec = [float(x) for x in view_loss_duration_list]
    target_loss_total_duration_sec = float(sum(target_loss_durations_sec))
    target_loss_max_duration_sec = float(max(target_loss_durations_sec)) if target_loss_durations_sec else 0.0
    collision_count = inter_drone_collision_count + obstacle_collision_count
    replan_hovering_total_duration_by_drone_out = {
        str(k): float(v) for k, v in replan_hovering_total_duration_by_drone.items()
    }
    replan_hovering_max_continuous_by_drone_out = {
        str(k): float(v) for k, v in replan_hovering_max_continuous_by_drone.items()
    }
    replan_hovering_max_continuous_sec = (
        float(max(replan_hovering_max_continuous_by_drone.values()))
        if replan_hovering_max_continuous_by_drone else 0.0
    )
    long_hovering_drones = []
    if hovering_fail_duration_sec > 1e-6:
        for drone_id, dur in replan_hovering_max_continuous_by_drone.items():
            if dur >= hovering_fail_duration_sec:
                long_hovering_drones.append(int(drone_id))
    long_hovering_drones = sorted(long_hovering_drones)

    task_failure_reasons = []
    if collision_count > 0:
        task_failure_reasons.append(
            'collision_count={} (inter_drone={}, obstacle={})'.format(
                collision_count, inter_drone_collision_count, obstacle_collision_count
            )
        )
    if target_loss_max_duration_sec > float(reacq_timeout_sec):
        task_failure_reasons.append(
            'target_loss_max_duration_sec={:.3f} > reacq_timeout_sec={:.3f}'.format(
                target_loss_max_duration_sec, float(reacq_timeout_sec)
            )
        )
    if long_hovering_drones:
        drone_parts = [
            'drone{}:{:.3f}s'.format(did, replan_hovering_max_continuous_by_drone.get(did, 0.0))
            for did in long_hovering_drones
        ]
        task_failure_reasons.append(
            'replan_failed_hovering_max_continuous >= {:.3f}s ({})'.format(
                hovering_fail_duration_sec, ', '.join(drone_parts)
            )
        )

    task_success = (len(task_failure_reasons) == 0)

    result = {
        'bag_path': bag_path,
        'duration_sec': duration,
        'num_drones_observed': len(drone_speeds),
        'visibility_rate': visibility_rate,
        'single_drone_visibility_rate_by_drone': single_vis_rate_by_drone,
        'single_drone_visibility_source_by_drone': single_vis_source_by_drone,
        'single_drone_visibility_rate_mean': mean_or_zero(single_vis_rates),
        'single_drone_visibility_rate_min': min(single_vis_rates) if single_vis_rates else 0.0,
        'single_drone_visibility_rate_tracking_by_drone': single_vis_tracking_rate_by_drone,
        'single_drone_visibility_tracking_source_by_drone': single_vis_tracking_source_by_drone,
        'single_drone_visibility_tracking_valid_duration_sec_by_drone': single_vis_tracking_valid_duration_by_drone,
        'single_drone_tracking_mode_duration_sec_by_drone': single_tracking_mode_duration_by_drone,
        'single_drone_visibility_rate_tracking_mean': mean_or_zero(single_vis_tracking_rates),
        'single_drone_visibility_rate_tracking_min': (
            min(single_vis_tracking_rates) if single_vis_tracking_rates else 0.0
        ),
        'view_loss_count': view_loss_count,
        'target_loss_durations_sec': target_loss_durations_sec,
        'target_loss_total_duration_sec': target_loss_total_duration_sec,
        'target_loss_max_duration_sec': target_loss_max_duration_sec,
        'reacq_count': len(reacq_time_list),
        'reacq_time_mean_sec': mean_or_zero(reacq_time_list),
        'reacq_time_p95_sec': percentile(reacq_time_list, 0.95),
        'reacq_time_max_sec': max(reacq_time_list) if reacq_time_list else 0.0,
        'reacq_failure_count': reacq_failure_count,
        'task_success': task_success,
        'task_failure_reasons': task_failure_reasons,
        'task_failure_reason': '; '.join(task_failure_reasons),
        'task_success_loss_timeout_sec': float(reacq_timeout_sec),
        'formation_error_mean': mean_or_zero(formation_err_samples),
        'formation_error_p95': percentile(formation_err_samples, 0.95),
        'formation_error_max': max(formation_err_samples) if formation_err_samples else 0.0,
        'formation_error_mean_tracking': mean_or_zero(formation_err_samples_tracking),
        'formation_error_p95_tracking': percentile(formation_err_samples_tracking, 0.95),
        'formation_error_max_tracking': max(formation_err_samples_tracking) if formation_err_samples_tracking else 0.0,
        'formation_error_mean_search': mean_or_zero(formation_err_samples_search),
        'formation_error_p95_search': percentile(formation_err_samples_search, 0.95),
        'formation_error_max_search': max(formation_err_samples_search) if formation_err_samples_search else 0.0,
        'formation_type': int(formation_type),
        'formation_graph_jf_mean': mean_or_zero(formation_jf_samples),
        'formation_graph_jf_p95': percentile(formation_jf_samples, 0.95),
        'formation_graph_jf_max': max(formation_jf_samples) if formation_jf_samples else 0.0,
        'formation_graph_jf_mean_tracking': mean_or_zero(formation_jf_samples_tracking),
        'formation_graph_jf_p95_tracking': percentile(formation_jf_samples_tracking, 0.95),
        'formation_graph_jf_max_tracking': (
            max(formation_jf_samples_tracking) if formation_jf_samples_tracking else 0.0
        ),
        'formation_graph_jf_mean_search': mean_or_zero(formation_jf_samples_search),
        'formation_graph_jf_p95_search': percentile(formation_jf_samples_search, 0.95),
        'formation_graph_jf_max_search': (
            max(formation_jf_samples_search) if formation_jf_samples_search else 0.0
        ),
        'collision_count': collision_count,
        'inter_drone_collision_count': inter_drone_collision_count,
        'obstacle_collision_count': obstacle_collision_count,
        'min_inter_drone_distance': min(min_pair_dist_samples) if min_pair_dist_samples else 0.0,
        'obstacle_map_available': bool(obstacle_map_points_count > 0),
        'obstacle_map_points_count': int(obstacle_map_points_count),
        'min_obstacle_clearance': min(min_obstacle_dist_samples) if min_obstacle_dist_samples else 0.0,
        'dynamics_vmax_limit': float(dyn_vmax_limit),
        'dynamics_amax_limit': float(dyn_amax_limit),
        'dynamics_total_duration_sec': dyn_total_duration_sec,
        'dynamics_speed_overlimit_ratio': dyn_speed_over_ratio,
        'dynamics_acc_overlimit_ratio': dyn_acc_over_ratio,
        'dynamics_overlimit_ratio': dyn_any_over_ratio,
        'dynamics_legal_ratio': dyn_legal_ratio,
        'dynamics_overlimit_ratio_by_drone': dyn_any_over_ratio_by_drone,
        'replan_failed_hovering_count': replan_failed_hovering_count,
        'replan_failed_hovering_duration_by_drone': replan_hovering_total_duration_by_drone_out,
        'replan_failed_hovering_max_continuous_sec_by_drone': replan_hovering_max_continuous_by_drone_out,
        'replan_failed_hovering_max_continuous_sec': replan_hovering_max_continuous_sec,
        'hovering_fail_duration_sec': float(hovering_fail_duration_sec),
        'replan_failed_hovering_long_stuck_drones': long_hovering_drones,
        'replan_failed_hovering_long_stuck_detected': bool(len(long_hovering_drones) > 0),
        'emergency_stop_count': emergency_stop_count,
        'speed_eval_basis': speed_eval_basis,
        'speed_tail_trim_sec': float(speed_tail_trim_sec),
        'speed_eval_start_time_sec': float(speed_eval_start_ts - t_min),
        'speed_eval_end_time_sec': float(speed_eval_end_ts - t_min),
        'speed_eval_duration_sec': float(max(0.0, speed_eval_end_ts - speed_eval_start_ts)),
        'speed_mean_all': mean_or_zero(all_speed_samples),
        'speed_p95_all': percentile(all_speed_samples, 0.95),
        'speed_max_all': max(all_speed_samples) if all_speed_samples else 0.0,
        'speed_mean_tracking': mean_or_zero(all_speed_samples_tracking),
        'speed_p95_tracking': percentile(all_speed_samples_tracking, 0.95),
        'speed_max_tracking': max(all_speed_samples_tracking) if all_speed_samples_tracking else 0.0,
        'speed_mean_search': mean_or_zero(all_speed_samples_search),
        'speed_p95_search': percentile(all_speed_samples_search, 0.95),
        'speed_max_search': max(all_speed_samples_search) if all_speed_samples_search else 0.0,
        'speed_by_drone': speed_metrics,
        'speed_by_drone_tracking': speed_metrics_tracking,
        'speed_by_drone_search': speed_metrics_search,
    }
    return result


def write_summary_csv(rows, out_csv):
    if not rows:
        return
    fieldnames = [
        'run_id',
        'bag_path',
        'duration_sec',
        'num_drones_observed',
        'visibility_rate',
        'single_drone_visibility_rate_mean',
        'single_drone_visibility_rate_min',
        'single_drone_visibility_rate_tracking_mean',
        'single_drone_visibility_rate_tracking_min',
        'view_loss_count',
        'target_loss_total_duration_sec',
        'target_loss_max_duration_sec',
        'reacq_count',
        'reacq_time_mean_sec',
        'reacq_time_p95_sec',
        'reacq_time_max_sec',
        'reacq_failure_count',
        'task_success',
        'task_failure_reason',
        'formation_error_mean',
        'formation_error_p95',
        'formation_error_max',
        'formation_error_mean_tracking',
        'formation_error_p95_tracking',
        'formation_error_max_tracking',
        'formation_error_mean_search',
        'formation_error_p95_search',
        'formation_error_max_search',
        'formation_graph_jf_mean',
        'formation_graph_jf_p95',
        'formation_graph_jf_max',
        'formation_graph_jf_mean_tracking',
        'formation_graph_jf_p95_tracking',
        'formation_graph_jf_max_tracking',
        'formation_graph_jf_mean_search',
        'formation_graph_jf_p95_search',
        'formation_graph_jf_max_search',
        'collision_count',
        'inter_drone_collision_count',
        'obstacle_collision_count',
        'min_inter_drone_distance',
        'obstacle_map_available',
        'obstacle_map_points_count',
        'min_obstacle_clearance',
        'dynamics_overlimit_ratio',
        'dynamics_speed_overlimit_ratio',
        'dynamics_acc_overlimit_ratio',
        'dynamics_legal_ratio',
        'replan_failed_hovering_count',
        'replan_failed_hovering_max_continuous_sec',
        'replan_failed_hovering_long_stuck_detected',
        'emergency_stop_count',
        'speed_eval_basis',
        'speed_tail_trim_sec',
        'speed_eval_start_time_sec',
        'speed_eval_end_time_sec',
        'speed_eval_duration_sec',
        'speed_mean_all',
        'speed_p95_all',
        'speed_max_all',
        'speed_mean_tracking',
        'speed_p95_tracking',
        'speed_max_tracking',
        'speed_mean_search',
        'speed_p95_search',
        'speed_max_search',
    ]
    with open(out_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            out = {k: row.get(k, '') for k in fieldnames}
            writer.writerow(out)


def main():
    parser = argparse.ArgumentParser(description='Analyze tracking experiment rosbag.')
    parser.add_argument('bags', nargs='+', help='Bag file paths')
    parser.add_argument('--formation-side-length', type=float, default=2.0)
    parser.add_argument('--formation-type', type=int, default=2,
                        help='Formation type consistent with traj_opt (1:hex,2:triangle,3:square,4:pentagon).')
    parser.add_argument('--collision-distance', type=float, default=0)
    parser.add_argument('--collision-release-distance', type=float, default=0)
    parser.add_argument('--obstacle-collision-distance', type=float, default=0.0)
    parser.add_argument('--obstacle-collision-release-distance', type=float, default=0.0)
    parser.add_argument('--reacq-timeout-sec', type=float, default=10.0)
    parser.add_argument('--hovering-fail-duration-sec', type=float, default=5.0,
                        help='Fail task if any drone stays in replan failed hovering continuously beyond this time.')
    parser.add_argument('--dyn-vmax-limit', type=float, default=3.0,
                        help='Speed limit used by trajectory dynamics legality analysis.')
    parser.add_argument('--dyn-amax-limit', type=float, default=6.0,
                        help='Acceleration limit used by trajectory dynamics legality analysis.')
    parser.add_argument('--speed-tail-trim-sec', type=float, default=11.0,
                        help='Trim this many seconds from bag end when computing speed metrics.')
    parser.add_argument('--target-motion-speed-thresh', type=float, default=0.2,
                        help='Target speed threshold (m/s) for maneuver-start detection.')
    parser.add_argument('--target-motion-dist-thresh', type=float, default=0.5,
                        help='Target displacement threshold (m) from initial point for maneuver-start detection.')
    parser.add_argument('--target-motion-confirm-sec', type=float, default=1.0,
                        help='Required continuous moving duration (s) to confirm maneuver start.')
    parser.add_argument('--out-json', default='', help='Output JSON file path')
    parser.add_argument('--out-csv', default='', help='Output CSV file path')
    args = parser.parse_args()

    rows = []
    for i, bag_path in enumerate(args.bags, start=1):
        result = analyze_bag(
            bag_path=bag_path,
            formation_side_length=args.formation_side_length,
            formation_type=args.formation_type,
            collision_distance=args.collision_distance,
            collision_release_distance=args.collision_release_distance,
            obstacle_collision_distance=args.obstacle_collision_distance,
            obstacle_collision_release_distance=args.obstacle_collision_release_distance,
            reacq_timeout_sec=args.reacq_timeout_sec,
            hovering_fail_duration_sec=args.hovering_fail_duration_sec,
            dyn_vmax_limit=args.dyn_vmax_limit,
            dyn_amax_limit=args.dyn_amax_limit,
            speed_tail_trim_sec=args.speed_tail_trim_sec,
            target_motion_speed_thresh=args.target_motion_speed_thresh,
            target_motion_dist_thresh=args.target_motion_dist_thresh,
            target_motion_confirm_sec=args.target_motion_confirm_sec,
        )
        result['run_id'] = i
        if not result.get('task_success', True):
            print('[run {}] task failed: {}'.format(i, result.get('task_failure_reason', 'unknown')), file=sys.stderr)
        rows.append(result)

    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, 'w') as f:
            json.dump(rows, f, indent=2)

    if args.out_csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_csv)), exist_ok=True)
        write_summary_csv(rows, args.out_csv)

    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
