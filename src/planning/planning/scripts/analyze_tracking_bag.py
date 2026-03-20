#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import statistics
from collections import defaultdict

import rosbag


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


def mean_or_zero(values):
    return float(statistics.mean(values)) if values else 0.0


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


def analyze_bag(
    bag_path,
    formation_side_length=2.0,
    collision_distance=0.35,
    collision_release_distance=0.45,
    reacq_timeout_sec=15.0,
):
    drone_positions = {}  # drone_id -> (x,y,z)
    drone_speeds = defaultdict(list)
    drone_last_replan_state = {}
    emergency_stop_count = 0

    # search_state timeline
    search_state_now = {}
    search_state_events = []  # (t, drone_id, state)

    # odom-driven evaluation timeline
    eval_times = []
    formation_err_samples = []
    min_pair_dist_samples = []
    collision_count = 0
    collision_active = False

    t_min = None
    t_max = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            ts = t.to_sec()
            if t_min is None or ts < t_min:
                t_min = ts
            if t_max is None or ts > t_max:
                t_max = ts

            # Drone odom: /droneX/odom
            if topic.endswith('/odom') and topic.startswith('/drone') and topic.count('/') == 2:
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                drone_positions[drone_id] = (p.x, p.y, p.z)
                drone_speeds[drone_id].append(vec_norm3(v.x, v.y, v.z))

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

                        # collision event with hysteresis
                        if (not collision_active) and min_d <= collision_distance:
                            collision_count += 1
                            collision_active = True
                        elif collision_active and min_d >= collision_release_distance:
                            collision_active = False
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

            # replanState: /droneX/replanState or /droneX/planning/replanState
            if topic.endswith('/replanState') and topic.startswith('/drone'):
                drone_id = parse_drone_id_from_topic(topic)
                if drone_id is None:
                    continue
                st = int(msg.state)
                prev = drone_last_replan_state.get(drone_id, None)
                if st == 2 and prev != 2:
                    emergency_stop_count += 1
                drone_last_replan_state[drone_id] = st
                continue

    if t_min is None or t_max is None:
        raise RuntimeError('bag is empty: {}'.format(bag_path))

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

    # speed stats
    speed_metrics = {}
    all_speed_samples = []
    for drone_id in sorted(drone_speeds.keys()):
        s = drone_speeds[drone_id]
        all_speed_samples.extend(s)
        speed_metrics[str(drone_id)] = {
            'mean': mean_or_zero(s),
            'p95': percentile(s, 0.95),
            'max': max(s) if s else 0.0,
        }

    result = {
        'bag_path': bag_path,
        'duration_sec': duration,
        'num_drones_observed': len(drone_speeds),
        'visibility_rate': visibility_rate,
        'view_loss_count': view_loss_count,
        'reacq_count': len(reacq_time_list),
        'reacq_time_mean_sec': mean_or_zero(reacq_time_list),
        'reacq_time_p95_sec': percentile(reacq_time_list, 0.95),
        'reacq_time_max_sec': max(reacq_time_list) if reacq_time_list else 0.0,
        'reacq_failure_count': reacq_failure_count,
        'formation_error_mean': mean_or_zero(formation_err_samples),
        'formation_error_p95': percentile(formation_err_samples, 0.95),
        'formation_error_max': max(formation_err_samples) if formation_err_samples else 0.0,
        'collision_count': collision_count,
        'min_inter_drone_distance': min(min_pair_dist_samples) if min_pair_dist_samples else 0.0,
        'emergency_stop_count': emergency_stop_count,
        'speed_mean_all': mean_or_zero(all_speed_samples),
        'speed_p95_all': percentile(all_speed_samples, 0.95),
        'speed_max_all': max(all_speed_samples) if all_speed_samples else 0.0,
        'speed_by_drone': speed_metrics,
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
        'view_loss_count',
        'reacq_count',
        'reacq_time_mean_sec',
        'reacq_time_p95_sec',
        'reacq_time_max_sec',
        'reacq_failure_count',
        'formation_error_mean',
        'formation_error_p95',
        'formation_error_max',
        'collision_count',
        'min_inter_drone_distance',
        'emergency_stop_count',
        'speed_mean_all',
        'speed_p95_all',
        'speed_max_all',
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
    parser.add_argument('--collision-distance', type=float, default=0.35)
    parser.add_argument('--collision-release-distance', type=float, default=0.45)
    parser.add_argument('--reacq-timeout-sec', type=float, default=15.0)
    parser.add_argument('--out-json', default='', help='Output JSON file path')
    parser.add_argument('--out-csv', default='', help='Output CSV file path')
    args = parser.parse_args()

    rows = []
    for i, bag_path in enumerate(args.bags, start=1):
        result = analyze_bag(
            bag_path=bag_path,
            formation_side_length=args.formation_side_length,
            collision_distance=args.collision_distance,
            collision_release_distance=args.collision_release_distance,
            reacq_timeout_sec=args.reacq_timeout_sec,
        )
        result['run_id'] = i
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
