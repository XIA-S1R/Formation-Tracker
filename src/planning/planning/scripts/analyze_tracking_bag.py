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
    collision_distance=0.0,
    collision_release_distance=0.0,
    reacq_timeout_sec=10.0,
):
    drone_positions = {}  # drone_id -> (x,y,z)
    drone_speeds = defaultdict(list)
    drone_speeds_tracking = defaultdict(list)
    drone_speeds_search = defaultdict(list)
    drone_last_replan_state = {}
    replan_failed_hovering_count = 0
    emergency_stop_count = 0

    # search_state timeline
    search_state_now = {}
    search_state_events = []  # (t, drone_id, state)

    # odom-driven evaluation timeline
    eval_times = []
    formation_err_samples = []
    formation_err_samples_tracking = []
    formation_err_samples_search = []
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
                speed = vec_norm3(v.x, v.y, v.z)
                drone_positions[drone_id] = (p.x, p.y, p.z)
                drone_speeds[drone_id].append(speed)
                # 速度按本机模式拆分
                if drone_id in search_state_now:
                    if search_state_now[drone_id]:
                        drone_speeds_search[drone_id].append(speed)
                    else:
                        drone_speeds_tracking[drone_id].append(speed)

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
                if st == 1 and prev != 1:
                    replan_failed_hovering_count += 1
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
    speed_metrics_tracking = {}
    speed_metrics_search = {}
    all_speed_samples = []
    all_speed_samples_tracking = []
    all_speed_samples_search = []
    for drone_id in sorted(drone_speeds.keys()):
        s = drone_speeds[drone_id]
        s_tracking = drone_speeds_tracking[drone_id]
        s_search = drone_speeds_search[drone_id]
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
    task_success = (collision_count == 0 and target_loss_max_duration_sec <= float(reacq_timeout_sec))

    result = {
        'bag_path': bag_path,
        'duration_sec': duration,
        'num_drones_observed': len(drone_speeds),
        'visibility_rate': visibility_rate,
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
        'collision_count': collision_count,
        'min_inter_drone_distance': min(min_pair_dist_samples) if min_pair_dist_samples else 0.0,
        'replan_failed_hovering_count': replan_failed_hovering_count,
        'emergency_stop_count': emergency_stop_count,
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
        'view_loss_count',
        'target_loss_total_duration_sec',
        'target_loss_max_duration_sec',
        'reacq_count',
        'reacq_time_mean_sec',
        'reacq_time_p95_sec',
        'reacq_time_max_sec',
        'reacq_failure_count',
        'task_success',
        'formation_error_mean',
        'formation_error_p95',
        'formation_error_max',
        'formation_error_mean_tracking',
        'formation_error_p95_tracking',
        'formation_error_max_tracking',
        'formation_error_mean_search',
        'formation_error_p95_search',
        'formation_error_max_search',
        'collision_count',
        'min_inter_drone_distance',
        'replan_failed_hovering_count',
        'emergency_stop_count',
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
    parser.add_argument('--collision-distance', type=float, default=0.0)
    parser.add_argument('--collision-release-distance', type=float, default=0.0)
    parser.add_argument('--reacq-timeout-sec', type=float, default=10.0)
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
