#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import datetime as dt
import json
import os
import signal
import subprocess
import time
from pathlib import Path


def start_process(cmd, cwd, log_path):
    log_f = open(log_path, 'w')
    proc = subprocess.Popen(
        ['/bin/bash', '-lc', cmd],
        cwd=str(cwd),
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, log_f


def stop_process(proc, log_f, name='proc', sig=signal.SIGINT, wait_sec=12):
    if proc is None:
        return
    if proc.poll() is not None:
        if log_f:
            log_f.close()
        return
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < wait_sec:
        if proc.poll() is not None:
            break
        time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        t1 = time.time()
        while time.time() - t1 < 5:
            if proc.poll() is not None:
                break
            time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    if log_f:
        log_f.close()


def wait_for_ros_master(timeout_sec=30):
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        rc = subprocess.run(
            ['/bin/bash', '-lc', 'rosparam list >/dev/null 2>&1'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        if rc == 0:
            return True
        time.sleep(0.5)
    return False


def wait_for_topics(topics, timeout_sec=40):
    needed = set(topics)
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        p = subprocess.run(
            ['/bin/bash', '-lc', 'rostopic list'],
            capture_output=True,
            text=True,
        )
        if p.returncode == 0:
            existing = set([x.strip() for x in p.stdout.splitlines() if x.strip()])
            if needed.issubset(existing):
                return True
        time.sleep(0.8)
    return False


def run_cmd(cmd, cwd, log_path):
    with open(log_path, 'w') as f:
        return subprocess.run(['/bin/bash', '-lc', cmd], cwd=str(cwd), stdout=f, stderr=subprocess.STDOUT).returncode


def run_trigger_cmd(cmd, cwd, log_path, max_run_sec=1.5):
    """
    Trigger command may be a continuous publisher (e.g. pub_triger.sh).
    Run it for at most max_run_sec, then stop and treat timeout-stop as success.
    """
    with open(log_path, 'w') as f:
        proc = subprocess.Popen(
            ['/bin/bash', '-lc', cmd],
            cwd=str(cwd),
            stdout=f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        try:
            return proc.wait(timeout=max(0.1, float(max_run_sec)))
        except subprocess.TimeoutExpired:
            # Expected for continuous trigger publishers; stop and continue.
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
            return 0


def read_metrics_json(path):
    with open(path, 'r') as f:
        data = json.load(f)
    if isinstance(data, list) and data:
        return data[0]
    if isinstance(data, dict):
        return data
    return {}


def write_summary_csv(rows, out_csv):
    fieldnames = [
        'run_id',
        'status',
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
        'replan_failed_hovering_count',
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
            writer.writerow({k: row.get(k, '') for k in fieldnames})


def build_default_topics(drone_count):
    topics = ['/target/odom']
    # topics.append('/global_map')
    for i in range(drone_count):
        topics.append(f'/drone{i}/odom')
    for i in range(drone_count):
        topics.append(f'/drone{i}/drone{i}_target_dpf/search_state')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/imu')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/so3cmd')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/position_cmd')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/search_particles_vis_pre_prune')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/local_stats')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/search_pos_gmm')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/search_particles_vis_post_prune')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/invalid_region_gmm')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/search_targets')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/drone{i}_target_dpf/search_particles_vis')
    # for i in range(drone_count):
    #     topics.append(f'/drone{i}/replanState')
    return topics


def main():
    script_path = Path(__file__).resolve()
    repo_root = script_path.parents[4]
    script_dir = script_path.parent

    parser = argparse.ArgumentParser(description='Batch run full_evasion tracking experiments and compute metrics.')
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--run-duration', type=float, default=120.0,
                        help='Max run seconds as fallback if evasion script does not exit.')
    parser.add_argument('--warmup', type=float, default=8.0)
    parser.add_argument('--cooldown', type=float, default=2.0)
    parser.add_argument('--drone-count', type=int, default=3)

    parser.add_argument('--launch-cmd', default='roslaunch simulation tracking_sim_triangle.launch')
    parser.add_argument('--trigger-cmd', default='./sh_utils/pub_triger.sh')
    parser.add_argument('--trigger-max-seconds', type=float, default=1.5)
    parser.add_argument('--evasion-cmd', default='python3 src/planning/planning/scripts/full_evasion.py')
    parser.add_argument('--rviz-cmd', default='rviz -d $(rospack find simulation)/config/tracking_sim.rviz')
    parser.add_argument('--with-rviz', action='store_true')

    parser.add_argument('--formation-side-length', type=float, default=2.0)
    parser.add_argument('--formation-type', type=int, default=2)
    parser.add_argument('--collision-distance', type=float, default=0.0)
    parser.add_argument('--collision-release-distance', type=float, default=0.0)
    parser.add_argument('--obstacle-collision-distance', type=float, default=0.0)
    parser.add_argument('--obstacle-collision-release-distance', type=float, default=0.0)
    parser.add_argument('--reacq-timeout-sec', type=float, default=10.0)
    parser.add_argument('--post-reacq-extra-sec', type=float, default=2.0,
                        help='Extra seconds after reacq-timeout before stopping one run.')
    parser.add_argument('--speed-tail-trim-sec', type=float, default=11.0,
                        help='Trim this many seconds from each run tail when computing speed metrics.')
    parser.add_argument('--target-motion-speed-thresh', type=float, default=0.2,
                        help='Target speed threshold for maneuver-start detection in analysis.')
    parser.add_argument('--target-motion-dist-thresh', type=float, default=0.5,
                        help='Target displacement threshold for maneuver-start detection in analysis.')
    parser.add_argument('--target-motion-confirm-sec', type=float, default=1.0,
                        help='Required continuous moving duration to confirm target maneuver start in analysis.')

    parser.add_argument('--output-dir', default='')
    args = parser.parse_args()

    if args.runs <= 0:
        raise ValueError('--runs must be > 0')

    timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    if args.output_dir:
        out_dir = Path(args.output_dir).expanduser().resolve()
    else:
        out_dir = (repo_root / 'experiment_results' / f'full_evasion_batch_{timestamp}').resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    topics = build_default_topics(args.drone_count)

    all_rows = []

    for run_idx in range(1, args.runs + 1):
        run_tag = f'run_{run_idx:03d}'
        run_dir = out_dir / run_tag
        run_dir.mkdir(parents=True, exist_ok=True)

        bag_path = run_dir / 'tracking.bag'
        run_metrics_json = run_dir / 'metrics.json'
        launch_log = run_dir / 'launch.log'
        rviz_log = run_dir / 'rviz.log'
        bag_log = run_dir / 'rosbag.log'
        trigger_log = run_dir / 'trigger.log'
        evasion_log = run_dir / 'evasion.log'
        analyze_log = run_dir / 'analyze.log'

        print(f'[{run_tag}] starting launch...')
        launch_proc, launch_f = start_process(args.launch_cmd, repo_root, launch_log)
        rviz_proc, rviz_f = (None, None)
        if args.with_rviz:
            rviz_proc, rviz_f = start_process(args.rviz_cmd, repo_root, rviz_log)

        status = 'ok'
        run_row = {'run_id': run_idx, 'status': status, 'bag_path': str(bag_path)}

        try:
            if not wait_for_ros_master(timeout_sec=40):
                raise RuntimeError('ROS master not ready')

            essential_topics = ['/target/odom'] + [f'/drone{i}/odom' for i in range(args.drone_count)]
            if not wait_for_topics(essential_topics, timeout_sec=50):
                raise RuntimeError('essential odom topics not ready')

            topic_str = ' '.join(topics)
            bag_cmd = f'rosbag record -O {bag_path} {topic_str}'
            print(f'[{run_tag}] start rosbag...')
            bag_proc, bag_f = start_process(bag_cmd, repo_root, bag_log)

            time.sleep(max(0.0, args.warmup))

            print(f'[{run_tag}] trigger...')
            trig_rc = run_trigger_cmd(
                args.trigger_cmd,
                repo_root,
                trigger_log,
                max_run_sec=args.trigger_max_seconds,
            )
            if trig_rc != 0:
                raise RuntimeError('trigger command failed')

            print(f'[{run_tag}] evasion...')
            evasion_proc, evasion_f = start_process(args.evasion_cmd, repo_root, evasion_log)

            # 自动收尾策略：
            # 1) evasion脚本结束 -> 再等待(reacq_timeout + extra)秒后结束本轮
            # 2) 若evasion迟迟不结束 -> run-duration作为兜底上限
            t0 = time.time()
            evasion_done_t = None
            post_wait_sec = max(0.0, args.reacq_timeout_sec + args.post_reacq_extra_sec)
            while True:
                now = time.time()
                elapsed = now - t0

                if evasion_done_t is None and evasion_proc.poll() is not None:
                    if evasion_proc.returncode != 0:
                        raise RuntimeError(f'evasion command failed, returncode={evasion_proc.returncode}')
                    evasion_done_t = now
                    print(f'[{run_tag}] evasion done, waiting tail {post_wait_sec:.1f}s...')

                if evasion_done_t is not None:
                    if (now - evasion_done_t) >= post_wait_sec:
                        break
                else:
                    if elapsed >= args.run_duration:
                        print(f'[{run_tag}] fallback timeout {args.run_duration:.1f}s reached, stop this run.')
                        break

                time.sleep(0.5)

            stop_process(evasion_proc, evasion_f, name='evasion')
            time.sleep(max(0.0, args.cooldown))

            stop_process(bag_proc, bag_f, name='rosbag')

            analyze_script = script_dir / 'analyze_tracking_bag.py'
            analyze_cmd = (
                f'python3 {analyze_script} {bag_path} '
                f'--formation-side-length {args.formation_side_length} '
                f'--formation-type {args.formation_type} '
                f'--collision-distance {args.collision_distance} '
                f'--collision-release-distance {args.collision_release_distance} '
                f'--obstacle-collision-distance {args.obstacle_collision_distance} '
                f'--obstacle-collision-release-distance {args.obstacle_collision_release_distance} '
                f'--reacq-timeout-sec {args.reacq_timeout_sec} '
                f'--speed-tail-trim-sec {args.speed_tail_trim_sec} '
                f'--target-motion-speed-thresh {args.target_motion_speed_thresh} '
                f'--target-motion-dist-thresh {args.target_motion_dist_thresh} '
                f'--target-motion-confirm-sec {args.target_motion_confirm_sec} '
                f'--out-json {run_metrics_json}'
            )
            print(f'[{run_tag}] analyzing bag...')
            analyze_rc = run_cmd(analyze_cmd, repo_root, analyze_log)
            if analyze_rc != 0:
                raise RuntimeError('bag analysis failed')

            metrics = read_metrics_json(run_metrics_json)
            metrics['run_id'] = run_idx
            metrics['status'] = 'ok'
            all_rows.append(metrics)
            print(f'[{run_tag}] done, visibility={metrics.get("visibility_rate", 0.0):.3f}, collisions={metrics.get("collision_count", 0)}, task_success={metrics.get("task_success", False)}')

        except Exception as e:
            run_row['status'] = f'failed: {e}'
            all_rows.append(run_row)
            print(f'[{run_tag}] FAILED: {e}')

        finally:
            stop_process(rviz_proc, rviz_f, name='rviz')
            stop_process(launch_proc, launch_f, name='launch')

    summary_json = out_dir / 'summary.json'
    summary_csv = out_dir / 'summary.csv'

    with open(summary_json, 'w') as f:
        json.dump(all_rows, f, indent=2)
    write_summary_csv(all_rows, summary_csv)

    success_rows = [r for r in all_rows if 'task_success' in r]
    if success_rows:
        success_count = sum(1 for r in success_rows if bool(r.get('task_success', False)))
        success_rate = success_count / float(len(success_rows))
        print(f'Task success rate: {success_count}/{len(success_rows)} = {success_rate:.3f}')

    print(f'All done. Summary JSON: {summary_json}')
    print(f'All done. Summary CSV : {summary_csv}')


if __name__ == '__main__':
    main()
