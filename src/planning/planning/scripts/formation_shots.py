#!/usr/bin/env python3

import argparse
import math
from itertools import combinations
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
import rosbag
import sensor_msgs.point_cloud2 as pc2
from matplotlib.lines import Line2D
from matplotlib import font_manager
from matplotlib.patches import Circle
from matplotlib.legend_handler import HandlerBase
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


class QuadLegendProxy:
    def __init__(self, color):
        self.color = color


class QuadLegendHandler(HandlerBase):
    def create_artists(
        self,
        legend,
        orig_handle,
        xdescent,
        ydescent,
        width,
        height,
        fontsize,
        trans,
    ):
        c = orig_handle.color
        cx = xdescent + 0.5 * width
        cy = ydescent + 0.5 * height
        arm = 0.32 * width
        rotor_r = 0.085 * height

        artists = []
        artists.append(
            Line2D([cx - arm, cx + arm], [cy, cy], color=c, lw=1.4, transform=trans)
        )
        artists.append(
            Line2D([cx, cx], [cy - arm, cy + arm], color=c, lw=1.4, transform=trans)
        )
        for px, py in ((cx - arm, cy), (cx + arm, cy), (cx, cy - arm), (cx, cy + arm)):
            artists.append(
                Circle((px, py), rotor_r, fill=False, edgecolor=c, linewidth=1.1, transform=trans)
            )
        artists.append(
            Circle((cx, cy), rotor_r * 0.75, fill=True, facecolor=c, edgecolor=c, linewidth=0.8, transform=trans)
        )
        return artists


def list_candidate_cjk_fonts():
    try:
        font_manager._load_fontmanager(try_read_cache=False)
    except Exception:
        pass
    names = sorted({f.name for f in font_manager.fontManager.ttflist})
    keys = ("cjk", "noto", "han", "hei", "song", "fang", "yahei", "wenquanyi")
    return [n for n in names if any(k in n.lower() for k in keys)]


def configure_chinese_font(preferred_font=None, verbose=True):
    """
    配置中文字体，尽量避免中文图例/标题乱码。
    若系统缺字体，会保留英文兼容字体并给出提示。
    """
    try:
        font_manager._load_fontmanager(try_read_cache=False)
    except Exception:
        pass

    available = sorted({f.name for f in font_manager.fontManager.ttflist})
    available_l = [n.lower() for n in available]

    def find_exact_or_contains(name):
        if not name:
            return None
        n = name.lower()
        # exact first
        for i, cand in enumerate(available_l):
            if cand == n:
                return available[i]
        # then contains
        for i, cand in enumerate(available_l):
            if n in cand:
                return available[i]
        return None

    chosen = find_exact_or_contains(preferred_font)
    if chosen is None:
        keyword_groups = [
            ("noto", "cjk"),
            ("source", "han"),
            ("yahei",),
            ("simhei",),
            ("wenquanyi",),
            ("pingfang",),
            ("song",),
            ("heiti",),
        ]
        for group in keyword_groups:
            for i, cand in enumerate(available_l):
                if all(k in cand for k in group):
                    chosen = available[i]
                    break
            if chosen is not None:
                break

    if chosen is not None:
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams["font.sans-serif"] = [chosen, "DejaVu Sans"]
        if verbose:
            print(f"[font] 使用中文字体: {chosen}")
    else:
        if verbose:
            cjk_list = list_candidate_cjk_fonts()
            if cjk_list:
                print("[warn] 未命中预设中文字体，但系统里检测到这些候选：")
                for n in cjk_list[:12]:
                    print(f"  - {n}")
                print("[hint] 可用 --font-family \"上面任一字体名\" 指定。")
            else:
                print(
                    "[warn] 未检测到常见中文字体；若中文显示为方块，可安装: sudo apt install fonts-noto-cjk"
                )
    plt.rcParams["axes.unicode_minus"] = False


def parse_drone_id(topic):
    if not topic.startswith("/drone"):
        return None
    rest = topic[len("/drone"):]
    digits = []
    for ch in rest:
        if ch.isdigit():
            digits.append(ch)
        else:
            break
    if not digits:
        return None
    return int("".join(digits))


def parse_float_list(raw_text):
    if raw_text is None:
        return []
    text = str(raw_text).strip()
    if not text:
        return []
    vals = []
    for token in text.replace("\n", ",").replace(" ", ",").split(","):
        tk = token.strip()
        if not tk:
            continue
        vals.append(float(tk))
    return vals


def parse_float_list_file(path):
    p = Path(path).expanduser().resolve()
    text = p.read_text(encoding="utf-8")
    return parse_float_list(text)


def extract_data_from_bag(bag_file_path):
    """
    从bag文件中提取目标、追踪者轨迹和障碍点云。
    优先使用 /global_map，避免重复叠加导致视觉噪声。
    """
    target_traj = {"time": [], "x": [], "y": [], "z": []}
    tracker_trajectories = {}
    obstacles = {"x": [], "y": [], "z": []}

    loaded_global_map = False
    loaded_any_map = False

    with rosbag.Bag(bag_file_path) as bag:
        for topic, msg, t in bag.read_messages():
            ts = t.to_sec()

            if topic == "/target/odom":
                target_traj["time"].append(ts)
                target_traj["x"].append(float(msg.pose.pose.position.x))
                target_traj["y"].append(float(msg.pose.pose.position.y))
                target_traj["z"].append(float(msg.pose.pose.position.z))
                continue

            # 只取无人机真实里程计轨迹，避免混入 target_dpf 的估计里程计
            if topic.startswith("/drone") and topic.endswith("/odom") and topic.count("/") == 2:
                drone_id = parse_drone_id(topic)
                if drone_id is None:
                    continue
                if drone_id not in tracker_trajectories:
                    tracker_trajectories[drone_id] = {"time": [], "x": [], "y": [], "z": []}
                tracker_trajectories[drone_id]["time"].append(ts)
                tracker_trajectories[drone_id]["x"].append(float(msg.pose.pose.position.x))
                tracker_trajectories[drone_id]["y"].append(float(msg.pose.pose.position.y))
                tracker_trajectories[drone_id]["z"].append(float(msg.pose.pose.position.z))
                continue

            # 优先加载一次 /global_map
            if topic == "/global_map" and not loaded_global_map:
                try:
                    for px, py, pz in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                        obstacles["x"].append(float(px))
                        obstacles["y"].append(float(py))
                        obstacles["z"].append(float(pz))
                    loaded_global_map = True
                    loaded_any_map = True
                except Exception as exc:
                    print(f"[warn] parse /global_map failed: {exc}")
                continue

            # 如果没有 /global_map，再回退到其它 map/cloud 话题（只加载一次）
            if (not loaded_any_map) and any(k in topic.lower() for k in ("map", "cloud", "obstacle")):
                try:
                    if hasattr(msg, "fields"):
                        for px, py, pz in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                            obstacles["x"].append(float(px))
                            obstacles["y"].append(float(py))
                            obstacles["z"].append(float(pz))
                        loaded_any_map = True
                    elif hasattr(msg, "points"):
                        for pt in msg.points:
                            obstacles["x"].append(float(pt.x))
                            obstacles["y"].append(float(pt.y))
                            obstacles["z"].append(float(pt.z))
                        loaded_any_map = True
                except Exception as exc:
                    print(f"[warn] parse map topic {topic} failed: {exc}")

    return target_traj, tracker_trajectories, obstacles


def nearest_pose_at_time(traj, t_query):
    if not traj["time"]:
        return None
    ts = np.asarray(traj["time"], dtype=float)
    idx = int(np.searchsorted(ts, t_query))
    if idx <= 0:
        k = 0
    elif idx >= len(ts):
        k = len(ts) - 1
    else:
        left = idx - 1
        right = idx
        k = left if abs(ts[left] - t_query) <= abs(ts[right] - t_query) else right
    return (
        float(traj["x"][k]),
        float(traj["y"][k]),
        float(traj["z"][k]),
        float(ts[k]),
    )


def estimate_motion_start_time(target_traj, move_thresh=0.3):
    """
    以 /target/odom 第一帧位置为基准，首次位移超过阈值时刻作为机动开始时间。
    若始终未超过阈值，回退到第一帧时间。
    """
    ts = np.asarray(target_traj["time"], dtype=float)
    if len(ts) == 0:
        return None

    x = np.asarray(target_traj["x"], dtype=float)
    y = np.asarray(target_traj["y"], dtype=float)
    z = np.asarray(target_traj["z"], dtype=float)
    p0 = np.array([x[0], y[0], z[0]], dtype=float)

    thr = max(0.0, float(move_thresh))
    for i in range(len(ts)):
        pi = np.array([x[i], y[i], z[i]], dtype=float)
        if float(np.linalg.norm(pi - p0)) >= thr:
            return float(ts[i])
    return float(ts[0])


def formation_quality_metric(target_xyz, tracker_xyz_list):
    if len(tracker_xyz_list) < 2:
        return float("inf")
    tx, ty, tz = target_xyz
    d_to_target = []
    for px, py, pz in tracker_xyz_list:
        d_to_target.append(math.sqrt((px - tx) ** 2 + (py - ty) ** 2 + (pz - tz) ** 2))

    pair_d = []
    for (ax, ay, az), (bx, by, bz) in combinations(tracker_xyz_list, 2):
        pair_d.append(math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2))

    if not pair_d:
        return float("inf")

    # 越小越好：到目标距离一致 + 编队边长一致
    return float(np.std(d_to_target) + 0.9 * np.std(pair_d))


def point_to_segment_distance_xy(px, py, ax, ay, bx, by):
    abx, aby = (bx - ax), (by - ay)
    apx, apy = (px - ax), (py - ay)
    ab2 = abx * abx + aby * aby
    if ab2 <= 1e-9:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)
    u = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    qx = ax + u * abx
    qy = ay + u * aby
    return math.sqrt((px - qx) ** 2 + (py - qy) ** 2)


def find_best_view_for_snapshot(target_traj, tracker_trajectories, snap_t, z_scale=1.0):
    """
    仅按你的定义：
    - 用三个追踪者构成平面
    - 用该平面的法线作为视线方向
    """
    tracker_ids = sorted(tracker_trajectories.keys())
    if len(tracker_ids) < 3:
        return None, None

    # 仅使用前三个追踪者（通常是 drone0/1/2）
    use_ids = tracker_ids[:3]
    pts = []
    for did in use_ids:
        p = nearest_pose_at_time(tracker_trajectories[did], snap_t)
        if p is None:
            return None, None
        px, py, pz, _ = p
        pts.append(np.array([px, py, pz], dtype=float))

    p1, p2, p3 = pts
    v1 = p2 - p1
    v2 = p3 - p1

    normal = np.cross(v1, v2)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-8:
        return None, None
    normal = normal / norm

    # 固定法向朝向，避免 +/-n 带来的视角翻转
    if normal[2] < 0.0:
        normal = -normal

    # 视线方向取 -normal（朝向平面）
    view_dir = -normal
    azim = float(np.degrees(np.arctan2(view_dir[1], view_dir[0])))
    elev = float(np.degrees(np.arctan2(view_dir[2], np.hypot(view_dir[0], view_dir[1]))))
    elev = float(np.clip(elev, -90.0, 90.0))

    # 视角中心取三个追踪者几何中心
    center = np.mean(np.vstack(pts), axis=0)
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2] * z_scale)
    return (elev, azim), (cx, cy, cz)


def select_good_snapshot_times(
    target_traj,
    tracker_trajectories,
    num_snapshots=5,
    min_gap_sec=3.5,
    waypoint1_y=-18.0,
    waypoint2_y=18.0,
    before_wp1_seconds=3.0,
):
    if not target_traj["time"]:
        return []
    target_times = np.asarray(target_traj["time"], dtype=float)
    if len(target_times) == 0:
        return []

    target_y = np.asarray(target_traj["y"], dtype=float)

    wp1_idx = np.argmin(np.abs(target_y - waypoint1_y))
    wp1_time = target_times[wp1_idx]

    wp2_idx = np.argmin(np.abs(target_y - waypoint2_y))
    wp2_time = target_times[wp2_idx]

    if wp1_time > wp2_time:
        wp1_time, wp2_time = wp2_time, wp1_time

    print(f"[DEBUG] WP1 time: {wp1_time:.2f}s (y={target_y[wp1_idx]:.1f}), WP2 time: {wp2_time:.2f}s (y={target_y[wp2_idx]:.1f})")

    chosen = []

    # 在第一个航点前12秒采样一个时刻
    before_time = wp1_time - before_wp1_seconds
    if before_time >= target_times[0]:
        chosen.append(before_time)

    # 在两个航点时间段内采样：
    # 第一段间隔拉长，后续区间均分
    remaining_snapshots = num_snapshots - len(chosen)
    if wp2_time > wp1_time and remaining_snapshots > 0:
        span = float(wp2_time - wp1_time)
        if remaining_snapshots == 1:
            between_times = [float(wp2_time)]
        elif remaining_snapshots == 2:
            between_times = [float(wp1_time), float(wp2_time)]
        else:
            # 第一段使用更大的时间占比（40%），后面均分剩余60%
            first_gap_ratio = 0.50
            first_anchor = float(wp1_time + span * first_gap_ratio)
            tail = np.linspace(first_anchor, wp2_time, remaining_snapshots - 1)
            between_times = [float(wp1_time)] + [float(t) for t in tail]

        for t in between_times:
            chosen.append(float(t))

    return sorted(chosen[:num_snapshots])

def draw_quad_marker(ax, x, y, z, arm=0.55, rotor_r=0.11, color="#1f77b4", lw=1.2, alpha=1.0, zorder=5):
    ax.plot([x - arm, x + arm], [y, y], [z, z], color=color, linewidth=lw, alpha=alpha, zorder=zorder)
    ax.plot([x, x], [y - arm, y + arm], [z, z], color=color, linewidth=lw, alpha=alpha, zorder=zorder)
    th = np.linspace(0.0, 2.0 * np.pi, 32)
    rotor_centers = [(x - arm, y), (x + arm, y), (x, y - arm), (x, y + arm)]
    for cx, cy in rotor_centers:
        xr = cx + rotor_r * np.cos(th)
        yr = cy + rotor_r * np.sin(th)
        zr = np.full_like(th, z)
        ax.plot(xr, yr, zr, color=color, linewidth=max(0.8, lw * 0.9), alpha=alpha, zorder=zorder)


def densify_obstacle_points(
    obstacles,
    max_base_points=120000,
    densify_factor=2.5,
    jitter_xy=0.06,
    jitter_z=0.03,
    seed=7,
):
    if not obstacles["x"]:
        return np.empty((0, 3), dtype=float)

    pts = np.column_stack(
        (
            np.asarray(obstacles["x"], dtype=float),
            np.asarray(obstacles["y"], dtype=float),
            np.asarray(obstacles["z"], dtype=float),
        )
    )

    rng = np.random.default_rng(seed)

    # 避免固定步长抽样导致环形障碍出现“条纹空洞”
    if len(pts) > max_base_points:
        idx = rng.choice(len(pts), size=max_base_points, replace=False)
        pts = pts[idx]

    densify_factor = max(1.0, float(densify_factor))
    extra_n = int(round(len(pts) * (densify_factor - 1.0)))
    if extra_n <= 0:
        return pts

    base_idx = rng.integers(0, len(pts), size=extra_n)
    base = pts[base_idx]
    jitter = np.column_stack(
        (
            rng.normal(0.0, jitter_xy, size=extra_n),
            rng.normal(0.0, jitter_xy, size=extra_n),
            rng.normal(0.0, jitter_z, size=extra_n),
        )
    )
    dense = np.vstack((pts, base + jitter))
    return dense


def filter_random_obstacle_region(points_xyz, target_xy, keep_dist=8.0):
    """
    仅保留靠近目标运动走廊的障碍点，常用于去掉远处宽墙/柱子。
    """
    if len(points_xyz) == 0:
        return points_xyz
    if len(target_xy) == 0:
        return points_xyz

    keep_dist2 = float(keep_dist) * float(keep_dist)
    path_xy = np.asarray(target_xy, dtype=float)
    if len(path_xy) > 320:
        step = max(1, len(path_xy) // 320)
        path_xy = path_xy[::step]

    pts = np.asarray(points_xyz, dtype=float)
    kept = np.zeros(len(pts), dtype=bool)

    chunk = 12000
    for s in range(0, len(pts), chunk):
        e = min(len(pts), s + chunk)
        pxy = pts[s:e, :2]
        dx = pxy[:, None, 0] - path_xy[None, :, 0]
        dy = pxy[:, None, 1] - path_xy[None, :, 1]
        d2 = dx * dx + dy * dy
        min_d2 = np.min(d2, axis=1)
        kept[s:e] = (min_d2 <= keep_dist2)

    filtered = pts[kept]
    if len(filtered) < 1000:
        return pts
    return filtered


def set_equal_3d_axes(ax, xs, ys, zs, pad_ratio=0.01):
    if len(xs) == 0:
        return
    x_min, x_max = float(np.min(xs)), float(np.max(xs))
    y_min, y_max = float(np.min(ys)), float(np.max(ys))
    z_min, z_max = float(np.min(zs)), float(np.max(zs))

    span = max(x_max - x_min, y_max - y_min, z_max - z_min)
    span = max(span, 1.0)
    pad = span * pad_ratio
    half = span * 0.5 + pad

    mid_x = 0.5 * (x_min + x_max)
    mid_y = 0.5 * (y_min + y_max)
    mid_z = 0.5 * (z_min + z_max)

    ax.set_xlim(mid_x - half, mid_x + half)
    ax.set_ylim(mid_y - half, mid_y + half)
    ax.set_zlim(mid_z - half, mid_z + half)


def attach_roll_projection(ax, roll_deg=0.0):
    """
    Matplotlib 3.1 没有原生 roll 参数。
    通过对 clip-space 的 x/y 做旋转来实现绕视线滚转。
    """
    state = {"roll_deg": float(roll_deg)}
    base_get_proj = ax.get_proj

    def get_proj_with_roll():
        proj = base_get_proj()
        r = float(state["roll_deg"])
        if abs(r) < 1e-9:
            return proj
        th = np.deg2rad(r)
        c = float(np.cos(th))
        s = float(np.sin(th))
        rot = np.array(
            [
                [c, -s, 0.0, 0.0],
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return rot @ proj

    ax.get_proj = get_proj_with_roll
    return state


def create_3d_visualization(
    bag_file_path,
    output_path,
    num_snapshots=5,
    elev=24.0,
    azim=-58.0,
    roll=0.0,
    banner_width=24.0,
    banner_height=8.0,
    z_scale=1.0,
    obstacle_max_points=120000,
    obstacle_densify_factor=2.5,
    obstacle_jitter_xy=0.06,
    obstacle_jitter_z=0.03,
    obstacle_point_size=1.8,
    keep_random_obstacles_only=True,
    random_region_dist=8.0,
    focus_wp1=(7.0, -6.5),
    focus_wp2=(11.0, -6.5),
    focus_seg_width=2.6,
    focus_seg_ratio=0.60,
    font_family="",
    show_axes=False,
    interactive=False,
    show=False,
    show_trajectories=True,
    snapshot_times_abs=None,
    snapshot_times_rel=None,
    snapshot_times_file="",
    time_reference="motion_start",
    time_reference_move_thresh=0.3,
):
    configure_chinese_font(font_family if font_family else None)
    target_traj, tracker_trajectories, obstacles = extract_data_from_bag(bag_file_path)

    if not target_traj["time"]:
        raise RuntimeError("No target trajectory found in bag.")
    if not tracker_trajectories:
        raise RuntimeError("No tracker odom trajectories found in bag.")

    fig = plt.figure(figsize=(banner_width, banner_height))
    ax = fig.add_subplot(111, projection="3d")
    
    ax.view_init(elev=elev, azim=azim)
    roll_state = attach_roll_projection(ax, roll_deg=float(roll))

    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(False)

    obs_dense = densify_obstacle_points(
        obstacles,
        max_base_points=obstacle_max_points,
        densify_factor=obstacle_densify_factor,
        jitter_xy=obstacle_jitter_xy,
        jitter_z=obstacle_jitter_z,
    )
    if keep_random_obstacles_only and len(obs_dense) > 0:
        target_xy = np.column_stack((np.asarray(target_traj["x"]), np.asarray(target_traj["y"])))
        obs_dense = filter_random_obstacle_region(
            obs_dense,
            target_xy,
            keep_dist=float(random_region_dist),
        )

    if len(obs_dense) > 0:
        obs_dense_plot = obs_dense.copy()
        obs_dense_plot[:, 2] *= z_scale
        obs_sc = ax.scatter(
            obs_dense_plot[:, 0],
            obs_dense_plot[:, 1],
            obs_dense_plot[:, 2],
            c=obs_dense_plot[:, 2],
            cmap="plasma",
            s=obstacle_point_size,
            alpha=0.70,
            linewidths=0.0,
            depthshade=False,
            zorder=1,
        )
        cbar = plt.colorbar(obs_sc, ax=ax, fraction=0.022, pad=0.01)
        cbar.set_label("障碍高度 z (m)", fontsize=10)

    # 选择快照时刻：优先手工指定，其次自动选帧
    target_times_all = np.asarray(target_traj["time"], dtype=float)
    t_min = float(target_times_all[0])
    t_max = float(target_times_all[-1])
    snapshot_times = []

    manual_abs = []
    manual_rel = []
    if snapshot_times_file:
        manual_rel = parse_float_list_file(snapshot_times_file)
    if snapshot_times_abs:
        manual_abs = [float(v) for v in snapshot_times_abs]
    if snapshot_times_rel:
        manual_rel = [float(v) for v in snapshot_times_rel]

    if manual_abs:
        snapshot_times = sorted([v for v in manual_abs if t_min <= v <= t_max])
        print(f"[info] 使用手工绝对时刻，共 {len(snapshot_times)} 个。")
    elif manual_rel:
        if time_reference == "motion_start":
            tref = estimate_motion_start_time(
                target_traj, move_thresh=float(time_reference_move_thresh)
            )
            ref_name = f"motion_start(thr={time_reference_move_thresh:.2f}m)"
        else:
            tref = t_min
            ref_name = "first_target_odom"
        snapshot_times = sorted([tref + float(v) for v in manual_rel if v >= 0.0])
        snapshot_times = [t for t in snapshot_times if t_min <= t <= t_max]
        print(f"[info] 使用手工相对时刻，共 {len(snapshot_times)} 个，参考={ref_name} @ {tref:.3f}s")
    else:
        snapshot_times = select_good_snapshot_times(
            target_traj,
            tracker_trajectories,
            num_snapshots=num_snapshots,
            min_gap_sec=3.5,
            waypoint1_y=focus_wp1[1],
            waypoint2_y=focus_wp2[1],
            before_wp1_seconds=7.0,
        )

    if len(snapshot_times) == 0:
        raise RuntimeError("未得到有效快照时刻，请检查 --snapshot-times-* 参数是否落在 bag 时间范围内。")

    t0 = float(target_traj["time"][0])
    # 调试信息
    print(f"[DEBUG] focus_wp1={focus_wp1}, focus_wp2={focus_wp2}")
    print(f"[DEBUG] waypoint1_y={focus_wp1[1]}, waypoint2_y={focus_wp2[1]}")
    print(f"[DEBUG] snapshot_times={snapshot_times}")
    
    # 计算每个快照的推荐视角
    print("\n=== 推荐视角参数 ===")
    for i, snap_t in enumerate(snapshot_times):
        view_params, center = find_best_view_for_snapshot(
            target_traj, tracker_trajectories, snap_t, z_scale
        )
        if view_params:
            tgt = nearest_pose_at_time(target_traj, snap_t)
            t_rel = snap_t - t0
            print(f"Snapshot {i+1}: t={t_rel:.1f}s, elev={view_params[0]:.1f}, azim={view_params[1]:.1f}, center=({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f})")
    print("====================\n")
    
    # 轨迹（平滑处理）
    target_times = np.asarray(target_traj["time"], dtype=float)
    target_x = np.asarray(target_traj["x"], dtype=float)
    target_y = np.asarray(target_traj["y"], dtype=float)
    target_z = np.asarray(target_traj["z"], dtype=float) * z_scale
    
    # 使用航点前快照时刻作为轨迹起点
    if len(snapshot_times) > 0:
        start_time = snapshot_times[0]
    else:
        start_time = target_times[0]
    
    start_idx = np.searchsorted(target_times, start_time)
    
    target_x = target_x[start_idx:]
    target_y = target_y[start_idx:]
    target_z = target_z[start_idx:]
    
    if show_trajectories:
        if len(target_x) > 3:
            target_x_smooth = gaussian_filter1d(target_x, sigma=1.5)
            target_y_smooth = gaussian_filter1d(target_y, sigma=1.5)
            target_z_smooth = gaussian_filter1d(target_z, sigma=1.5)
        else:
            target_x_smooth, target_y_smooth, target_z_smooth = target_x, target_y, target_z
        ax.plot(target_x_smooth, target_y_smooth, target_z_smooth, "-", color="#ff8c00", linewidth=2.8, alpha=0.95, zorder=3)

        for _, traj in sorted(tracker_trajectories.items()):
            traj_times = np.asarray(traj["time"], dtype=float)
            tx = np.asarray(traj["x"], dtype=float)
            ty = np.asarray(traj["y"], dtype=float)
            tz = np.asarray(traj["z"], dtype=float) * z_scale
            
            traj_start_idx = np.searchsorted(traj_times, start_time)
            tx = tx[traj_start_idx:]
            ty = ty[traj_start_idx:]
            tz = tz[traj_start_idx:]
            
            if len(tx) > 3:
                tx_smooth = gaussian_filter1d(tx, sigma=1.5)
                ty_smooth = gaussian_filter1d(ty, sigma=1.5)
                tz_smooth = gaussian_filter1d(tz, sigma=1.5)
            else:
                tx_smooth, ty_smooth, tz_smooth = tx, ty, tz
            ax.plot(tx_smooth, ty_smooth, tz_smooth, "-", color="#1f77b4", linewidth=1.8, alpha=0.60, zorder=2)

    for snap_t in snapshot_times:
        tgt = nearest_pose_at_time(target_traj, snap_t)
        if tgt is None:
            continue
        tx, ty, tz, _ = tgt
        tz *= z_scale

        draw_quad_marker(ax, tx, ty, tz, arm=0.60, rotor_r=0.13, color="#e74c3c", lw=1.4, alpha=1.0, zorder=5)
        ax.scatter([tx], [ty], [tz], color="#e74c3c", s=34, alpha=1.0, zorder=7)

        tracker_pts = []
        for _, traj in sorted(tracker_trajectories.items()):
            p = nearest_pose_at_time(traj, snap_t)
            if p is None:
                continue
            px, py, pz, _ = p
            pz *= z_scale
            tracker_pts.append((px, py, pz))
            draw_quad_marker(ax, px, py, pz, arm=0.55, rotor_r=0.11, color="#1f77b4", lw=1.3, alpha=1.0, zorder=5)
            ax.scatter([px], [py], [pz], color="#1f77b4", s=26, alpha=1.0, zorder=7)

        # 编队连线：追踪者之间互连
        for (ax0, ay0, az0), (bx0, by0, bz0) in combinations(tracker_pts, 2):
            ax.plot(
                [ax0, bx0],
                [ay0, by0],
                [az0, bz0],
                "--",
                color="#1f77b4",
                linewidth=1.2,
                alpha=0.70,
                zorder=6,
            )

        # ax.text(
        #     tx,
        #     ty,
        #     tz + 0.85,
        #     f"t={snap_t - t0:.1f}s",
        #     fontsize=9,
        #     bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="gray", alpha=0.75),
        #     zorder=20,
        # )

    # 坐标范围
    all_x = list(target_x)
    all_y = list(target_y)
    all_z = list(target_z)
    for traj in tracker_trajectories.values():
        all_x.extend(traj["x"])
        all_y.extend(traj["y"])
        all_z.extend((np.asarray(traj["z"], dtype=float) * z_scale).tolist())
    if len(obs_dense) > 0:
        all_x.extend(obs_dense[:, 0].tolist())
        all_y.extend(obs_dense[:, 1].tolist())
        all_z.extend((obs_dense[:, 2] * z_scale).tolist())

    set_equal_3d_axes(ax, np.asarray(all_x), np.asarray(all_y), np.asarray(all_z))

    ax.set_title("1.5 m/s 目标速度下的多时刻编队快照", fontsize=16, fontweight="bold")
    if show_axes:
        ax.set_xlabel("X (m)", fontsize=12)
        ax.set_ylabel("Y (m)", fontsize=12)
        ax.set_zlabel("Z (m)", fontsize=12)
    else:
        ax.set_axis_off()

    legend_handles = [
        Line2D([0], [0], color="#ff8c00", lw=2.8),
        Line2D([0], [0], color="#1f77b4", lw=1.8),
        QuadLegendProxy("#e74c3c"),
        QuadLegendProxy("#1f77b4"),
        Line2D([0], [0], color="#1f77b4", lw=1.2, linestyle="--"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#42a5f5", markersize=6, alpha=0.7, lw=0),
    ]
    legend_labels = [
        "目标轨迹",
        "追踪者轨迹",
        "目标四旋翼",
        "追踪者四旋翼",
        "编队连线",
        "障碍点云",
    ]
    ax.legend(
        handles=legend_handles,
        labels=legend_labels,
        handler_map={QuadLegendProxy: QuadLegendHandler()},
        loc="upper left",
        bbox_to_anchor=(0.01, 0.98),
        framealpha=0.90,
    )

    plt.tight_layout()

    if interactive:
        save_path = Path(output_path).expanduser().resolve()

        def on_key(event):
            if event.key in ("s", "S"):
                save_path.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(str(save_path), dpi=320, bbox_inches="tight")
                print(f"[ok] snapshot saved: {save_path}")
            elif event.key in ("p", "P"):
                print(
                    f"[view] elev={ax.elev:.2f}, azim={ax.azim:.2f}, roll={roll_state['roll_deg']:.2f}"
                )
            elif event.key in ("j", "J"):
                roll_state["roll_deg"] -= 2.0
                fig.canvas.draw_idle()
                print(f"[view] roll={roll_state['roll_deg']:.2f}")
            elif event.key in ("k", "K"):
                roll_state["roll_deg"] += 2.0
                fig.canvas.draw_idle()
                print(f"[view] roll={roll_state['roll_deg']:.2f}")
            elif event.key in ("q", "Q", "escape"):
                plt.close(fig)

        fig.canvas.mpl_connect("key_press_event", on_key)
        print(
            "[interactive] 鼠标拖动旋转视角，滚轮缩放；按 j/k 调整 roll；按 s 保存，按 p 打印视角，按 q 退出。"
        )
        plt.show()
        plt.close(fig)
        return

    output_path = Path(output_path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=320, bbox_inches="tight")
    print(f"[ok] 3D banner saved: {output_path}")

    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Render multi-snapshot 3D formation banner from rosbag.")
    parser.add_argument(
        "bag",
        nargs="?",
        default="/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/experiment_results/三角编队_逃速1.5_障碍真实速度/run_001/tracking.bag",
    )
    parser.add_argument(
        "--output",
        default="/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/tracking_1p5ms_banner.png",
    )
    parser.add_argument("--num-snapshots", type=int, default=5)
    parser.add_argument("--elev", type=float, default=72.0)
    parser.add_argument("--azim", type=float, default=180.0)
    parser.add_argument("--roll", type=float, default=0.0, help="Rotate camera around viewing axis (degrees).")
    parser.add_argument("--banner-width", type=float, default=24.0)
    parser.add_argument("--banner-height", type=float, default=8.0)
    parser.add_argument("--z-scale", type=float, default=4.0)
    parser.add_argument("--obstacle-max-points", type=int, default=120000)
    parser.add_argument("--obstacle-densify-factor", type=float, default=2.5)
    parser.add_argument("--obstacle-jitter-xy", type=float, default=0.06)
    parser.add_argument("--obstacle-jitter-z", type=float, default=0.03)
    parser.add_argument("--obstacle-point-size", type=float, default=1.8)
    parser.add_argument("--keep-random-obstacles-only", action="store_true", default=True)
    parser.add_argument("--no-random-obstacles-only", dest="keep_random_obstacles_only", action="store_false")
    parser.add_argument("--random-region-dist", type=float, default=8.0,
                        help="Keep obstacle points within this XY distance to target trajectory.")
    parser.add_argument("--focus-wp1-x", type=float, default=7.0)
    parser.add_argument("--focus-wp1-y", type=float, default=-6.5)
    parser.add_argument("--focus-wp2-x", type=float, default=11.0)
    parser.add_argument("--focus-wp2-y", type=float, default=-6.5)
    parser.add_argument("--focus-seg-width", type=float, default=2.6,
                        help="Corridor half-width for dense snapshot sampling around wp1->wp2 segment.")
    parser.add_argument("--focus-seg-ratio", type=float, default=0.60,
                        help="Fraction of snapshots preferentially selected in wp1->wp2 segment.")
    parser.add_argument("--font-family", default="",
                        help="Chinese font family for labels, e.g. 'Noto Sans CJK SC'.")
    parser.add_argument("--list-cjk-fonts", action="store_true", help="Print detected CJK-related font names and exit.")
    parser.add_argument("--show-axes", action="store_true", help="Show xyz axes (hidden by default).")
    parser.add_argument("--hide-trajectories", action="store_true", help="Hide trajectory lines.")
    parser.add_argument(
        "--snapshot-times-rel",
        default="",
        help="手工指定快照相对时间(秒)，逗号分隔，如 18.9,25.9,36.5",
    )
    parser.add_argument(
        "--snapshot-times-abs",
        default="",
        help="手工指定快照绝对时间戳(秒)，逗号分隔（bag header stamp 口径）",
    )
    parser.add_argument(
        "--snapshot-times-file",
        default="",
        help="从文件读取快照相对时间(秒)，可逗号/空格/换行分隔",
    )
    parser.add_argument(
        "--time-reference",
        choices=["motion_start", "first_target_odom"],
        default="motion_start",
        help="snapshot-times-rel 的参考零时刻",
    )
    parser.add_argument(
        "--time-reference-move-thresh",
        type=float,
        default=0.3,
        help="time-reference=motion_start 时的位移阈值(m)",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open interactive window; press j/k to tune roll, s to save current view.",
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    if args.list_cjk_fonts:
        fonts = list_candidate_cjk_fonts()
        if not fonts:
            print("[info] 未检测到 CJK 相关字体名。")
        else:
            print("[info] 检测到的 CJK 相关字体名：")
            for n in fonts:
                print(f"  - {n}")
        return

    create_3d_visualization(
        bag_file_path=args.bag,
        output_path=args.output,
        num_snapshots=max(1, int(args.num_snapshots)),
        elev=float(args.elev),
        azim=float(args.azim),
        roll=float(args.roll),
        banner_width=float(args.banner_width),
        banner_height=float(args.banner_height),
        z_scale=float(args.z_scale),
        obstacle_max_points=max(1000, int(args.obstacle_max_points)),
        obstacle_densify_factor=max(1.0, float(args.obstacle_densify_factor)),
        obstacle_jitter_xy=max(0.0, float(args.obstacle_jitter_xy)),
        obstacle_jitter_z=max(0.0, float(args.obstacle_jitter_z)),
        obstacle_point_size=max(0.2, float(args.obstacle_point_size)),
        keep_random_obstacles_only=bool(args.keep_random_obstacles_only),
        random_region_dist=max(0.5, float(args.random_region_dist)),
        focus_wp1=(float(args.focus_wp1_x), float(args.focus_wp1_y)),
        focus_wp2=(float(args.focus_wp2_x), float(args.focus_wp2_y)),
        focus_seg_width=max(0.2, float(args.focus_seg_width)),
        focus_seg_ratio=max(0.0, min(1.0, float(args.focus_seg_ratio))),
        font_family=str(args.font_family),
        show_axes=bool(args.show_axes),
        show_trajectories=not bool(args.hide_trajectories),
        interactive=bool(args.interactive),
        show=bool(args.show),
        snapshot_times_abs=parse_float_list(args.snapshot_times_abs),
        snapshot_times_rel=parse_float_list(args.snapshot_times_rel),
        snapshot_times_file=str(args.snapshot_times_file),
        time_reference=str(args.time_reference),
        time_reference_move_thresh=max(0.0, float(args.time_reference_move_thresh)),
    )


if __name__ == "__main__":
    main()
