#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Render a raw 3D scenario snapshot from rosbag:
- obstacles as darker point cloud
- trackers as blue quadrotor markers
- target motion shown by light gradient arrows (trend only)
"""

import argparse
import ast
import bisect
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.lines import Line2D
from matplotlib.legend_handler import HandlerBase
from matplotlib.patches import Circle, FancyArrowPatch, Polygon as MplPolygon
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import yaml

plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC',
    'WenQuanYi Zen Hei',
    'Microsoft YaHei',
    'SimHei',
    'DejaVu Sans',
]


def get_chinese_font():
    candidates = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        '/usr/share/fonts/truetype/arphic/ukai.ttc',
        '/usr/share/fonts/truetype/arphic/uming.ttc',
    ]
    for p in candidates:
        if Path(p).exists():
            return fm.FontProperties(fname=p)
    return None


class DroneLegendHandle:
    def __init__(self, color, count=1):
        self.color = color
        self.count = int(max(1, count))


class TrendLegendHandle:
    pass


class HandlerDroneLegend(HandlerBase):
    def _draw_one_drone(self, artists, trans, cx, cy, size, color):
        arm = size * 0.44
        rotor_r = size * 0.15
        artists.append(Line2D([cx - arm, cx + arm], [cy, cy], color=color, linewidth=1.55, transform=trans))
        artists.append(Line2D([cx, cx], [cy - arm, cy + arm], color=color, linewidth=1.55, transform=trans))
        for rx, ry in ((cx - arm, cy), (cx + arm, cy), (cx, cy - arm), (cx, cy + arm)):
            artists.append(Circle((rx, ry), rotor_r, fill=False, ec=color, lw=1.2, alpha=0.98, transform=trans))

    def create_artists(self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans):
        artists = []
        n = int(max(1, orig_handle.count))
        if n == 1:
            centers = [xdescent + 0.5 * width]
        else:
            centers = np.linspace(xdescent + 0.19 * width, xdescent + 0.81 * width, n)
        cy = ydescent + 0.46 * height
        size = min(width, height) * (0.49 if n >= 3 else 0.58)
        for cx in centers:
            self._draw_one_drone(artists, trans, float(cx), float(cy), float(size), orig_handle.color)
        return artists


class HandlerTrendLegend(HandlerBase):
    def create_artists(self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans):
        artists = []
        y0 = ydescent + 0.46 * height
        x0 = xdescent + 0.10 * width
        x1 = xdescent + 0.92 * width
        col_base = (0.66, 0.66, 0.66, 1.0)
        col_tip = (0.46, 0.46, 0.46, 1.0)
        body = FancyArrowPatch(
            (x0, y0), (x1, y0),
            connectionstyle='arc3,rad=-0.22',
            arrowstyle='simple,head_length=8,head_width=7,tail_width=2.6',
            linewidth=0.0,
            fc=(col_base[0], col_base[1], col_base[2], 0.72),
            ec='none',
            transform=trans,
        )
        highlight = FancyArrowPatch(
            (x0 + 0.07 * width, y0 + 0.02 * height), (x1 - 0.08 * width, y0 + 0.01 * height),
            connectionstyle='arc3,rad=-0.18',
            arrowstyle='-|>',
            mutation_scale=7.5,
            linewidth=1.2,
            color=(col_tip[0], col_tip[1], col_tip[2], 0.78),
            transform=trans,
        )
        artists.append(body)
        artists.append(highlight)
        return artists


def parse_drone_id(topic):
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


def quat_xyzw_to_rotmat(qx, qy, qz, qw):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def load_camera_model(camera_yaml_path):
    p = Path(camera_yaml_path).resolve()
    with p.open('r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    width = float(cfg.get('cam_width', 640.0))
    height = float(cfg.get('cam_height', 480.0))
    fx = float(cfg.get('cam_fx', 387.229248046875))
    fy = float(cfg.get('cam_fy', 387.229248046875))
    cam_range = float(cfg.get('camera_range', 7.0))
    cam2body_R = np.asarray(cfg.get('cam2body_R', [
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0
    ]), dtype=np.float64).reshape(3, 3)
    cam2body_p = np.asarray(cfg.get('cam2body_p', [0.0, 0.0, 0.05]), dtype=np.float64).reshape(3)
    h_fov = 2.0 * math.atan(width / (2.0 * max(1e-6, fx)))
    v_fov = 2.0 * math.atan(height / (2.0 * max(1e-6, fy)))
    return {
        'yaml_path': str(p),
        'range': cam_range,
        'h_fov': h_fov,
        'v_fov': v_fov,
        'cam2body_R': cam2body_R,
        'cam2body_p': cam2body_p,
    }


def build_camera_fov_frustum_world(
    drone_pos,
    drone_quat_xyzw,
    camera_model,
    range_scale=1.0,
    size_scale=1.0,
    forward_offset=0.0,
):
    p_b = np.asarray(drone_pos, dtype=np.float64)
    qx, qy, qz, qw = [float(v) for v in drone_quat_xyzw]
    R_wb = quat_xyzw_to_rotmat(qx, qy, qz, qw)
    R_bc = camera_model['cam2body_R']
    p_bc = camera_model['cam2body_p']
    R_wc = R_wb.dot(R_bc)
    p_cw = p_b + R_wb.dot(p_bc)

    rng = float(camera_model['range']) * float(range_scale)
    h = float(camera_model['h_fov'])
    v = float(camera_model['v_fov'])
    hx = rng * math.tan(0.5 * h) * float(size_scale)
    hy = rng * math.tan(0.5 * v) * float(size_scale)

    apex = p_cw + R_wc.dot(np.array([0.0, 0.0, float(forward_offset)], dtype=np.float64))

    far_corners_c = np.array([
        [-hx, -hy, rng],
        [hx, -hy, rng],
        [hx, hy, rng],
        [-hx, hy, rng],
    ], dtype=np.float64)
    far_corners_w = [apex + R_wc.dot(pt) for pt in far_corners_c]
    return apex, far_corners_w


def draw_drone_fov(
    ax,
    drone_pos,
    drone_quat_xyzw,
    camera_model,
    color=(0.52, 0.80, 0.98, 0.11),
    range_scale=1.0,
    size_scale=1.0,
    forward_offset=0.0,
):
    apex, far = build_camera_fov_frustum_world(
        drone_pos,
        drone_quat_xyzw,
        camera_model,
        range_scale=range_scale,
        size_scale=size_scale,
        forward_offset=forward_offset,
    )

    side_faces = []
    for i in range(4):
        j = (i + 1) % 4
        side_faces.append([apex, far[i], far[j]])
    sides = Poly3DCollection(side_faces, facecolors=[color] * len(side_faces), edgecolors='none')
    ax.add_collection3d(sides)

    far_face = Poly3DCollection(
        [[far[0], far[1], far[2], far[3]]],
        facecolors=[(color[0], color[1], color[2], max(0.06, color[3] * 0.85))],
        edgecolors='none',
    )
    ax.add_collection3d(far_face)

    edge_c = (0.52, 0.80, 0.98, 0.22)
    for i in range(4):
        j = (i + 1) % 4
        pi, pj = far[i], far[j]
        ax.plot([pi[0], pj[0]], [pi[1], pj[1]], [pi[2], pj[2]], color=edge_c, linewidth=0.7)
    for i in range(4):
        pi = far[i]
        ax.plot([apex[0], pi[0]], [apex[1], pi[1]], [apex[2], pi[2]], color=edge_c, linewidth=0.6)

def _eval_numeric_expr(node, env):
    if isinstance(node, ast.Constant):
        if isinstance(node.value, (int, float)):
            return float(node.value)
        raise ValueError('Non numeric constant')
    if isinstance(node, ast.Name):
        if node.id in env:
            return float(env[node.id])
        raise ValueError('Unknown name {}'.format(node.id))
    if isinstance(node, ast.UnaryOp):
        v = _eval_numeric_expr(node.operand, env)
        if isinstance(node.op, ast.UAdd):
            return v
        if isinstance(node.op, ast.USub):
            return -v
    if isinstance(node, ast.BinOp):
        a = _eval_numeric_expr(node.left, env)
        b = _eval_numeric_expr(node.right, env)
        if isinstance(node.op, ast.Add):
            return a + b
        if isinstance(node.op, ast.Sub):
            return a - b
        if isinstance(node.op, ast.Mult):
            return a * b
        if isinstance(node.op, ast.Div):
            return a / b
    raise ValueError('Unsupported expression')


def _eval_waypoints_node(node, env):
    if not isinstance(node, (ast.List, ast.Tuple)):
        raise ValueError('WAYPOINTS must be list/tuple')
    out = []
    for elt in node.elts:
        if not isinstance(elt, (ast.List, ast.Tuple)) or len(elt.elts) < 2:
            continue
        x = _eval_numeric_expr(elt.elts[0], env)
        y = _eval_numeric_expr(elt.elts[1], env)
        out.append((float(x), float(y)))
    return out


def load_waypoints_from_script(script_path):
    p = Path(script_path).resolve()
    if not p.exists():
        return []
    src = p.read_text(encoding='utf-8')
    tree = ast.parse(src, filename=str(p))
    env = {}
    waypoints = []
    for stmt in tree.body:
        if not isinstance(stmt, ast.Assign) or len(stmt.targets) != 1:
            continue
        target = stmt.targets[0]
        if not isinstance(target, ast.Name):
            continue
        name = target.id
        try:
            if name == 'WAYPOINTS':
                waypoints = _eval_waypoints_node(stmt.value, env)
            else:
                env[name] = _eval_numeric_expr(stmt.value, env)
        except Exception:
            continue
    return waypoints


def draw_quad_marker(ax, x, y, z, arm=0.55, rotor_r=0.11, color='#1f77b4', lw=1.2, alpha=1.0):
    # Cross arms
    ax.plot([x - arm, x + arm], [y, y], [z, z], color=color, linewidth=lw, alpha=alpha)
    ax.plot([x, x], [y - arm, y + arm], [z, z], color=color, linewidth=lw, alpha=alpha)
    # Four rotor circles in XY plane
    th = np.linspace(0.0, 2.0 * np.pi, 32)
    rotor_centers = [(x - arm, y), (x + arm, y), (x, y - arm), (x, y + arm)]
    for cx, cy in rotor_centers:
        xr = cx + rotor_r * np.cos(th)
        yr = cy + rotor_r * np.sin(th)
        zr = np.full_like(th, z)
        ax.plot(xr, yr, zr, color=color, linewidth=max(0.8, lw * 0.9), alpha=alpha)


def densify_obstacle_points_np(
    pts_xyz,
    max_base_points=120000,
    densify_factor=2.5,
    jitter_xy=0.06,
    jitter_z=0.03,
    seed=7,
):
    pts = np.asarray(pts_xyz, dtype=np.float64)
    if len(pts) == 0:
        return np.empty((0, 3), dtype=np.float64)

    rng = np.random.default_rng(int(seed))

    # 与 formation_shots 一致：随机采样，避免环障碍条纹空洞
    if len(pts) > int(max_base_points):
        idx = rng.choice(len(pts), size=int(max_base_points), replace=False)
        pts = pts[idx]

    densify_factor = max(1.0, float(densify_factor))
    extra_n = int(round(len(pts) * (densify_factor - 1.0)))
    if extra_n <= 0:
        return pts

    base_idx = rng.integers(0, len(pts), size=extra_n)
    base = pts[base_idx]
    jitter = np.column_stack(
        (
            rng.normal(0.0, float(jitter_xy), size=extra_n),
            rng.normal(0.0, float(jitter_xy), size=extra_n),
            rng.normal(0.0, float(jitter_z), size=extra_n),
        )
    )
    return np.vstack((pts, base + jitter))


def sample_history_at_or_before(ts_list, value_list, query_ts):
    if len(ts_list) == 0:
        return None, None
    i = bisect.bisect_right(ts_list, float(query_ts)) - 1
    if i < 0:
        i = 0
    return ts_list[i], value_list[i]


def pick_snapshot_and_data(
    bag_path,
    drone_ids,
    desired_side,
    min_target_move,
    min_elapsed,
    max_obs_points,
    crop_obstacles=False,
    crop_margin_xy=10.0,
    crop_margin_z=4.0,
    state_lookback_sec=3.0,
):
    topics = ['/global_map', '/target/odom']
    for did in drone_ids:
        topics.append('/drone{}/odom'.format(did))
        topics.append('/drone{}/drone{}_target_dpf/search_state'.format(did, did))

    search_state = {}
    drone_pos = {}
    drone_vel = {}
    drone_quat = {}
    target_pos = None
    target_start = None
    start_ts = None
    traj = []
    target_hist_ts = []
    target_hist_pos = []
    obstacle_pts = None
    drone_hist = {did: {'ts': [], 'pos': [], 'quat': []} for did in drone_ids}

    cands = []  # (score, ts, positions_dict, target_pos, quat_dict)
    fallback = []

    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if start_ts is None:
                start_ts = ts

            if topic == '/global_map' and obstacle_pts is None:
                pts = []
                for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    pts.append((float(x), float(y), float(z)))
                if pts:
                    obstacle_pts = np.asarray(pts, dtype=np.float32)
                continue

            if topic == '/target/odom':
                p = msg.pose.pose.position
                target_pos = (float(p.x), float(p.y), float(p.z))
                traj.append((ts, target_pos[0], target_pos[1], target_pos[2]))
                target_hist_ts.append(ts)
                target_hist_pos.append(target_pos)
                if target_start is None:
                    target_start = target_pos
                continue

            if topic.endswith('/search_state') and topic.startswith('/drone'):
                did = parse_drone_id(topic)
                if did is not None:
                    search_state[did] = bool(msg.data)
                continue

            if topic.endswith('/odom') and topic.startswith('/drone') and topic.count('/') == 2:
                did = parse_drone_id(topic)
                if did is None:
                    continue
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                drone_pos[did] = (float(p.x), float(p.y), float(p.z))
                drone_vel[did] = (float(v.x), float(v.y), float(v.z))
                drone_quat[did] = (float(q.x), float(q.y), float(q.z), float(q.w))
                drone_hist[did]['ts'].append(ts)
                drone_hist[did]['pos'].append(drone_pos[did])
                drone_hist[did]['quat'].append(drone_quat[did])

                if target_pos is None or not all(i in drone_pos for i in drone_ids):
                    continue
                if not all(i in drone_quat for i in drone_ids):
                    continue

                positions_now = {i: drone_pos[i] for i in drone_ids}
                quat_now = {i: drone_quat[i] for i in drone_ids}
                dists = []
                for i in range(len(drone_ids)):
                    for j in range(i + 1, len(drone_ids)):
                        a = positions_now[drone_ids[i]]
                        b = positions_now[drone_ids[j]]
                        dists.append(math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2))
                rms = math.sqrt(sum((d - desired_side) ** 2 for d in dists) / len(dists))
                std = float(np.std(dists))
                speed_mean = float(np.mean([
                    math.sqrt(drone_vel[i][0] ** 2 + drone_vel[i][1] ** 2 + drone_vel[i][2] ** 2)
                    for i in drone_ids
                ]))
                score = rms + 0.3 * std + (0.2 if speed_mean < 0.2 else 0.0)
                fallback.append((score, ts, positions_now, target_pos, quat_now))

                if not all(i in search_state for i in drone_ids):
                    continue
                if any(search_state[i] for i in drone_ids):
                    continue
                if target_start is not None:
                    moved = math.sqrt(
                        (target_pos[0] - target_start[0]) ** 2 +
                        (target_pos[1] - target_start[1]) ** 2 +
                        (target_pos[2] - target_start[2]) ** 2
                    )
                    if moved < min_target_move:
                        continue
                if (ts - start_ts) < min_elapsed:
                    continue
                cands.append((score, ts, positions_now, target_pos, quat_now))

    if not traj:
        raise RuntimeError('No target trajectory found in bag')
    if not fallback:
        raise RuntimeError('No tracker odom found in bag')

    best = min(cands, key=lambda x: x[0]) if cands else min(fallback, key=lambda x: x[0])
    best_score, best_ts, best_pos, best_tgt, best_quat = best
    traj_arr = np.asarray(traj, dtype=np.float64)

    # For visualization: use state a few seconds before selected snapshot.
    lookback = max(0.0, float(state_lookback_sec))
    render_ts = max(float(start_ts), float(best_ts) - lookback)
    ts_tgt, tgt_at = sample_history_at_or_before(target_hist_ts, target_hist_pos, render_ts)
    if tgt_at is None:
        ts_tgt = best_ts
        tgt_at = best_tgt

    pos_at = {}
    quat_at = {}
    ts_used = [float(ts_tgt)]
    for did in drone_ids:
        h = drone_hist[did]
        ts_p, p_at = sample_history_at_or_before(h['ts'], h['pos'], render_ts)
        ts_q, q_at = sample_history_at_or_before(h['ts'], h['quat'], render_ts)
        if p_at is None or q_at is None:
            p_at = best_pos[did]
            q_at = best_quat[did]
            ts_p = best_ts
            ts_q = best_ts
        pos_at[did] = p_at
        quat_at[did] = q_at
        ts_used.append(float(ts_p))
        ts_used.append(float(ts_q))
    render_ts = min([float(render_ts)] + ts_used)

    if obstacle_pts is None:
        obstacle_pts = np.empty((0, 3), dtype=np.float32)

    # Optional: crop obstacles around target trajectory area.
    # Default is disabled to preserve full scene structures (wide walls/columns).
    if bool(crop_obstacles) and len(obstacle_pts) > 0:
        x_min, x_max = float(np.min(traj_arr[:, 1])), float(np.max(traj_arr[:, 1]))
        y_min, y_max = float(np.min(traj_arr[:, 2])), float(np.max(traj_arr[:, 2]))
        z_min, z_max = float(np.min(traj_arr[:, 3])), float(np.max(traj_arr[:, 3]))
        margin_xy = float(crop_margin_xy)
        margin_z = float(crop_margin_z)
        m = (
            (obstacle_pts[:, 0] >= x_min - margin_xy) & (obstacle_pts[:, 0] <= x_max + margin_xy) &
            (obstacle_pts[:, 1] >= y_min - margin_xy) & (obstacle_pts[:, 1] <= y_max + margin_xy) &
            (obstacle_pts[:, 2] >= z_min - margin_z) & (obstacle_pts[:, 2] <= z_max + margin_z)
        )
        obs = obstacle_pts[m]
    else:
        obs = obstacle_pts

    if len(obs) > max_obs_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(obs), size=max_obs_points, replace=False)
        obs = obs[idx]

    return {
        'traj_arr': traj_arr,
        'obs': obs,
        'best_score': best_score,
        'best_ts': best_ts,
        'render_ts': render_ts,
        'start_ts': start_ts,
        'best_pos': pos_at,
        'best_tgt': tgt_at,
        'best_quat': quat_at,
        'drone_ids': drone_ids,
    }


def catmull_rom_chain(ctrl_pts, samples_per_seg=28):
    ctrl = np.asarray(ctrl_pts, dtype=np.float64)
    if len(ctrl) < 2:
        return ctrl.copy()

    out = []
    for i in range(len(ctrl) - 1):
        p0 = ctrl[i - 1] if i > 0 else ctrl[i]
        p1 = ctrl[i]
        p2 = ctrl[i + 1]
        p3 = ctrl[i + 2] if (i + 2) < len(ctrl) else ctrl[i + 1]

        ts = np.linspace(0.0, 1.0, max(6, samples_per_seg), endpoint=False)
        for t in ts:
            t2 = t * t
            t3 = t2 * t
            c = 0.5 * (
                (2.0 * p1) +
                (-p0 + p2) * t +
                (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            out.append(c)
    out.append(ctrl[-1])
    return np.asarray(out, dtype=np.float64)


def _smooth_control_points(anchors):
    if len(anchors) < 3:
        return anchors
    sm = anchors.copy()
    for i in range(1, len(anchors) - 1):
        sm[i] = 0.2 * anchors[i - 1] + 0.6 * anchors[i] + 0.2 * anchors[i + 1]
    return sm


def _closest_projection_on_polyline_xy(start_xyz, polyline_xyz):
    s = np.asarray(start_xyz, dtype=np.float64)
    pl = np.asarray(polyline_xyz, dtype=np.float64)
    if len(pl) < 2:
        return s.copy(), 0

    best_dist = float('inf')
    best_proj = pl[0].copy()
    best_seg = 0
    p = s[:2]
    for i in range(len(pl) - 1):
        a = pl[i]
        b = pl[i + 1]
        v = b[:2] - a[:2]
        vv = float(np.dot(v, v))
        if vv < 1e-10:
            t = 0.0
        else:
            t = float(np.dot(p - a[:2], v) / vv)
            t = max(0.0, min(1.0, t))
        proj_xy = a[:2] + t * v
        d = float(np.linalg.norm(p - proj_xy))
        if d < best_dist:
            best_dist = d
            best_seg = i
            z = (1.0 - t) * a[2] + t * b[2]
            best_proj = np.array([proj_xy[0], proj_xy[1], z], dtype=np.float64)
    return best_proj, best_seg


def build_smooth_trend_curve_from_waypoints(best_tgt, waypoint_xy, waypoint_z=None, ctrl_count=10):
    start = np.asarray(best_tgt, dtype=np.float64)
    z = float(start[2] if waypoint_z is None else waypoint_z)
    if not waypoint_xy:
        return np.empty((0, 3), dtype=np.float64)

    wp = np.asarray([[float(x), float(y), z] for (x, y) in waypoint_xy], dtype=np.float64)
    # User requirement: from current target position, connect all waypoints in script order.
    seq = wp

    # Keep strict waypoint alignment:
    # curve must pass through current target point and every waypoint in order.
    anchors = [start]
    for p in seq:
        if np.linalg.norm(p - anchors[-1]) > 1e-6:
            anchors.append(p)
    if len(anchors) < 2:
        anchors.append(start + np.array([1.2, 0.0, 0.0], dtype=np.float64))

    anchors = np.asarray(anchors, dtype=np.float64)
    anchors[0] = start
    curve = catmull_rom_chain(anchors, samples_per_seg=34)
    if len(curve) > 0:
        curve[0] = start
    return curve


def build_smooth_trend_curve_from_traj(traj_arr, best_ts, best_tgt, ctrl_count=7):
    start = np.asarray(best_tgt, dtype=np.float64)
    future = traj_arr[traj_arr[:, 0] >= best_ts, 1:4]
    if len(future) == 0:
        near_idx = int(np.argmin(np.abs(traj_arr[:, 0] - best_ts)))
        i0 = max(0, min(len(traj_arr) - 2, near_idx))
        future = traj_arr[i0:i0 + 2, 1:4]

    anchors = [start]
    for p in future:
        p = np.asarray(p, dtype=np.float64)
        if np.linalg.norm(p - anchors[-1]) > 0.08:
            anchors.append(p)

    if len(anchors) < 2:
        anchors.append(start + np.array([1.2, 0.0, 0.0], dtype=np.float64))

    anchors = np.asarray(anchors, dtype=np.float64)
    if len(anchors) > ctrl_count:
        idx = np.linspace(0, len(anchors) - 1, num=ctrl_count, dtype=int)
        anchors = anchors[np.unique(idx)]
    anchors = _smooth_control_points(anchors)
    anchors[0] = start
    curve = catmull_rom_chain(anchors, samples_per_seg=30)
    if len(curve) > 0:
        curve[0] = start
    return curve


def resample_polyline(pts, n_points):
    arr = np.asarray(pts, dtype=np.float64)
    if len(arr) < 2 or n_points <= 2:
        return arr.copy()

    seg = np.linalg.norm(np.diff(arr, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(s[-1])
    if total < 1e-8:
        return np.repeat(arr[:1], n_points, axis=0)

    q = np.linspace(0.0, total, n_points)
    out = np.zeros((n_points, 3), dtype=np.float64)
    for k in range(3):
        out[:, k] = np.interp(q, s, arr[:, k])
    return out


def sample_subcurve(pts, s, s0, s1, n_points):
    q = np.linspace(float(s0), float(s1), int(max(6, n_points)))
    out = np.zeros((len(q), 3), dtype=np.float64)
    for k in range(3):
        out[:, k] = np.interp(q, s, pts[:, k])
    return out


def draw_smooth_trend_arrow(ax, curve, segment_count=28, zorder_base=50):
    if len(curve) < 2:
        return 0

    pts = resample_polyline(curve, max(10, segment_count + 2))
    diff = np.diff(pts, axis=0)
    seg_len = np.linalg.norm(diff, axis=1)
    total_len = float(np.sum(seg_len))
    if total_len < 1e-5:
        return 0

    tang = np.zeros_like(pts)
    tang[0] = pts[1] - pts[0]
    tang[-1] = pts[-1] - pts[-2]
    if len(pts) > 2:
        tang[1:-1] = pts[2:] - pts[:-2]
    tnorm = np.linalg.norm(tang, axis=1, keepdims=True)
    tnorm[tnorm < 1e-8] = 1.0
    tang = tang / tnorm

    normals = np.zeros_like(pts)
    prev = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    for i in range(len(pts)):
        t = tang[i]
        n = np.array([-t[1], t[0], 0.0], dtype=np.float64)
        nn = np.linalg.norm(n)
        if nn < 1e-7:
            n = prev
        else:
            n /= nn
            prev = n
        normals[i] = n

    base_w = float(np.clip(total_len * 0.035, 0.18, 0.42))
    neck_w = base_w * 0.56
    widths = np.linspace(base_w, neck_w, len(pts))

    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    head_len = float(np.clip(total_len * 0.16, 0.8, 2.2))
    head_start_s = max(0.0, total_len - head_len)
    body_end = int(np.searchsorted(cum, head_start_s))
    body_end = max(2, min(len(pts) - 2, body_end))

    body_segments = max(1, body_end - 1)
    gray_vals = np.linspace(0.72, 0.46, body_segments)
    for i in range(body_segments):
        i0 = i
        i1 = i + 1
        l0 = pts[i0] + normals[i0] * (0.5 * widths[i0])
        r0 = pts[i0] - normals[i0] * (0.5 * widths[i0])
        l1 = pts[i1] + normals[i1] * (0.5 * widths[i1])
        r1 = pts[i1] - normals[i1] * (0.5 * widths[i1])
        g = float(gray_vals[i])
        quad = Poly3DCollection(
            [[l0, r0, r1, l1]],
            facecolors=[(g, g, g, 0.60)],
            edgecolors='none',
            zorder=float(zorder_base + 1),
        )
        if hasattr(quad, 'set_sort_zpos'):
            quad.set_sort_zpos(1e6)
        ax.add_collection3d(quad)

    tip = pts[-1]
    base = pts[body_end]
    head_n = normals[body_end]
    head_half_w = max(0.5 * neck_w, 0.13)
    bl = base + head_n * head_half_w
    br = base - head_n * head_half_w
    c_tip = (0.40, 0.40, 0.40, 1.0)
    head = Poly3DCollection(
        [[bl, br, tip]],
        facecolors=[(c_tip[0], c_tip[1], c_tip[2], 0.74)],
        edgecolors='none',
        zorder=float(zorder_base + 2),
    )
    if hasattr(head, 'set_sort_zpos'):
        head.set_sort_zpos(1e6)
    ax.add_collection3d(head)

    # Slight centerline highlight for smoother visual continuity.
    ax.plot(
        pts[:, 0], pts[:, 1], pts[:, 2],
        color=(0.58, 0.58, 0.58, 0.35),
        linewidth=1.1,
        zorder=float(zorder_base + 3),
    )

    return 1


def draw_smooth_trend_arrows(ax, curve, arrow_count=5, segment_count=16, zorder_base=50):
    if len(curve) < 2:
        return 0
    pts = resample_polyline(curve, max(120, arrow_count * 50))
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(s[-1])
    if total < 1e-5:
        return 0

    n = int(max(2, arrow_count))
    margin = min(0.55, 0.08 * total)
    usable = max(0.8, total - 2.0 * margin)
    span = usable / n
    drawn = 0
    for i in range(n):
        s0 = margin + i * span + 0.04 * span
        s1 = margin + (i + 1) * span - 0.08 * span
        if (s1 - s0) <= 0.22:
            continue
        sub = sample_subcurve(pts, s, s0, s1, n_points=max(24, segment_count * 2))
        drawn += draw_smooth_trend_arrow(
            ax, sub, segment_count=segment_count, zorder_base=zorder_base
        )
    return drawn


def project_points_to_figure(fig, ax, pts3):
    arr = np.asarray(pts3, dtype=np.float64)
    if len(arr) == 0:
        return np.empty((0, 2), dtype=np.float64)
    x2, y2, _ = proj3d.proj_transform(arr[:, 0], arr[:, 1], arr[:, 2], ax.get_proj())
    disp = ax.transData.transform(np.column_stack((x2, y2)))
    fig_xy = fig.transFigure.inverted().transform(disp)
    return np.asarray(fig_xy, dtype=np.float64)


def draw_trend_overlay_on_top(fig, ax, curve, arrow_count=5):
    """
    在 figure 2D 层叠加绘制目标趋势箭头，保证视觉上始终在最上层（不受3D遮挡）。
    """
    if len(curve) < 2:
        return 0

    pts = resample_polyline(curve, max(120, int(max(2, arrow_count)) * 50))
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(s[-1])
    if total < 1e-6:
        return 0

    n = int(max(2, arrow_count))
    margin = min(0.55, 0.08 * total)
    usable = max(0.8, total - 2.0 * margin)
    span = usable / n

    drawn = 0
    for i in range(n):
        s0 = margin + i * span + 0.04 * span
        s1 = margin + (i + 1) * span - 0.08 * span
        if (s1 - s0) <= 0.22:
            continue
        sub3 = sample_subcurve(pts, s, s0, s1, n_points=36)
        sub2 = project_points_to_figure(fig, ax, sub3)
        if len(sub2) < 3:
            continue
        if not np.all(np.isfinite(sub2)):
            continue

        # Build arrow head from the end tangent to ensure exact connection at curve end.
        tip = np.asarray(sub2[-1], dtype=np.float64)
        prev = np.asarray(sub2[max(0, len(sub2) - 4)], dtype=np.float64)
        tvec = tip - prev
        tnorm = float(np.linalg.norm(tvec))
        if tnorm < 1e-10:
            continue
        tdir = tvec / tnorm
        nrm = np.array([-tdir[1], tdir[0]], dtype=np.float64)

        # Length/width in figure-normalized coordinates
        head_len = float(np.clip(0.11 * tnorm, 0.010, 0.030))
        head_w = float(np.clip(0.65 * head_len, 0.006, 0.020))
        base = tip - tdir * head_len
        left = base + nrm * head_w
        right = base - nrm * head_w

        # 2D overlay centerline (end at head base, not passing tip)
        body = sub2.copy()
        body[-1] = base
        line = Line2D(
            body[:, 0], body[:, 1],
            transform=fig.transFigure,
            color=(0.50, 0.50, 0.50, 0.92),
            linewidth=2.0,
            solid_capstyle='round',
            zorder=10000,
        )
        fig.add_artist(line)

        # 2D overlay triangle head with exact tip at curve end.
        head = MplPolygon(
            np.vstack([left, tip, right]),
            closed=True,
            transform=fig.transFigure,
            facecolor=(0.36, 0.36, 0.36, 0.96),
            edgecolor='none',
            zorder=10001,
        )
        fig.add_artist(head)
        drawn += 1
    return drawn


def draw_scene(
    data,
    out_path,
    trend_arrows=28,
    evasion_waypoints=None,
    path_arrows=5,
    camera_model=None,
    show_fov=True,
    fov_range_scale=1.30,
    fov_size_scale=0.68,
    fov_forward_offset=0.85,
    obstacle_densify_factor=2.5,
    obstacle_jitter_xy=0.06,
    obstacle_jitter_z=0.03,
    obstacle_point_size=1.8,
):
    traj_arr = data['traj_arr']
    obs = data['obs']
    best_score = data['best_score']
    best_ts = data['best_ts']
    render_ts = data.get('render_ts', best_ts)
    start_ts = data['start_ts']
    best_pos = data['best_pos']
    best_tgt = data['best_tgt']
    best_quat = data['best_quat']
    drone_ids = data['drone_ids']

    fig = plt.figure(figsize=(10, 8), dpi=220)
    ax = fig.add_subplot(111, projection='3d')

    # Obstacle point cloud style aligned with formation_shots.py
    obs_draw = densify_obstacle_points_np(
        obs,
        max_base_points=max(1000, int(len(obs)) if len(obs) < 120000 else 120000),
        densify_factor=float(obstacle_densify_factor),
        jitter_xy=float(obstacle_jitter_xy),
        jitter_z=float(obstacle_jitter_z),
        seed=7,
    )
    if len(obs_draw) > 0:
        obs_vmin = float(obs_draw[:, 2].min())
        obs_vmax = float(obs_draw[:, 2].max())
        obs_sc = ax.scatter(
            obs_draw[:, 0], obs_draw[:, 1], obs_draw[:, 2],
            c=obs_draw[:, 2],
            cmap='plasma',
            vmin=obs_vmin, vmax=obs_vmax,
            s=float(obstacle_point_size),
            alpha=0.70,
            linewidths=0.0,
            depthshade=False,
            zorder=1,
            rasterized=True,
        )
        cbar = plt.colorbar(obs_sc, ax=ax, fraction=0.022, pad=0.01)
        cbar.set_label('高度 z (m)', fontsize=10)
    else:
        obs_vmin, obs_vmax = 0.0, 5.0

    # 轨迹：waypoints Catmull-Rom 平滑，起点加幽灵点保证切线自然
    trend_curve = np.empty((0, 3), dtype=np.float64)
    if evasion_waypoints:
        z_tgt = float(best_tgt[2])
        wp = np.array([[float(x), float(y), z_tgt] for x, y in evasion_waypoints],
                      dtype=np.float64)
        start = np.asarray(best_tgt, dtype=np.float64)
        anchors = [start]
        for p in wp:
            if np.linalg.norm(p - anchors[-1]) > 1e-6:
                anchors.append(p)
        if len(anchors) >= 2:
            # 幽灵点：起点沿反方向延伸，让曲线在起点处切线平滑
            ghost = anchors[0] - (anchors[1] - anchors[0]) * 0.4
            anchors_ext = np.array([ghost] + anchors, dtype=np.float64)
            trend_curve = catmull_rom_chain(anchors_ext, samples_per_seg=40)
            if len(trend_curve) > 0:
                trend_curve[0] = start  # 强制起点精确

    if len(trend_curve) < 2:
        # fallback：真实轨迹降采样平滑
        ctrl_idx = np.linspace(0, len(traj_arr) - 1, min(14, len(traj_arr)), dtype=int)
        trend_curve = catmull_rom_chain(traj_arr[ctrl_idx, 1:4], samples_per_seg=40)

    traj_xyz = trend_curve
    if len(traj_xyz) >= 2:
        nc = len(traj_xyz)
        cmap_traj = plt.get_cmap('cool')
        for i in range(nc - 1):
            t_frac = i / max(1, nc - 2)
            ax.plot(traj_xyz[i:i+2, 0], traj_xyz[i:i+2, 1], traj_xyz[i:i+2, 2],
                    color=cmap_traj(t_frac), lw=2.0, alpha=0.90, zorder=4,
                    solid_capstyle='round')

        n_arrows = 7
        arr_idx = np.linspace(int(nc * 0.05), int(nc * 0.90), n_arrows, dtype=int)
        step = max(1, nc // (n_arrows * 3))
        for i in arr_idx:
            j = min(i + step, nc - 1)
            p0, p1 = traj_xyz[i], traj_xyz[j]
            d = p1 - p0
            norm = np.linalg.norm(d)
            if norm < 0.02:
                continue
            d_unit = d / norm
            t_frac = i / max(1, nc - 2)
            col = cmap_traj(t_frac)
            tip = p0 + d_unit * 0.50
            ax.plot([p0[0], tip[0]], [p0[1], tip[1]], [p0[2], tip[2]],
                    color=col, lw=2.2, alpha=0.95, zorder=5, solid_capstyle='round')
            ax.quiver(tip[0], tip[1], tip[2],
                      d_unit[0], d_unit[1], d_unit[2],
                      length=0.28, color=col,
                      arrow_length_ratio=1.0, linewidth=0,
                      alpha=0.95, zorder=5)
    trend_arrow_count = n_arrows if len(traj_xyz) >= 2 else 0

    # Target marker at snapshot
    draw_quad_marker(
        ax,
        best_tgt[0], best_tgt[1], best_tgt[2],
        arm=0.62, rotor_r=0.13, color='#7B3F00', lw=1.6, alpha=1.0
    )

    # Trackers as black quad markers
    for did in drone_ids:
        p = best_pos[did]
        draw_quad_marker(ax, p[0], p[1], p[2], arm=0.52, rotor_r=0.10, color='#111111', lw=1.35, alpha=1.0)

    # FOV from bag snapshot pose/orientation (light blue, near transparent)
    if show_fov and (camera_model is not None):
        for did in drone_ids:
            draw_drone_fov(
                ax,
                drone_pos=best_pos[did],
                drone_quat_xyzw=best_quat[did],
                camera_model=camera_model,
                color=(0.56, 0.82, 0.99, 0.10),                range_scale=float(fov_range_scale),
                size_scale=float(fov_size_scale),
                forward_offset=float(fov_forward_offset),
            )

    # Formation polygon
    pts = np.array([best_pos[i] for i in drone_ids], dtype=float)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    order = np.argsort(angles)
    poly = pts[order]
    poly = np.vstack([poly, poly[0]])
    ax.plot(poly[:, 0], poly[:, 1], poly[:, 2], color='#1f77b4', linestyle='--', linewidth=1.25, alpha=0.95)

    # Draw trend arrows last and with high z-order (visual top layer).
    trend_arrow_count = 0

    # Limits with moderate z scale
    all_x = [float(np.min(traj_arr[:, 1])), float(np.max(traj_arr[:, 1])), best_tgt[0]] + [best_pos[i][0] for i in drone_ids]
    all_y = [float(np.min(traj_arr[:, 2])), float(np.max(traj_arr[:, 2])), best_tgt[1]] + [best_pos[i][1] for i in drone_ids]
    all_z = [float(np.min(traj_arr[:, 3])), float(np.max(traj_arr[:, 3])), best_tgt[2]] + [best_pos[i][2] for i in drone_ids]
    if len(obs_draw) > 0:
        all_x += [float(np.min(obs_draw[:, 0])), float(np.max(obs_draw[:, 0]))]
        all_y += [float(np.min(obs_draw[:, 1])), float(np.max(obs_draw[:, 1]))]
        all_z += [float(np.min(obs_draw[:, 2])), float(np.max(obs_draw[:, 2]))]

    x_mid = 0.5 * (min(all_x) + max(all_x))
    y_mid = 0.5 * (min(all_y) + max(all_y))
    z_mid = 0.5 * (min(all_z) + max(all_z))
    span_xy = max(max(all_x) - min(all_x), max(all_y) - min(all_y)) * 0.55
    if span_xy < 1.0:
        span_xy = 1.0
    z_span = span_xy * 0.5
    z0 = max(0.0, z_mid - z_span)
    ax.set_xlim(x_mid - span_xy, x_mid + span_xy)
    ax.set_ylim(y_mid - span_xy, y_mid + span_xy)
    ax.set_zlim(z0, z_mid + z_span)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect((1.0, 1.0, 0.52))

    # Clean axes: keep only ground frame
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_zlabel('')
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_visible(False)
        axis.line.set_color((1, 1, 1, 0))
    ax.grid(False)

    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()

    # 地板点云：与障碍物完全相同的点大小、密度、colormap
    floor_res = 260
    gx, gy = np.meshgrid(np.linspace(xmin, xmax, floor_res),
                         np.linspace(ymin, ymax, floor_res))
    floor_base = np.column_stack([gx.ravel(), gy.ravel(),
                                  np.full(floor_res * floor_res, z0)])
    floor_pts = densify_obstacle_points_np(
        floor_base,
        max_base_points=len(floor_base),
        densify_factor=float(obstacle_densify_factor),
        jitter_xy=float((xmax - xmin) / floor_res) * 0.5,
        jitter_z=0.0,
        seed=11,
    )
    ax.scatter(floor_pts[:, 0], floor_pts[:, 1], floor_pts[:, 2],
               c=floor_pts[:, 2], cmap='plasma',
               vmin=obs_vmin, vmax=obs_vmax,
               s=float(obstacle_point_size),
               alpha=0.70, linewidths=0,
               depthshade=False, zorder=0, rasterized=True)

    # 地板边框
    ax.plot([xmin, xmax, xmax, xmin, xmin],
            [ymin, ymin, ymax, ymax, ymin],
            [z0] * 5, color=(0.65, 0.65, 0.65, 0.8), linewidth=0.9)

    legend_handles = [
        DroneLegendHandle(color='#111111', count=3),
        DroneLegendHandle(color='#7B3F00', count=1),
        Line2D([0], [0], color='#00ccff', lw=1.8, label='目标运动路线'),
        Line2D([0], [0], marker='o', markersize=6.6, markerfacecolor='#42a5f5',
               markeredgecolor='none', linestyle='none', alpha=0.70, label='Obstacles'),
    ]
    legend_font = get_chinese_font()
    ax.legend(
        handles=legend_handles,
        labels=['追踪者', '目标', '目标运动路线', '障碍物点云'],
        loc='upper right',
        bbox_to_anchor=(0.98, 0.98),
        frameon=True,
        framealpha=0.92,
        edgecolor='#cfcfcf',
        facecolor='white',
        fontsize=9.0,
        handlelength=2.8,
        handleheight=1.0,
        handletextpad=0.6,
        labelspacing=0.5,
        prop=legend_font,
        handler_map={
            DroneLegendHandle: HandlerDroneLegend(),
        },
    )

    # More top-down, still biased toward the bisector direction in xy-plane.
    ax.view_init(elev=60, azim=236)

    fig.tight_layout(pad=0.05)
    fig.savefig(out_path, dpi=380)
    return {
        'best_snapshot_time_sec': render_ts - start_ts,
        'best_score': best_score,
        'obstacle_points_drawn': int(len(obs_draw)),
        'trend_arrows': int(trend_arrow_count),
    }


def main():
    parser = argparse.ArgumentParser(description='Render raw 3D scenario snapshot from rosbag.')
    parser.add_argument('bag', help='Input rosbag path')
    parser.add_argument('--out', default='', help='Output PNG path')
    parser.add_argument('--drone-ids', default='0,1,2', help='Comma-separated tracker drone ids')
    parser.add_argument('--desired-side', type=float, default=2.0)
    parser.add_argument('--min-target-move', type=float, default=5.0)
    parser.add_argument('--min-elapsed', type=float, default=10.0)
    parser.add_argument(
        '--state-lookback-sec',
        type=float,
        default=3.0,
        help='Use target/tracker/FOV states this many seconds before selected snapshot',
    )
    parser.add_argument('--max-obs-points', type=int, default=180000)
    parser.add_argument('--crop-obstacles', action='store_true', help='Crop obstacle cloud around target trajectory')
    parser.add_argument('--crop-margin-xy', type=float, default=10.0, help='XY crop margin when --crop-obstacles is enabled')
    parser.add_argument('--crop-margin-z', type=float, default=4.0, help='Z crop margin when --crop-obstacles is enabled')
    parser.add_argument('--obstacle-densify-factor', type=float, default=2.5, help='Obstacle densification factor')
    parser.add_argument('--obstacle-jitter-xy', type=float, default=0.06, help='Obstacle XY jitter for densification')
    parser.add_argument('--obstacle-jitter-z', type=float, default=0.03, help='Obstacle Z jitter for densification')
    parser.add_argument('--obstacle-point-size', type=float, default=1.8, help='Obstacle point size')
    parser.add_argument('--trend-arrows', type=int, default=18, help='Per-arrow smoothness segments')
    parser.add_argument('--path-arrows', type=int, default=5, help='How many smooth arrows along trend path')
    parser.add_argument(
        '--camera-yaml',
        default='src/mapping/config/camera.yaml',
        help='Camera model yaml for FOV (range/intrinsics/cam2body)',
    )
    parser.add_argument('--fov-range-scale', type=float, default=1.30, help='FOV range multiplier')
    parser.add_argument('--fov-size-scale', type=float, default=0.68, help='FOV width/height multiplier')
    parser.add_argument('--fov-forward-offset', type=float, default=0.85, help='Move FOV apex forward (m)')
    parser.add_argument('--no-fov', action='store_true', help='Disable drawing drone FOV frustums')
    parser.add_argument(
        '--evasion-script',
        default='src/planning/planning/scripts/full_evasion.py',
        help='Python script path that defines WAYPOINTS',
    )
    args = parser.parse_args()

    bag_path = Path(args.bag).resolve()
    if not bag_path.exists():
        raise FileNotFoundError(str(bag_path))
    if bag_path.stat().st_size == 0:
        raise RuntimeError(
            'Input bag is empty (0 bytes): {}. '
            'If you used multiline shell command, do not type the prompt symbol ">".'.format(str(bag_path))
        )

    if args.out:
        out_path = Path(args.out).resolve()
    else:
        out_path = bag_path.parent / 'scene_3d_top_quad.png'

    drone_ids = [int(x.strip()) for x in args.drone_ids.split(',') if x.strip()]
    if len(drone_ids) < 2:
        raise ValueError('Need at least 2 drone ids')

    data = pick_snapshot_and_data(
        bag_path=bag_path,
        drone_ids=drone_ids,
        desired_side=float(args.desired_side),
        min_target_move=float(args.min_target_move),
        min_elapsed=float(args.min_elapsed),
        max_obs_points=int(args.max_obs_points),
        crop_obstacles=bool(args.crop_obstacles),
        crop_margin_xy=float(args.crop_margin_xy),
        crop_margin_z=float(args.crop_margin_z),
        state_lookback_sec=float(args.state_lookback_sec),
    )
    evasion_waypoints = load_waypoints_from_script(args.evasion_script)
    camera_model = load_camera_model(args.camera_yaml)
    info = draw_scene(
        data,
        out_path=out_path,
        trend_arrows=int(args.trend_arrows),
        evasion_waypoints=evasion_waypoints,
        path_arrows=int(args.path_arrows),
        camera_model=camera_model,
        show_fov=(not args.no_fov),
        fov_range_scale=float(args.fov_range_scale),
        fov_size_scale=float(args.fov_size_scale),
        fov_forward_offset=float(args.fov_forward_offset),
        obstacle_densify_factor=max(1.0, float(args.obstacle_densify_factor)),
        obstacle_jitter_xy=max(0.0, float(args.obstacle_jitter_xy)),
        obstacle_jitter_z=max(0.0, float(args.obstacle_jitter_z)),
        obstacle_point_size=max(0.2, float(args.obstacle_point_size)),
    )

    print(str(out_path))
    print('best_snapshot_time_sec={:.3f}'.format(info['best_snapshot_time_sec']))
    print('best_score={:.6f}'.format(info['best_score']))
    print('obstacle_points_drawn={}'.format(info['obstacle_points_drawn']))
    print('trend_arrows={}'.format(info['trend_arrows']))


if __name__ == '__main__':
    main()
