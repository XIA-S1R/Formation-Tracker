#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyVista 高质量 3D 场景渲染：障碍物点云 + 地板点云 + 目标轨迹管道 + 无人机图标
用法:
  python3 render_scene_pyvista.py <bag> [--out scene.png] [--camera-yaml ...]
"""
import argparse
import bisect
import math
import sys
from pathlib import Path

import numpy as np
import pyvista as pv

try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
    import yaml
except ImportError:
    print("需要 rosbag: source devel/setup.bash")
    sys.exit(1)


# ── 相机/bag 工具 ─────────────────────────────────────────────────────────────

def load_camera_model(path):
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    w  = float(cfg.get('cam_width',  640))
    h  = float(cfg.get('cam_height', 480))
    fx = float(cfg.get('cam_fx', 387.229248046875))
    fy = float(cfg.get('cam_fy', 387.229248046875))
    rng = float(cfg.get('camera_range', 7.0))
    R_bc = np.array(cfg.get('cam2body_R', [0,0,1,-1,0,0,0,-1,0]),
                    dtype=np.float64).reshape(3, 3)
    return dict(range=rng, h_fov=2*math.atan(w/(2*fx)),
                v_fov=2*math.atan(h/(2*fy)), R_bc=R_bc)


def quat_to_rotmat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    qx,qy,qz,qw = qx/n,qy/n,qz/n,qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def _bisect_at(ts, vals, t):
    if not ts: return None
    i = min(bisect.bisect_right(ts, t), len(ts)-1)
    return vals[max(0, i-1)]


def load_bag(bag_path, drone_ids, max_obs=180000):
    topics = ['/global_map', '/target/odom']
    for d in drone_ids:
        topics += [f'/drone{d}/odom',
                   f'/drone{d}/drone{d}_target_dpf/search_state']

    obs_pts = None
    traj = []
    drone_hist = {d: {'ts':[], 'pos':[], 'quat':[]} for d in drone_ids}
    search_state = {d: False for d in drone_ids}
    start_ts = None

    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if start_ts is None: start_ts = ts

            if topic == '/global_map' and obs_pts is None:
                pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
                if pts:
                    arr = np.array(pts, dtype=np.float32)
                    if len(arr) > max_obs:
                        idx = np.random.default_rng(42).choice(len(arr), max_obs, replace=False)
                        arr = arr[idx]
                    obs_pts = arr

            elif topic == '/target/odom':
                p = msg.pose.pose.position
                traj.append((ts, p.x, p.y, p.z))

            elif '/search_state' in topic:
                for d in drone_ids:
                    if f'drone{d}' in topic:
                        search_state[d] = bool(msg.data)

            elif topic.endswith('/odom') and '/drone' in topic:
                for d in drone_ids:
                    if f'/drone{d}/odom' == topic:
                        p = msg.pose.pose.position
                        q = msg.pose.pose.orientation
                        drone_hist[d]['ts'].append(ts)
                        drone_hist[d]['pos'].append((p.x, p.y, p.z))
                        drone_hist[d]['quat'].append((q.x, q.y, q.z, q.w))

    if obs_pts is None: obs_pts = np.zeros((0,3), dtype=np.float32)
    traj_arr = np.array(traj, dtype=np.float64) if traj else np.zeros((0,4))
    return obs_pts, traj_arr, drone_hist, start_ts


def pick_snapshot(traj_arr, drone_hist, drone_ids, min_move=5.0, min_elapsed=10.0, start_ts=0.0):
    """选取编队追踪时刻的快照（所有无人机都有数据后）"""
    best_ts = None
    for d in drone_ids:
        h = drone_hist[d]
        if not h['ts']: continue
        for i, ts in enumerate(h['ts']):
            if ts - start_ts < min_elapsed: continue
            if traj_arr is None or len(traj_arr) == 0: continue
            # 目标移动距离
            tgt = _bisect_at(list(traj_arr[:,0]), traj_arr[:,1:4].tolist(), ts)
            if tgt is None: continue
            tgt0 = traj_arr[0, 1:4]
            if np.linalg.norm(np.array(tgt) - tgt0) < min_move: continue
            best_ts = ts
            break
        if best_ts: break

    if best_ts is None:
        # fallback: 取中间时刻
        all_ts = drone_hist[drone_ids[0]]['ts']
        best_ts = all_ts[len(all_ts)//2] if all_ts else start_ts + min_elapsed

    pos_at, quat_at = {}, {}
    for d in drone_ids:
        h = drone_hist[d]
        p = _bisect_at(h['ts'], h['pos'], best_ts)
        q = _bisect_at(h['ts'], h['quat'], best_ts)
        pos_at[d]  = np.array(p if p else (0,0,2), dtype=np.float64)
        quat_at[d] = np.array(q if q else (0,0,0,1), dtype=np.float64)

    tgt_at = _bisect_at(list(traj_arr[:,0]), traj_arr[:,1:4].tolist(), best_ts)
    tgt_at = np.array(tgt_at if tgt_at else (0,0,0), dtype=np.float64)
    return best_ts, pos_at, quat_at, tgt_at


# ── 几何构建 ──────────────────────────────────────────────────────────────────

def make_quad_mesh(cx, cy, cz, yaw=0.0, arm=0.55, rotor_r=0.11,
                   tube_r=0.025, color=(0.08, 0.08, 0.08)):
    """四旋翼：两根管道臂 + 4个旋翼圆环"""
    meshes = []
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    def rot(dx, dy):
        return cx + cos_y*dx - sin_y*dy, cy + sin_y*dx + cos_y*dy

    # 两根臂
    for (dx0, dy0), (dx1, dy1) in [
        ((-arm, 0), (arm, 0)),
        ((0, -arm), (0, arm)),
    ]:
        x0, y0 = rot(dx0, dy0)
        x1, y1 = rot(dx1, dy1)
        line = pv.Line((x0, y0, cz), (x1, y1, cz), resolution=2)
        tube = line.tube(radius=tube_r, n_sides=10)
        tube['color_val'] = np.full(tube.n_points, 0.5)
        meshes.append(tube)

    # 4个旋翼圆环
    th = np.linspace(0, 2*math.pi, 48, endpoint=False)
    for adx, ady in [(-arm,0),(arm,0),(0,-arm),(0,arm)]:
        rcx, rcy = rot(adx, ady)
        pts = np.column_stack([
            rcx + rotor_r*np.cos(th),
            rcy + rotor_r*np.sin(th),
            np.full(len(th), cz),
        ])
        pts_closed = np.vstack([pts, pts[:1]])
        line = pv.Spline(pts_closed, n_points=len(pts_closed))
        ring = line.tube(radius=tube_r*0.75, n_sides=8)
        ring['color_val'] = np.full(ring.n_points, 0.5)
        meshes.append(ring)

    combined = meshes[0].merge(meshes[1:])
    combined['color_val'] = np.full(combined.n_points, 0.5)
    return combined, color


def make_fov_mesh(pos, quat, cam):
    """FOV 锥体（半透明面）"""
    R_wb = quat_to_rotmat(*quat)
    R_wc = R_wb @ cam['R_bc']
    rng = cam['range']
    hx = rng * math.tan(0.5 * cam['h_fov'])
    hy = rng * math.tan(0.5 * cam['v_fov'])

    far_c = np.array([[ hx, hy, rng],[-hx, hy, rng],
                      [-hx,-hy, rng],[ hx,-hy, rng]], dtype=np.float64)
    apex = np.array(pos, dtype=np.float64)
    far_w = [apex + R_wc @ p for p in far_c]

    faces = []
    for i in range(4):
        j = (i+1) % 4
        # 侧面三角形
        faces += [3, 0, i+1, j+1]
    # 远端面（quad → 2 triangles）
    faces += [3, 1, 2, 3, 3, 1, 3, 4]

    verts = np.vstack([apex, far_w])
    mesh = pv.PolyData(verts, np.array(faces))
    return mesh


def catmull_rom(ctrl, n_per_seg=40):
    ctrl = np.asarray(ctrl, dtype=np.float64)
    if len(ctrl) < 2: return ctrl
    out = []
    for i in range(len(ctrl)-1):
        p0 = ctrl[i-1] if i > 0 else ctrl[i]
        p1 = ctrl[i]
        p2 = ctrl[i+1]
        p3 = ctrl[i+2] if i+2 < len(ctrl) else ctrl[i+1]
        for t in np.linspace(0, 1, n_per_seg, endpoint=False):
            t2,t3 = t*t, t*t*t
            out.append(0.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3))
    out.append(ctrl[-1])
    return np.array(out)


def make_traj_mesh(waypoints_xy, start_xyz, z=None, tube_radius=0.06):
    """轨迹管道 + 沿线箭头"""
    if not waypoints_xy:
        return None, []
    z = z if z is not None else start_xyz[2]
    wp = np.array([[x, y, z] for x,y in waypoints_xy], dtype=np.float64)
    start = np.array(start_xyz, dtype=np.float64)

    anchors = [start]
    for p in wp:
        if np.linalg.norm(p - anchors[-1]) > 0.05:
            anchors.append(p)
    if len(anchors) < 2:
        return None, []

    # 幽灵点让起点切线自然
    ghost = anchors[0] - (anchors[1] - anchors[0]) * 0.5
    curve = catmull_rom([ghost] + anchors, n_per_seg=50)
    curve[0] = start

    spline = pv.Spline(curve, n_points=len(curve))

    # 沿弧长参数化颜色（0→1）
    seg = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    arc = np.concatenate([[0], np.cumsum(seg)])
    arc_norm = arc / max(arc[-1], 1e-6)

    tube = spline.tube(radius=tube_radius, n_sides=16)
    # 给 tube 插值弧长颜色
    tube_pts = tube.points
    # 最近邻映射到 curve
    from scipy.spatial import cKDTree
    tree = cKDTree(curve)
    _, idx = tree.query(tube_pts)
    tube['arc'] = arc_norm[np.clip(idx, 0, len(arc_norm)-1)]

    # 箭头：沿曲线均匀放置
    n_arrows = 8
    arrow_meshes = []
    arrow_arc_pos = np.linspace(0.05, 0.92, n_arrows)
    for frac in arrow_arc_pos:
        target_s = frac * arc[-1]
        i = int(np.searchsorted(arc, target_s))
        i = min(i, len(curve)-2)
        d = curve[i+1] - curve[i]
        norm = np.linalg.norm(d)
        if norm < 0.01: continue
        direction = d / norm
        arrow = pv.Arrow(start=curve[i], direction=direction,
                         tip_length=0.35, tip_radius=0.14,
                         shaft_radius=0.0,  # 只要锥头
                         scale=0.55)
        arrow['arc'] = np.full(arrow.n_points, frac)
        arrow_meshes.append(arrow)

    return tube, arrow_meshes


# ── 主渲染函数 ────────────────────────────────────────────────────────────────

def render_scene(obs_pts, traj_arr, drone_hist, drone_ids,
                 waypoints_xy, cam_model, out_path,
                 snapshot_ts=None, pos_at=None, quat_at=None, tgt_at=None,
                 window_size=(1920, 1080)):
    """
    PyVista 离屏渲染：障碍物点云 + 地板平面 + 规划路径管道 + 无人机图标 + FOV棱线
    """
    TRACKER_COLOR = '#0055ff'   # 深蓝
    TARGET_COLOR  = '#ee0000'   # 深红
    PLAN_COLOR    = '#ffe033'
    FORM_COLOR    = '#66bbff'

    pl = pv.Plotter(off_screen=True, window_size=list(window_size))
    pl.set_background('white')

    # ── 场景范围 ──────────────────────────────────────────────────────────────
    all_xy = []
    if obs_pts is not None and len(obs_pts) > 0:
        all_xy.append(obs_pts[:, :2])
    if traj_arr is not None and len(traj_arr) > 0:
        all_xy.append(traj_arr[:, 1:3])
    if pos_at:
        all_xy.append(np.array([pos_at[d][:2] for d in drone_ids]))
    if tgt_at is not None:
        all_xy.append(tgt_at[:2].reshape(1, 2))
    if waypoints_xy:
        all_xy.append(np.array(waypoints_xy))
    if all_xy:
        cxy = np.vstack(all_xy)
        pad = 4.0
        xmin = float(cxy[:, 0].min()) - pad
        xmax = float(cxy[:, 0].max()) + pad
        ymin = float(cxy[:, 1].min()) - pad
        ymax = float(cxy[:, 1].max()) + pad
    else:
        xmin, xmax, ymin, ymax = -5, 25, -10, 15

    # ── 地板：先渲染，z 低于障碍物最低点，障碍物在其上方 ────────────────────
    if obs_pts is not None and len(obs_pts) > 0:
        z_min = float(obs_pts[:, 2].min())
        z_max = float(obs_pts[:, 2].max())
    else:
        z_min, z_max = 0.0, 3.0

    z0 = z_min - 0.5
    floor = pv.Plane(center=((xmin+xmax)/2, (ymin+ymax)/2, z0),
                     direction=(0, 0, 1),
                     i_size=xmax-xmin, j_size=ymax-ymin,
                     i_resolution=1, j_resolution=1)
    pl.add_mesh(floor, color='#d8d8e8', opacity=0.6, show_edges=False)
    floor_grid = pv.Plane(center=((xmin+xmax)/2, (ymin+ymax)/2, z0+0.01),
                          direction=(0, 0, 1),
                          i_size=xmax-xmin, j_size=ymax-ymin,
                          i_resolution=max(2, int((xmax-xmin)//2)),
                          j_resolution=max(2, int((ymax-ymin)//2)))
    pl.add_mesh(floor_grid, color='#9999bb', opacity=0.4,
                style='wireframe', line_width=1)

    # ── 障碍物点云（地板之后渲染，在地板上方）────────────────────────────────
    if obs_pts is not None and len(obs_pts) > 0:
        cloud = pv.PolyData(obs_pts.astype(np.float32))
        cloud['z'] = obs_pts[:, 2].astype(np.float32)
        pl.add_mesh(cloud, scalars='z', cmap='plasma',
                    clim=[z_min, z_max],
                    point_size=3.0, render_points_as_spheres=True,
                    opacity=0.9, show_scalar_bar=False)

    pl.enable_eye_dome_lighting()

    # ── 规划路径：从 tgt_at 直接连第一个航点，再依次连后续航点 ───────────────
    if waypoints_xy and tgt_at is not None:
        z_plan = float(tgt_at[2])
        anchors = [tgt_at.copy()]
        for wx, wy in waypoints_xy:
            anchors.append(np.array([wx, wy, z_plan]))
        anchors = np.array(anchors, dtype=np.float64)

        if len(anchors) >= 2:
            # Catmull-Rom 平滑，ghost 点只影响切线，不影响起点
            ghost = anchors[0] - (anchors[1] - anchors[0]) * 0.5
            curve = catmull_rom(np.vstack([ghost, anchors]), n_per_seg=60)
            curve[0] = anchors[0]  # 强制起点 = tgt_at

            # 直接用密集点构建 PolyData 管道，不经过 pv.Spline
            # 跳过前几个点，视觉上截掉起始段的扭曲部分
            skip = 32
            draw_curve = curve[skip:]
            n = len(draw_curve)
            path_pd = pv.PolyData()
            path_pd.points = draw_curve.astype(np.float32)
            path_pd.lines = np.hstack([[n], np.arange(n)])
            path_tube = path_pd.tube(radius=0.12, n_sides=16)
            pl.add_mesh(path_tube, color=PLAN_COLOR, opacity=0.85)

            # 每段航点间放一个锥头箭头
            seg_len = np.linalg.norm(np.diff(curve, axis=0), axis=1)
            arc = np.concatenate([[0], np.cumsum(seg_len)])
            for k in range(len(anchors) - 1):
                frac = (k + 0.55) / (len(anchors) - 1)
                idx = int(np.searchsorted(arc, frac * arc[-1]))
                idx = min(idx, n - 2)
                dv = curve[idx+1] - curve[idx]
                nrm = np.linalg.norm(dv)
                if nrm < 0.01:
                    continue
                arrow = pv.Arrow(start=curve[idx], direction=dv/nrm,
                                 tip_length=0.5, tip_radius=0.25,
                                 shaft_radius=0.0, scale=1.8)
                pl.add_mesh(arrow, color=PLAN_COLOR, opacity=0.95)

            for pt in anchors[1:]:
                sphere = pv.Sphere(radius=0.35, center=pt)
                pl.add_mesh(sphere, color=PLAN_COLOR, opacity=1.0)

    # ── 追踪者图标 + FOV 棱线 ─────────────────────────────────────────────────
    if pos_at and quat_at:
        for d in drone_ids:
            pos = pos_at[d]
            quat = quat_at[d]
            mesh, _ = make_quad_mesh(pos[0], pos[1], pos[2],
                                     yaw=0.0, arm=1, rotor_r=0.16,
                                     tube_r=0.14, color=TRACKER_COLOR)
            pl.add_mesh(mesh, color=TRACKER_COLOR, opacity=1.0)

            if cam_model:
                R_wb = quat_to_rotmat(*quat)
                R_wc = R_wb @ cam_model['R_bc']
                rng_c = cam_model['range']
                hx = rng_c * math.tan(0.5 * cam_model['h_fov'])
                hy = rng_c * math.tan(0.5 * cam_model['v_fov'])
                far_c = np.array([[ hx, hy, rng_c], [-hx, hy, rng_c],
                                   [-hx,-hy, rng_c], [ hx,-hy, rng_c]])
                apex = np.array(pos)
                far_w = [apex + R_wc @ p for p in far_c]
                for fp in far_w:
                    e = pv.Line(apex, fp).tube(radius=0.04, n_sides=6)
                    pl.add_mesh(e, color=TRACKER_COLOR, opacity=0.55)
                for k in range(4):
                    e = pv.Line(far_w[k], far_w[(k+1)%4]).tube(radius=0.04, n_sides=6)
                    pl.add_mesh(e, color=TRACKER_COLOR, opacity=0.55)

    # ── 目标图标 ──────────────────────────────────────────────────────────────
    if tgt_at is not None:
        tgt_mesh, _ = make_quad_mesh(
            tgt_at[0], tgt_at[1], tgt_at[2],
            yaw=0.0, arm=0.8, rotor_r=0.14,
            tube_r=0.12, color=TARGET_COLOR)
        pl.add_mesh(tgt_mesh, color=TARGET_COLOR, opacity=1.0)

    # ── 编队连线 ──────────────────────────────────────────────────────────────
    if pos_at and len(drone_ids) >= 2:
        positions = [pos_at[d] for d in drone_ids]
        for i in range(len(positions)):
            j = (i + 1) % len(positions)
            tube = pv.Line(positions[i], positions[j]).tube(radius=0.06, n_sides=8)
            pl.add_mesh(tube, color=FORM_COLOR, opacity=0.65)

    # ── 颜色条 ────────────────────────────────────────────────────────────────
    dummy = pv.Line((xmin-200, 0, z_min), (xmin-200, 0, z_max), resolution=1)
    dummy['z'] = np.array([z_min, z_max], dtype=np.float32)
    pl.add_mesh(dummy, scalars='z', cmap='plasma',
                clim=[z_min, z_max], opacity=0.0,
                scalar_bar_args=dict(
                    title='高度 z (m)', title_font_size=20,
                    label_font_size=15, color='black',
                    vertical=True, position_x=0.90, position_y=0.20,
                    width=0.04, height=0.55))

    # ── 相机视角 ──────────────────────────────────────────────────────────────
    cx = (xmin + xmax) / 2
    cy = (ymin + ymax) / 2
    scene_w = xmax - xmin
    scene_h = ymax - ymin
    dist = max(scene_w, scene_h) * 1.4
    pl.camera.position    = (cx + dist * 0.5, cy - dist * 0.9, dist * 0.75)
    pl.camera.focal_point = (cx, cy, z_min + (z_max - z_min) * 0.4)
    pl.camera.up          = (0, 0, 1)

    pl.screenshot(str(out_path), transparent_background=False)
    pl.close()
    print(f"保存: {out_path}")


# ── CLI 入口 ──────────────────────────────────────────────────────────────────

# ── full_evasion.py 中的航点定义（直接同步）────────────────────────────────
_W0_BOT, _W0_TOP = -6.5,  6.5
_W1_BOT, _W1_TOP = -3.5,  9.5
_W2_BOT, _W2_TOP = -6.5,  6.5
_X_BEFORE_W0   =  7.0
_X_BETWEEN_01  = 11.0
_X_BETWEEN_12  = 15.0
_X_AFTER_W2    = 19.0

EVASION_WAYPOINTS = [
    (_X_BEFORE_W0,  _W0_BOT),
    (_X_BETWEEN_01, _W0_BOT),
    (_X_BETWEEN_01, _W1_TOP),
    (_X_BETWEEN_12, _W1_TOP),
    (_X_BETWEEN_12, _W2_BOT),
    (_X_AFTER_W2,   _W2_BOT),
    (_X_AFTER_W2,   _W2_TOP),
    (-5.0, -18.0),
    (-5.0,  18.0),
]


def parse_args():
    p = argparse.ArgumentParser(description='PyVista 高质量 3D 场景渲染')
    p.add_argument('bag', help='rosbag 路径')
    p.add_argument('--out', default='scene_pyvista.png', help='输出图片路径')
    p.add_argument('--camera-yaml', default=None,
                   help='camera.yaml 路径（可选，用于绘制FOV）')
    p.add_argument('--drones', default='0,1,2',
                   help='无人机ID列表，逗号分隔，默认 0,1,2')
    p.add_argument('--waypoints', default=None,
                   help='覆盖航点：JSON 文件，格式 [[x,y],...]；默认使用 full_evasion 航点')
    p.add_argument('--width',  type=int, default=1920)
    p.add_argument('--height', type=int, default=1080)
    return p.parse_args()


def main():
    args = parse_args()
    drone_ids = [int(x) for x in args.drones.split(',')]

    cam_model = None
    if args.camera_yaml:
        cam_model = load_camera_model(args.camera_yaml)

    if args.waypoints:
        import json
        with open(args.waypoints) as f:
            waypoints_xy = json.load(f)
    else:
        waypoints_xy = EVASION_WAYPOINTS

    print(f"读取 bag: {args.bag}")
    obs_pts, traj_arr, drone_hist, start_ts = load_bag(
        Path(args.bag), drone_ids)

    print("选取快照时刻...")
    best_ts, pos_at, quat_at, tgt_at = pick_snapshot(
        traj_arr, drone_hist, drone_ids, start_ts=start_ts or 0.0)
    print(f"  快照时刻: t={best_ts:.3f}s  目标位置: {tgt_at}")

    render_scene(
        obs_pts, traj_arr, drone_hist, drone_ids,
        waypoints_xy=waypoints_xy,
        cam_model=cam_model,
        out_path=Path(args.out),
        snapshot_ts=best_ts,
        pos_at=pos_at,
        quat_at=quat_at,
        tgt_at=tgt_at,
        window_size=(args.width, args.height),
    )


if __name__ == '__main__':
    main()
