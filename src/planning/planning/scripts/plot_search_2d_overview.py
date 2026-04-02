#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D俯视图：障碍物 + 目标轨迹 + 无人机位置/搜索方向
围绕指定失锁事件绘制前后一段时间的场景。

用法:
  python3 plot_search_2d_overview.py \
    --bag /path/to/tracking.bag \
    --loss_idx 5 \
    --before 3.0 \
    --after 10.0 \
    --out_dir ./search_2d_out
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from pathlib import Path
from matplotlib import font_manager as fm

try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("需要rosbag: source devel/setup.bash")
    sys.exit(1)

# ── 颜色 ──────────────────────────────────────────────────────────────────────
DRONE_COLOR  = '#2980b9'
DRONE_COLORS = [DRONE_COLOR, DRONE_COLOR, DRONE_COLOR]
TARGET_COLOR = '#e74c3c'
OBS_COLOR    = '#444455'
LOSS_COLOR   = '#cc2222'
REACQ_COLOR  = '#22aa55'
BG_COLOR     = 'white'

plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC', 'WenQuanYi Zen Hei',
    'Microsoft YaHei', 'SimHei', 'DejaVu Sans',
]


def get_chinese_font():
    candidates = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    ]
    for p in candidates:
        if Path(p).exists():
            return fm.FontProperties(fname=p)
    return None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', default=
        '/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/'
        'experiment_results/参考bag包/run_001/tracking.bag')
    p.add_argument('--loss_idx', type=int, default=5,
                   help='第几次失锁（从1开始）')
    p.add_argument('--before', type=float, default=3.0,
                   help='失锁前绘制多少秒')
    p.add_argument('--after', type=float, default=10.0,
                   help='失锁后绘制多少秒')
    p.add_argument('--n_frames', type=int, default=6,
                   help='在时间窗口内均匀采样多少帧')
    p.add_argument('--obs_max_pts', type=int, default=30000,
                   help='障碍物点云最大采样点数')
    p.add_argument('--out_dir', default='./search_2d_out')
    return p.parse_args()


# ── 2D 四旋翼图标 ─────────────────────────────────────────────────────────────
def draw_quad_2d(ax, x, y, heading_rad=0.0, arm=0.6, rotor_r=0.12,
                 color='#1f77b4', lw=1.4, alpha=1.0, zorder=6):
    """在2D轴上绘制四旋翼俯视图标（十字臂 + 4个旋翼圆）。
    heading_rad: 机头朝向角（弧度，相对X轴），用于旋转图标。
    """
    cos_h, sin_h = np.cos(heading_rad), np.sin(heading_rad)

    def rot(dx, dy):
        return x + cos_h * dx - sin_h * dy, y + sin_h * dx + cos_h * dy

    # 十字臂
    x0, y0 = rot(-arm, 0); x1, y1 = rot(arm, 0)
    ax.plot([x0, x1], [y0, y1], color=color, lw=lw, alpha=alpha, zorder=zorder)
    x0, y0 = rot(0, -arm); x1, y1 = rot(0, arm)
    ax.plot([x0, x1], [y0, y1], color=color, lw=lw, alpha=alpha, zorder=zorder)

    # 4个旋翼圆
    th = np.linspace(0, 2 * np.pi, 32)
    for adx, ady in [(-arm, 0), (arm, 0), (0, -arm), (0, arm)]:
        cx, cy = rot(adx, ady)
        ax.plot(cx + rotor_r * np.cos(th), cy + rotor_r * np.sin(th),
                color=color, lw=max(0.8, lw * 0.85), alpha=alpha, zorder=zorder)

    # 机头方向小三角
    tip_x, tip_y = rot(arm * 1.25, 0)
    ax.plot([x, tip_x], [y, tip_y], color=color, lw=lw * 0.7,
            alpha=alpha * 0.7, zorder=zorder, ls='--')


def draw_target_2d(ax, x, y, r=0.35, color=TARGET_COLOR, alpha=1.0, zorder=7):
    """目标图标：红色同心圆"""
    for ri, lw, fill in [(r, 0, True), (r * 1.8, 1.6, False), (r * 2.8, 1.0, False)]:
        if fill:
            ax.add_patch(plt.Circle((x, y), ri, color=color, fill=True,
                                    alpha=alpha * 0.9, zorder=zorder))
        else:
            ax.add_patch(plt.Circle((x, y), ri, color=color, fill=False,
                                    lw=lw, alpha=alpha * 0.7, zorder=zorder))


# ── 相机参数（与 camera.yaml 一致）────────────────────────────────────────────
import math
_CAM_RANGE = 7.0
_CAM_H_FOV = 2.0 * math.atan(640 / (2.0 * 387.229248046875))   # ≈ 79.3°
_CAM_V_FOV = 2.0 * math.atan(480 / (2.0 * 387.229248046875))   # ≈ 63.0°
_R_BC = np.array([[ 0., 0., 1.],
                  [-1., 0., 0.],
                  [ 0.,-1., 0.]], dtype=np.float64)


def _quat_to_rotmat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def draw_fov_2d(ax, drone_x, drone_y, drone_z, qx, qy, qz, qw,
                color='#2980b9', alpha=0.18, zorder=3):
    """将相机 FOV 锥体四个远端角点投影到 XY 平面，绘制扇形轮廓。"""
    R_wb = _quat_to_rotmat(qx, qy, qz, qw)
    R_wc = R_wb @ _R_BC

    hx = _CAM_RANGE * math.tan(0.5 * _CAM_H_FOV)
    hy = _CAM_RANGE * math.tan(0.5 * _CAM_V_FOV)

    far_cam = np.array([[ hx,  hy, _CAM_RANGE],
                        [-hx,  hy, _CAM_RANGE],
                        [-hx, -hy, _CAM_RANGE],
                        [ hx, -hy, _CAM_RANGE]], dtype=np.float64)

    apex_w = np.array([drone_x, drone_y, drone_z])
    far_xy = np.array([(apex_w + R_wc @ pt)[:2] for pt in far_cam])
    apex_xy = apex_w[:2]

    # 填充扇形
    ax.fill(np.append(far_xy[:, 0], far_xy[0, 0]),
            np.append(far_xy[:, 1], far_xy[0, 1]),
            color=color, alpha=alpha, zorder=zorder, linewidth=0)
    # 四条棱线
    for corner in far_xy:
        ax.plot([apex_xy[0], corner[0]], [apex_xy[1], corner[1]],
                color=color, lw=0.8, alpha=min(1.0, alpha * 3), zorder=zorder)
    # 远端闭合框
    ax.plot(np.append(far_xy[:, 0], far_xy[0, 0]),
            np.append(far_xy[:, 1], far_xy[0, 1]),
            color=color, lw=0.8, alpha=min(1.0, alpha * 3), zorder=zorder)


# ── Bag 读取 ──────────────────────────────────────────────────────────────────
def load_bag(bag_path, obs_max_pts=30000):
    bag = rosbag.Bag(bag_path)
    t0 = bag.get_start_time()

    search_states  = {d: [] for d in range(3)}
    search_targets = {d: [] for d in range(3)}
    target_odom    = []
    drone_odom     = {d: [] for d in range(3)}
    drone_orient   = {d: [] for d in range(3)}  # (rel_t, qx, qy, qz, qw)
    drone_vel      = {d: [] for d in range(3)}  # (rel_t, vx, vy)
    obs_pts_xy     = None  # 只取一帧障碍物点云

    topics = (
        [f'/drone{d}/drone{d}_target_dpf/search_state'   for d in range(3)] +
        [f'/drone{d}/drone{d}_target_dpf/search_targets' for d in range(3)] +
        [f'/drone{d}/odom' for d in range(3)] +
        ['/target/odom', '/global_map']
    )

    for topic, msg, t in bag.read_messages(topics=topics):
        rel = t.to_sec() - t0
        d_str = topic.split('/')[1]
        d = int(d_str.replace('drone', '')) if d_str.startswith('drone') else -1

        if 'search_state' in topic and d >= 0:
            search_states[d].append((rel, msg.data))

        elif 'search_targets' in topic and d >= 0:
            pts = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
            search_targets[d].append((rel, pts))

        elif topic == '/target/odom':
            p = msg.pose.pose.position
            target_odom.append((rel, p.x, p.y, p.z))

        elif '/odom' in topic and 'target' not in topic and d >= 0:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            drone_odom[d].append((rel, p.x, p.y, p.z))
            drone_orient[d].append((rel, q.x, q.y, q.z, q.w))
            drone_vel[d].append((rel, v.x, v.y))

        elif topic == '/global_map' and obs_pts_xy is None:
            pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                       skip_nans=True))
            if pts:
                arr = np.array(pts)
                if len(arr) > obs_max_pts:
                    idx = np.random.choice(len(arr), obs_max_pts, replace=False)
                    arr = arr[idx]
                obs_pts_xy = arr[:, :2]

    bag.close()
    print(f"  drone_odom records: {[len(drone_odom[d]) for d in range(3)]}")
    print(f"  target_odom records: {len(target_odom)}")
    print(f"  obs_pts: {len(obs_pts_xy) if obs_pts_xy is not None else 0}")
    return search_states, search_targets, target_odom, drone_odom, drone_orient, obs_pts_xy


def find_loss_intervals(search_states):
    d0 = sorted(search_states[0])
    prev_val = False
    loss_start = None
    intervals = []
    for t, v in d0:
        if v and not prev_val:
            loss_start = t
        elif not v and prev_val and loss_start is not None:
            intervals.append((loss_start, t, t - loss_start))
            loss_start = None
        prev_val = v
    return intervals


def nearest(arr, t):
    if not arr:
        return None
    idx = np.searchsorted([x[0] for x in arr], t)
    idx = min(idx, len(arr) - 1)
    return arr[idx]


def quat_to_yaw(qx, qy, qz, qw):
    """从四元数提取偏航角（绕Z轴）"""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny, cosy)


# ── 单帧绘制 ──────────────────────────────────────────────────────────────────
def plot_frame(ax, t_query, target_odom, drone_odom, drone_orient,
               obs_pts_xy, t_loss_start, t_loss_end,
               t_win_start, t_win_end, title, fp=None):
    ax.set_facecolor(BG_COLOR)
    ax.set_aspect('equal')

    # 障碍物点云
    if obs_pts_xy is not None and len(obs_pts_xy) > 0:
        ax.scatter(obs_pts_xy[:, 0], obs_pts_xy[:, 1],
                   s=8, c=OBS_COLOR, alpha=0.7, linewidths=0, zorder=1)

    # 目标轨迹（时间窗口内，搜索期间红色，追踪期间橙色）
    tgt_arr = np.array(target_odom)
    mask = (tgt_arr[:, 0] >= t_win_start) & (tgt_arr[:, 0] <= t_win_end)
    tgt_seg = tgt_arr[mask]
    if len(tgt_seg) > 1:
        for i in range(len(tgt_seg) - 1):
            t_mid = (tgt_seg[i, 0] + tgt_seg[i+1, 0]) / 2
            in_s = t_loss_start <= t_mid <= t_loss_end
            c = '#e74c3c' if in_s else '#e67e22'
            ax.plot(tgt_seg[i:i+2, 1], tgt_seg[i:i+2, 2],
                    c=c, lw=1.5, alpha=0.8, zorder=2)

    # 无人机图标 + FOV
    for d in range(3):
        entry = nearest(drone_odom[d], t_query)
        o_entry = nearest(drone_orient[d], t_query)
        if not entry:
            continue
        yaw = quat_to_yaw(*o_entry[1:]) if o_entry else 0.0

        # FOV 投影（始终绘制）
        if o_entry:
            draw_fov_2d(ax, entry[1], entry[2], entry[3],
                        o_entry[1], o_entry[2], o_entry[3], o_entry[4],
                        color=DRONE_COLOR, alpha=0.15, zorder=3)

        draw_quad_2d(ax, entry[1], entry[2], heading_rad=yaw,
                     arm=0.55, rotor_r=0.11,
                     color=DRONE_COLOR, lw=1.3, zorder=6)
        kw = dict(fontsize=6, color=DRONE_COLOR, zorder=7, ha='center', va='bottom')
        if fp:
            kw['fontproperties'] = fp
        ax.text(entry[1], entry[2] + 0.75, f'D{d}', **kw)

    # 目标图标（当前时刻）
    entry = nearest(target_odom, t_query)
    if entry:
        draw_target_2d(ax, entry[1], entry[2], r=0.32, color=TARGET_COLOR, zorder=7)

    # 失锁/重获标记（轨迹上）
    t_loss_entry = nearest(target_odom, t_loss_start)
    t_reacq_entry = nearest(target_odom, t_loss_end)
    if t_loss_entry and t_win_start <= t_loss_start <= t_win_end:
        ax.plot(t_loss_entry[1], t_loss_entry[2], 'x',
                ms=10, mew=2.0, color=LOSS_COLOR, zorder=8)
    if t_reacq_entry and t_win_start <= t_loss_end <= t_win_end:
        ax.plot(t_reacq_entry[1], t_reacq_entry[2], '+',
                ms=10, mew=2.0, color=REACQ_COLOR, zorder=8)

    # 标题
    in_search = t_loss_start <= t_query <= t_loss_end
    status = f"SEARCH +{t_query - t_loss_start:.1f}s" if in_search else (
        f"PRE -{t_loss_start - t_query:.1f}s" if t_query < t_loss_start else
        f"POST +{t_query - t_loss_end:.1f}s"
    )
    title_kw = dict(fontsize=8, color='black', pad=3)
    if fp:
        title_kw['fontproperties'] = fp
    ax.set_title(f"{title}\n{status}", **title_kw)
    ax.tick_params(colors='black', labelsize=6)
    for spine in ax.spines.values():
        spine.set_edgecolor('#aaaaaa')


# ── 主函数 ────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    fp = get_chinese_font()

    print(f"Loading bag: {args.bag}")
    (search_states, search_targets, target_odom,
     drone_odom, drone_orient, obs_pts_xy) = load_bag(args.bag, args.obs_max_pts)

    intervals = find_loss_intervals(search_states)
    print(f"\nFound {len(intervals)} loss events:")
    for i, (s, e, dur) in enumerate(intervals):
        print(f"  #{i+1}: t={s:.3f}s ~ {e:.3f}s  dur={dur:.3f}s")

    if args.loss_idx < 1 or args.loss_idx > len(intervals):
        print(f"loss_idx={args.loss_idx} out of range [1, {len(intervals)}]")
        sys.exit(1)

    t_loss_start, t_loss_end, loss_dur = intervals[args.loss_idx - 1]
    t_win_start = t_loss_start - args.before
    t_win_end   = t_loss_start + args.after
    print(f"\nLoss #{args.loss_idx}: t={t_loss_start:.3f}s, dur={loss_dur:.3f}s")
    print(f"Window: [{t_win_start:.3f}s, {t_win_end:.3f}s]")

    # 确定绘图范围
    tgt_arr = np.array(target_odom)
    mask = (tgt_arr[:, 0] >= t_win_start) & (tgt_arr[:, 0] <= t_win_end)
    xlim = (3, 26)
    ylim = (-7, 16)

    # ── 多帧网格图 ────────────────────────────────────────────────────────────
    frame_times = np.linspace(t_win_start, t_win_end, args.n_frames)
    ncols = min(args.n_frames, 3)
    nrows = (args.n_frames + ncols - 1) // ncols

    fig = plt.figure(figsize=(ncols * 4.5, nrows * 4.5 + 1.2), facecolor=BG_COLOR)
    title_str = f'搜索模式2D俯视  |  第{args.loss_idx}次失锁  (t={t_loss_start:.2f}s, 持续{loss_dur:.2f}s)'
    title_kw = dict(color='black', fontsize=12, y=0.99)
    if fp:
        title_kw['fontproperties'] = fp
    fig.suptitle(title_str, **title_kw)

    for i, t_q in enumerate(frame_times):
        ax = fig.add_subplot(nrows, ncols, i + 1)
        plot_frame(ax, t_q, target_odom, drone_odom, drone_orient,
                   obs_pts_xy,
                   t_loss_start, t_loss_end, t_win_start, t_win_end,
                   f't={t_q:.2f}s', fp=fp)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_xlabel('X (m)', fontsize=6, color='black')
        ax.set_ylabel('Y (m)', fontsize=6, color='black')

    # 图例
    legend_elements = [
        Line2D([0], [0], color=DRONE_COLOR, lw=1.5, label='无人机'),
        mpatches.Circle((0.5, 0.5), 0.25, color=TARGET_COLOR, label='目标'),
        Line2D([0], [0], color='#e74c3c', lw=1.5, label='目标轨迹(搜索)'),
        Line2D([0], [0], color='#e67e22', lw=1.5, label='目标轨迹(追踪)'),
        Line2D([0], [0], marker='x', color=LOSS_COLOR, ms=8, lw=0, label='失锁时刻'),
        Line2D([0], [0], marker='+', color=REACQ_COLOR, ms=8, lw=0, label='重获时刻'),
        mpatches.Patch(color=OBS_COLOR, alpha=0.8, label='障碍物'),
    ]
    legend_kw = dict(loc='lower center', ncol=4, facecolor='white',
                     edgecolor='#aaaaaa', fontsize=8,
                     bbox_to_anchor=(0.5, 0.005))
    if fp:
        legend_kw['prop'] = fp
    fig.legend(handles=legend_elements, **legend_kw)

    plt.tight_layout(rect=[0, 0.06, 1, 0.97])
    out_path = os.path.join(args.out_dir, f'search_2d_loss{args.loss_idx}.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f"\n保存: {out_path}")

    # ── 单张大图（关键时刻图标）────────────────────────
    fig2, ax2 = plt.subplots(figsize=(9, 9), facecolor=BG_COLOR)
    ax2.set_facecolor(BG_COLOR)
    ax2.set_aspect('equal')

    # 障碍物
    if obs_pts_xy is not None and len(obs_pts_xy) > 0:
        ax2.scatter(obs_pts_xy[:, 0], obs_pts_xy[:, 1],
                    s=8, c=OBS_COLOR, alpha=0.7, linewidths=0, zorder=1)

    # 目标轨迹（搜索期间红色，追踪期间橙色）
    tgt_arr2 = np.array(target_odom)
    mask2 = (tgt_arr2[:, 0] >= t_win_start) & (tgt_arr2[:, 0] <= t_win_end)
    tgt_seg2 = tgt_arr2[mask2]
    if len(tgt_seg2) > 1:
        for i in range(len(tgt_seg2) - 1):
            t_mid = (tgt_seg2[i, 0] + tgt_seg2[i+1, 0]) / 2
            in_s = t_loss_start <= t_mid <= t_loss_end
            c = '#e74c3c' if in_s else '#e67e22'
            ax2.plot(tgt_seg2[i:i+2, 1], tgt_seg2[i:i+2, 2],
                     c=c, lw=2.0, alpha=0.85, zorder=2)

    # 关键时刻图标：失锁前、失锁中、重获后
    key_times = [t_loss_start - args.before * 0.5,
                 t_loss_start,
                 (t_loss_start + t_loss_end) / 2,
                 t_loss_end,
                 min(t_loss_end + args.after * 0.3, t_win_end)]
    key_alphas = [0.4, 0.7, 1.0, 0.7, 0.4]

    for t_q, alpha in zip(key_times, key_alphas):
        for d in range(3):
            entry = nearest(drone_odom[d], t_q)
            o_entry = nearest(drone_orient[d], t_q)
            if entry:
                yaw = quat_to_yaw(*o_entry[1:]) if o_entry else 0.0
                if o_entry:
                    draw_fov_2d(ax2, entry[1], entry[2], entry[3],
                                o_entry[1], o_entry[2], o_entry[3], o_entry[4],
                                color=DRONE_COLOR, alpha=alpha * 0.15, zorder=3)
                draw_quad_2d(ax2, entry[1], entry[2], heading_rad=yaw,
                             arm=0.5, rotor_r=0.10,
                             color=DRONE_COLOR, lw=1.2, alpha=alpha, zorder=5)

        tgt_entry = nearest(target_odom, t_q)
        if tgt_entry:
            draw_target_2d(ax2, tgt_entry[1], tgt_entry[2],
                           r=0.28, color=TARGET_COLOR, alpha=alpha, zorder=6)

    # 失锁/重获标记
    t_loss_entry = nearest(target_odom, t_loss_start)
    t_reacq_entry = nearest(target_odom, t_loss_end)
    if t_loss_entry:
        ax2.plot(t_loss_entry[1], t_loss_entry[2], 'x',
                 ms=14, mew=2.5, color=LOSS_COLOR, zorder=9, label='失锁时刻')
    if t_reacq_entry:
        ax2.plot(t_reacq_entry[1], t_reacq_entry[2], '+',
                 ms=14, mew=2.5, color=REACQ_COLOR, zorder=9, label='重获时刻')

    title2_str = f'场景总览 — 第{args.loss_idx}次失锁 (持续{loss_dur:.2f}s)'
    title2_kw = dict(color='black', fontsize=11)
    if fp:
        title2_kw['fontproperties'] = fp
    ax2.set_title(title2_str, **title2_kw)
    ax2.set_xlabel('X (m)', color='black', fontsize=10)
    ax2.set_ylabel('Y (m)', color='black', fontsize=10)
    ax2.tick_params(colors='black')
    ax2.set_xlim(*xlim)
    ax2.set_ylim(*ylim)
    for spine in ax2.spines.values():
        spine.set_edgecolor('#aaaaaa')

    legend2_kw = dict(facecolor='white', edgecolor='#aaaaaa', fontsize=9)
    if fp:
        legend2_kw['prop'] = fp
    ax2.legend(**legend2_kw)

    plt.tight_layout()
    out_path2 = os.path.join(args.out_dir, f'search_2d_overview_loss{args.loss_idx}.png')
    plt.savefig(out_path2, dpi=150, bbox_inches='tight', facecolor=fig2.get_facecolor())
    plt.close()
    print(f"保存: {out_path2}")

    print(f"\n全部完成，输出目录: {args.out_dir}")


if __name__ == '__main__':
    main()
