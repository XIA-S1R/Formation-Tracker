#!/usr/bin/env python3
import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# 中文字体 — matplotlib 3.1 兼容方式，直接注册 .ttc 字体文件
import matplotlib.font_manager as _fm
_CJK_CANDIDATES = [
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    '/usr/share/fonts/truetype/arphic/ukai.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
]
_cjk_registered = False
for _p in _CJK_CANDIDATES:
    if os.path.exists(_p):
        _fm.fontManager.ttflist += _fm.createFontList([_p])
        _cjk_name = _fm.FontProperties(fname=_p).get_name()
        matplotlib.rcParams['font.sans-serif'] = [_cjk_name, 'DejaVu Sans', 'sans-serif']
        _cjk_registered = True
        break
if not _cjk_registered:
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

# ROS bag读取
try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("需要rosbag: source devel/setup.bash")
    sys.exit(1)

DRONE_COLORS = ['#e74c3c', '#2ecc71', '#3498db']  # drone0/1/2
DRONE_NAMES  = ['Drone 0', 'Drone 1', 'Drone 2']


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', default=
        '/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/'
        'experiment_results/参考bag包/run_001/tracking.bag')
    p.add_argument('--loss_idx', type=int, default=5,
                   help='第几次失锁（从1开始），默认第5次（最长8.35s）')
    p.add_argument('--duration', type=float, default=12.0,
                   help='从失锁开始绘制多少秒')
    p.add_argument('--n_frames', type=int, default=8,
                   help='在duration内均匀采样多少帧绘制')
    p.add_argument('--out_dir', default='./search_heatmap_out')
    return p.parse_args()


def load_bag(bag_path):
    """加载bag，返回各topic的时间序列数据"""
    bag = rosbag.Bag(bag_path)
    t0 = bag.get_start_time()

    search_states  = {d: [] for d in range(3)}
    search_gmm     = {d: [] for d in range(3)}
    invalid_gmms   = {d: [] for d in range(3)}
    search_targets = {d: [] for d in range(3)}
    target_odom    = []
    drone_odom     = {d: [] for d in range(3)}
    obstacle_xy    = None   # Nx2 障碍物XY点云（只取第一帧）

    topics = (
        [f'/drone{d}/drone{d}_target_dpf/search_state'       for d in range(3)] +
        [f'/drone{d}/drone{d}_target_dpf/search_pos_gmm'     for d in range(3)] +
        [f'/drone{d}/drone{d}_target_dpf/invalid_region_gmm' for d in range(3)] +
        [f'/drone{d}/drone{d}_target_dpf/search_targets'     for d in range(3)] +
        [f'/drone{d}/odom' for d in range(3)] +
        ['/target/odom', '/global_map']
    )

    for topic, msg, t in bag.read_messages(topics=topics):
        rel = t.to_sec() - t0
        d_str = topic.split('/')[1]
        d = int(d_str.replace('drone', '')) if d_str.startswith('drone') else -1

        if 'search_state' in topic and d >= 0:
            search_states[d].append((rel, msg.data))

        elif 'search_pos_gmm' in topic and d >= 0:
            # 6D GMM: mu_c = zeta_a[c] / zeta_alpha[c]
            C = msg.num_components
            if C > 0 and len(msg.zeta_alpha) >= C and len(msg.zeta_a) >= C * 6:
                weights = np.array(msg.zeta_alpha[:C])
                means = []
                for c in range(C):
                    alpha_c = weights[c]
                    if abs(alpha_c) > 1e-30:
                        mu = np.array(msg.zeta_a[c*6:(c+1)*6]) / alpha_c
                    else:
                        mu = np.zeros(6)
                    means.append(mu[:3])  # 只取位置
                w_sum = weights.sum()
                if w_sum > 1e-30:
                    weights = weights / w_sum
                search_gmm[d].append((rel, weights, np.array(means)))

        elif 'invalid_region_gmm' in topic and d >= 0:
            C = msg.num_components
            if C > 0 and len(msg.weights) >= C and len(msg.means) >= C * 3:
                weights = np.array(msg.weights[:C])
                means = np.array(msg.means[:C*3]).reshape(C, 3)
                search_gmm_type = msg.type  # 0=neg_obs, 1=obstacle
                invalid_gmms[d].append((rel, search_gmm_type, weights, means))

        elif 'search_targets' in topic and d >= 0:
            pts = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
            search_targets[d].append((rel, pts))

        elif topic == '/target/odom':
            p = msg.pose.pose.position
            target_odom.append((rel, p.x, p.y, p.z))

        elif '/odom' in topic and 'target' not in topic and d >= 0:
            p = msg.pose.pose.position
            drone_odom[d].append((rel, p.x, p.y, p.z))

        elif topic == '/global_map' and obstacle_xy is None:
            pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            if pts:
                arr = np.array(pts)
                obstacle_xy = arr[:, :2]

    bag.close()
    print(f"  search_gmm records: {[len(search_gmm[d]) for d in range(3)]}")
    print(f"  invalid_gmm records: {[len(invalid_gmms[d]) for d in range(3)]}")
    print(f"  obstacle points: {len(obstacle_xy) if obstacle_xy is not None else 0}")
    return search_states, search_gmm, invalid_gmms, search_targets, target_odom, drone_odom, obstacle_xy


def find_loss_intervals(search_states):
    """用drone0的搜索状态找失锁区间"""
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


def extract_gmm_positions(search_gmm, t_start, t_end):
    """
    从search_pos_gmm中提取[t_start, t_end]内的GMM分量位置和权重。
    返回: list of (rel_t, drone_id, weights, positions_Nx3)
    """
    results = []
    for d in range(3):
        for rel_t, weights, positions in search_gmm[d]:
            if rel_t < t_start or rel_t > t_end:
                continue
            results.append((rel_t, d, weights, positions))
    return results


def nearest(arr, t):
    """在arr（sorted list of (t, ...)）中找最近时刻的条目"""
    if not arr:
        return None
    idx = np.searchsorted([x[0] for x in arr], t)
    idx = min(idx, len(arr) - 1)
    return arr[idx]


DRONE_CMAPS = [
    matplotlib.colors.LinearSegmentedColormap.from_list('r', ['#1a1a2e', '#e74c3c']),
    matplotlib.colors.LinearSegmentedColormap.from_list('g', ['#1a1a2e', '#2ecc71']),
    matplotlib.colors.LinearSegmentedColormap.from_list('b', ['#1a1a2e', '#3498db']),
]


def gmm_density_grid(positions, weights, xx, yy, bandwidth=0.8):
    """在网格上计算加权GMM密度（XY平面）"""
    density = np.zeros_like(xx)
    w_sum = weights.sum()
    if w_sum < 1e-30:
        weights = np.ones(len(positions)) / len(positions)
        w_sum = 1.0
    for i, pos in enumerate(positions):
        dx = xx - pos[0]
        dy = yy - pos[1]
        r2 = (dx**2 + dy**2) / (bandwidth**2)
        density += (weights[i] / w_sum) * np.exp(-0.5 * r2)
    return density


def plot_frame(ax, t_query, gmm_data, target_odom, drone_odom,
               t_loss_start, t_loss_end, title, xlim, ylim,
               obstacle_xy=None):
    """绘制单帧的XY平面热力图"""
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a2e')

    # 障碍物点云（灰色小点，z方向投影到XY平面）
    if obstacle_xy is not None and len(obstacle_xy) > 0:
        ax.scatter(obstacle_xy[:, 0], obstacle_xy[:, 1],
                   s=0.3, c='#555577', alpha=0.4, linewidths=0, zorder=1)

    # 网格
    res = 80
    xs = np.linspace(xlim[0], xlim[1], res)
    ys = np.linspace(ylim[0], ylim[1], res)
    xx, yy = np.meshgrid(xs, ys)

    # 找该时刻最近的GMM数据（每架无人机），叠加热力图
    for d in range(3):
        drone_gmms = [(t, w, p) for t, dd, w, p in gmm_data if dd == d]
        if not drone_gmms:
            continue
        idx = np.searchsorted([x[0] for x in drone_gmms], t_query)
        idx = min(idx, len(drone_gmms) - 1)
        _, weights, positions = drone_gmms[idx]
        if len(positions) == 0:
            continue

        density = gmm_density_grid(positions, weights, xx, yy, bandwidth=0.8)
        if density.max() < 1e-30:
            continue
        density /= density.max()
        density = np.power(density, 0.1)  # sqrt 拉伸：突出高权重峰值，压低背景散漫区域

        rgba = DRONE_CMAPS[d](density)
        rgba[..., 3] = density * 0.65
        ax.imshow(rgba, extent=[xlim[0], xlim[1], ylim[0], ylim[1]],
                  origin='lower', aspect='auto', zorder=2)

    # 绘制无人机当前位置
    for d in range(3):
        entry = nearest(drone_odom[d], t_query)
        if entry:
            ax.plot(entry[1], entry[2], marker='^', ms=10,
                    color=DRONE_COLORS[d], zorder=5,
                    markeredgecolor='white', markeredgewidth=0.8)

    # 绘制目标真实位置
    entry = nearest(target_odom, t_query)
    if entry:
        in_search = t_loss_start <= t_query <= t_loss_end
        color = '#ff6b6b' if in_search else '#ffd700'
        ax.plot(entry[1], entry[2], marker='*', ms=14,
                color=color, zorder=6,
                markeredgecolor='white', markeredgewidth=0.8)

    # 搜索状态标注
    in_search = t_loss_start <= t_query <= t_loss_end
    status = f"SEARCH +{t_query - t_loss_start:.1f}s" if in_search else "TRACKING"
    ax.set_title(f"{title}\n{status}", fontsize=8, color='white', pad=3)
    ax.tick_params(colors='gray', labelsize=6)
    for spine in ax.spines.values():
        spine.set_edgecolor('#444444')


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"Loading bag: {args.bag}")
    search_states, search_gmm, invalid_gmms, search_targets, target_odom, drone_odom, obstacle_xy = load_bag(args.bag)

    # 找失锁区间
    intervals = find_loss_intervals(search_states)
    print(f"\nFound {len(intervals)} loss events:")
    for i, (s, e, dur) in enumerate(intervals):
        print(f"  #{i+1}: t={s:.3f}s ~ {e:.3f}s  dur={dur:.3f}s")

    if args.loss_idx < 1 or args.loss_idx > len(intervals):
        print(f"loss_idx={args.loss_idx} out of range [1, {len(intervals)}]")
        sys.exit(1)

    t_loss_start, t_loss_end, loss_dur = intervals[args.loss_idx - 1]
    t_plot_end = t_loss_start + args.duration
    print(f"\nAnalyzing loss #{args.loss_idx}: t={t_loss_start:.3f}s, dur={loss_dur:.3f}s")
    print(f"Plot range: [{t_loss_start:.3f}s, {t_plot_end:.3f}s]")

    # 提取GMM数据
    print("Extracting GMM data...")
    gmm_data = extract_gmm_positions(search_gmm,
                                     t_loss_start - 1.0,
                                     t_plot_end + 1.0)
    print(f"  {len(gmm_data)} GMM records")

    print(f"\n=== GMM分量诊断 (失锁后 {args.duration:.0f}s 内) ===")
    for d in range(3):
        entries = [(t, w, p) for t, dd, w, p in gmm_data if dd == d
                   and t_loss_start <= t <= t_plot_end]
        if not entries:
            print(f"  Drone {d}: 无数据")
            continue
        # 每隔2秒打印一次
        last_print = -999.0
        for t, weights, positions in sorted(entries, key=lambda x: x[0]):
            if t - last_print < 2.0:
                continue
            last_print = t
            print(f"  Drone {d} t={t:.2f}s: {len(positions)}个分量")
            for c, (w, pos) in enumerate(zip(weights, positions)):
                print(f"    [c{c}] w={w:.3f}  pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

    # 打印目标消失位置
    loss_entries = [(t, x, y, z) for t, x, y, z in target_odom
                    if abs(t - t_loss_start) < 0.5]
    if loss_entries:
        _, lx, ly, lz = min(loss_entries, key=lambda e: abs(e[0] - t_loss_start))
        print(f"\n  目标消失位置: ({lx:.2f}, {ly:.2f}, {lz:.2f})")
    print("=" * 50)

    # 均匀采样帧时刻
    frame_times = np.linspace(t_loss_start, t_plot_end, args.n_frames)

    # 固定40×40全地图范围
    xlim = (-20.0, 20.0)
    ylim = (-20.0, 20.0)

    # 创建主图
    ncols = min(4, args.n_frames)
    nrows = (args.n_frames + ncols - 1) // ncols
    fig, axes_arr = plt.subplots(nrows, ncols,
                                 figsize=(ncols * 4, nrows * 4 + 0.8),
                                 facecolor='#0d0d1a')
    fig.suptitle(f'搜索GMM热力图 — 第{args.loss_idx}次失锁 (持续{loss_dur:.2f}s)',
                 color='white', fontsize=12)
    axes = np.array(axes_arr).flatten()
    # 隐藏多余子图
    for j in range(args.n_frames, len(axes)):
        axes[j].set_visible(False)

    for i, t_q in enumerate(frame_times):
        ax = axes[i]
        plot_frame(ax, t_q, gmm_data, target_odom, drone_odom,
                   t_loss_start, t_loss_end,
                   f't={t_q:.2f}s', xlim, ylim,
                   obstacle_xy=obstacle_xy)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_xlabel('X (m)', fontsize=6, color='gray')
        ax.set_ylabel('Y (m)', fontsize=6, color='gray')

    # 图例
    legend_elements = [
        mpatches.Patch(color=DRONE_COLORS[d], label=f'Drone {d} 搜索密度') for d in range(3)
    ] + [
        plt.Line2D([0], [0], marker='^', color='w', ms=8, label='无人机位置',
                   markerfacecolor='white', linestyle='None'),
        plt.Line2D([0], [0], marker='*', color='w', ms=10, label='目标(搜索中)',
                   markerfacecolor='#ff6b6b', linestyle='None'),
        plt.Line2D([0], [0], marker='*', color='w', ms=10, label='目标(已追踪)',
                   markerfacecolor='#ffd700', linestyle='None'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=6,
               facecolor='#1a1a2e', edgecolor='#444444',
               fontsize=8, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.06, 1, 0.96])
    out_path = os.path.join(args.out_dir, f'search_gmm_loss{args.loss_idx}.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f"\n保存: {out_path}")

    # ===== 时间线图：目标距离 + 搜索状态 =====
    fig2, axes2 = plt.subplots(2, 1, figsize=(12, 5), facecolor='#0d0d1a',
                                sharex=True)
    fig2.suptitle('失锁事件时间线', color='white', fontsize=12)

    # 子图1：目标与各无人机距离
    ax_dist = axes2[0]
    ax_dist.set_facecolor('#1a1a2e')
    tgt_arr = np.array(target_odom)
    t_mask = (tgt_arr[:, 0] >= t_loss_start - 2) & (tgt_arr[:, 0] <= t_plot_end + 1)
    tgt_t = tgt_arr[t_mask, 0]
    tgt_xy = tgt_arr[t_mask, 1:3]

    for d in range(3):
        d_arr = np.array(drone_odom[d])
        d_mask = (d_arr[:, 0] >= t_loss_start - 2) & (d_arr[:, 0] <= t_plot_end + 1)
        d_t = d_arr[d_mask, 0]
        d_xy = d_arr[d_mask, 1:3]
        # 插值到目标时间轴
        if len(d_t) > 1 and len(tgt_t) > 1:
            dx = np.interp(tgt_t, d_t, d_xy[:, 0])
            dy = np.interp(tgt_t, d_t, d_xy[:, 1])
            dist = np.sqrt((tgt_xy[:, 0] - dx)**2 + (tgt_xy[:, 1] - dy)**2)
            ax_dist.plot(tgt_t - t_loss_start, dist,
                         color=DRONE_COLORS[d], label=f'Drone {d}', lw=1.5)

    ax_dist.axvspan(0, loss_dur, alpha=0.15, color='red', label='失锁期间')
    ax_dist.axvline(0, color='red', lw=1.5, ls='--', alpha=0.8)
    ax_dist.axvline(loss_dur, color='lime', lw=1.5, ls='--', alpha=0.8)
    ax_dist.set_ylabel('目标距离 (m)', color='white', fontsize=9)
    ax_dist.tick_params(colors='gray')
    ax_dist.legend(facecolor='#1a1a2e', edgecolor='#444444', fontsize=8)
    ax_dist.set_facecolor('#1a1a2e')
    for spine in ax_dist.spines.values():
        spine.set_edgecolor('#444444')

    # 子图2：各无人机搜索状态
    ax_state = axes2[1]
    ax_state.set_facecolor('#1a1a2e')
    for d in range(3):
        ss = np.array(search_states[d])
        t_mask2 = (ss[:, 0] >= t_loss_start - 2) & (ss[:, 0] <= t_plot_end + 1)
        ss_t = ss[t_mask2, 0] - t_loss_start
        ss_v = ss[t_mask2, 1].astype(float)
        ax_state.step(ss_t, ss_v * (d + 1) * 0.3 + d * 0.05,
                      color=DRONE_COLORS[d], lw=2, label=f'Drone {d}', where='post')

    ax_state.axvspan(0, loss_dur, alpha=0.15, color='red')
    ax_state.axvline(0, color='red', lw=1.5, ls='--', alpha=0.8, label='失锁')
    ax_state.axvline(loss_dur, color='lime', lw=1.5, ls='--', alpha=0.8, label='重获')
    ax_state.set_xlabel(f'时间 (s, 相对失锁时刻 t={t_loss_start:.2f}s)', color='white', fontsize=9)
    ax_state.set_ylabel('搜索模式', color='white', fontsize=9)
    ax_state.set_yticks([])
    ax_state.tick_params(colors='gray')
    ax_state.legend(facecolor='#1a1a2e', edgecolor='#444444', fontsize=8)
    for spine in ax_state.spines.values():
        spine.set_edgecolor('#444444')

    plt.tight_layout()
    out_path2 = os.path.join(args.out_dir, f'search_timeline_loss{args.loss_idx}.png')
    plt.savefig(out_path2, dpi=150, bbox_inches='tight', facecolor=fig2.get_facecolor())
    plt.close()
    print(f"保存: {out_path2}")

    # ===== 轨迹总览图 =====
    fig3, ax3 = plt.subplots(figsize=(8, 8), facecolor='#0d0d1a')
    ax3.set_facecolor('#1a1a2e')
    ax3.set_aspect('equal')

    # 目标轨迹（按搜索/追踪着色）
    tgt_full = np.array(target_odom)
    t_range = (tgt_full[:, 0] >= t_loss_start - 2) & (tgt_full[:, 0] <= t_plot_end + 1)
    tgt_seg = tgt_full[t_range]
    for i in range(len(tgt_seg) - 1):
        t_mid = (tgt_seg[i, 0] + tgt_seg[i+1, 0]) / 2
        in_s = t_loss_start <= t_mid <= t_loss_end
        c = '#ff6b6b' if in_s else '#ffd700'
        ax3.plot(tgt_seg[i:i+2, 1], tgt_seg[i:i+2, 2], c=c, lw=2, alpha=0.8)

    # 无人机轨迹
    for d in range(3):
        d_arr = np.array(drone_odom[d])
        t_range_d = (d_arr[:, 0] >= t_loss_start - 2) & (d_arr[:, 0] <= t_plot_end + 1)
        d_seg = d_arr[t_range_d]
        if len(d_seg) > 1:
            ax3.plot(d_seg[:, 1], d_seg[:, 2], color=DRONE_COLORS[d],
                     lw=1.5, alpha=0.7, label=f'Drone {d}')
            ax3.plot(d_seg[-1, 1], d_seg[-1, 2], '^',
                     color=DRONE_COLORS[d], ms=10, zorder=5)

    # 标注失锁/重获时刻目标位置
    t_loss_entry = nearest(target_odom, t_loss_start)
    t_reacq_entry = nearest(target_odom, t_loss_end)
    if t_loss_entry:
        ax3.plot(t_loss_entry[1], t_loss_entry[2], 'o', ms=12,
                 color='red', zorder=7, label='失锁时刻')
    if t_reacq_entry:
        ax3.plot(t_reacq_entry[1], t_reacq_entry[2], 'o', ms=12,
                 color='lime', zorder=7, label='重获时刻')

    ax3.set_title(f'轨迹总览 — 第{args.loss_idx}次失锁 (持续{loss_dur:.2f}s)',
                  color='white', fontsize=11)
    ax3.set_xlabel('X (m)', color='white')
    ax3.set_ylabel('Y (m)', color='white')
    ax3.tick_params(colors='gray')
    ax3.legend(facecolor='#1a1a2e', edgecolor='#444444', fontsize=9)
    for spine in ax3.spines.values():
        spine.set_edgecolor('#444444')

    plt.tight_layout()
    out_path3 = os.path.join(args.out_dir, f'search_trajectory_loss{args.loss_idx}.png')
    plt.savefig(out_path3, dpi=150, bbox_inches='tight', facecolor=fig3.get_facecolor())
    plt.close()
    print(f"保存: {out_path3}")

    print(f"\n全部完成，输出目录: {args.out_dir}")


if __name__ == '__main__':
    main()
