#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

try:
    from scipy.stats import gaussian_kde
except Exception:
    gaussian_kde = None


def configure_chinese_font():
    candidates = [
        'Noto Sans CJK SC',
        'Noto Sans CJK JP',
        'Noto Serif CJK JP',
        'Source Han Sans CN',
        'Microsoft YaHei',
        'SimHei',
        'WenQuanYi Zen Hei',
        'Arial Unicode MS',
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            matplotlib.rcParams['font.sans-serif'] = [name]
            break
    matplotlib.rcParams['axes.unicode_minus'] = False
    matplotlib.rcParams['mathtext.fontset'] = 'stix'


def parse_args():
    parser = argparse.ArgumentParser(
        description='绘制轨迹安全性统计图：机间最小距离分布概率曲线（按场景）'
    )
    parser.add_argument('--root', default='experiment_results', help='实验结果根目录')
    parser.add_argument('--formation-prefix', default='三角编队', help='目录前缀，例如 三角编队')
    parser.add_argument('--speed', type=float, default=2.5, help='目标速度（默认 2.5 m/s）')
    parser.add_argument(
        '--include-tags',
        default='真实速度,Pro,Extra,_避撞,避撞',
        help='参与统计的目录后缀，逗号分隔（默认: 真实速度,Pro,Extra）',
    )
    parser.add_argument(
        '--scene-order',
        default='空地,障碍,摆脱机动',
        help='场景显示顺序，逗号分隔',
    )
    parser.add_argument('--bins', type=int, default=24, help='无KDE时的直方图分箱数')
    parser.add_argument('--kde-points', type=int, default=300, help='KDE采样点数')
    parser.add_argument(
        '--title',
        default='轨迹安全性统计图',
        help='图标题',
    )
    parser.add_argument('--out-png', default='', help='输出PNG路径')
    parser.add_argument('--out-csv', default='', help='输出CSV路径（统计表）')
    parser.add_argument('--dpi', type=int, default=300, help='输出DPI')
    parser.add_argument('--x-min', type=float, default=np.nan, help='横轴最小值（可选）')
    parser.add_argument('--x-max', type=float, default=np.nan, help='横轴最大值（可选）')
    parser.add_argument(
        '--tail-zero',
        action='store_true',
        default=True,
        help='将概率密度曲线两端平滑收敛到0（默认开启）',
    )
    parser.add_argument(
        '--no-tail-zero',
        dest='tail_zero',
        action='store_false',
        help='关闭两端收敛到0',
    )
    parser.add_argument(
        '--tail-taper-ratio',
        type=float,
        default=0.06,
        help='两端收敛带宽占比（默认6%%）',
    )
    parser.add_argument(
        '--min-valid-distance',
        type=float,
        default=0.0,
        help='剔除小于该值的机间最小距离样本（默认不过滤）',
    )
    return parser.parse_args()


def parse_list(raw):
    txt = str(raw).strip()
    if not txt:
        return []
    return [x.strip() for x in txt.split(',') if x.strip()]


def parse_scene_and_tag(rest, include_tags):
    for tag in sorted(include_tags, key=len, reverse=True):
        if rest.endswith(tag):
            scene = rest[:-len(tag)]
            scene = scene.rstrip('_- ')
            return scene, tag
    return rest, ''


def iter_rows(summary_data):
    if isinstance(summary_data, dict):
        return [summary_data]
    if isinstance(summary_data, list):
        return summary_data
    return []


def load_distance_values(root: Path, formation_prefix: str, speed: float, include_tags, min_valid_distance: float):
    # e.g., 三角编队_逃速2.5_障碍Pro
    pat = re.compile(r'^{}_逃速([0-9.]+)_(.+)$'.format(re.escape(formation_prefix)))
    grouped = defaultdict(lambda: {'values': [], 'folders': [], 'tag_counts': defaultdict(int)})

    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        m = pat.match(folder.name)
        if not m:
            continue
        sp = float(m.group(1))
        if abs(sp - float(speed)) > 1e-6:
            continue
        rest = m.group(2)
        scene, tag = parse_scene_and_tag(rest, include_tags)
        if include_tags and tag not in include_tags:
            continue

        summary_path = folder / 'summary.json'
        if not summary_path.exists():
            continue

        try:
            data = json.loads(summary_path.read_text(encoding='utf-8'))
        except Exception:
            continue

        rows = iter_rows(data)
        vals = []
        for row in rows:
            if not isinstance(row, dict):
                continue
            v = row.get('min_inter_drone_distance', None)
            if isinstance(v, (int, float)) and np.isfinite(v):
                vv = float(v)
                if vv < float(min_valid_distance):
                    continue
                vals.append(vv)
        if not vals:
            continue

        grouped[scene]['values'].extend(vals)
        grouped[scene]['folders'].append(str(folder))
        grouped[scene]['tag_counts'][tag if tag else '(none)'] += len(vals)

    return grouped


def smooth_hist_density(values, x_grid, bins):
    hist, edges = np.histogram(values, bins=int(max(6, bins)), range=(x_grid[0], x_grid[-1]), density=True)
    centers = 0.5 * (edges[:-1] + edges[1:])
    # 轻度平滑
    kernel = np.array([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float64)
    kernel /= np.sum(kernel)
    y = np.convolve(hist, kernel, mode='same')
    y_interp = np.interp(x_grid, centers, y, left=0.0, right=0.0)
    return y_interp


def apply_tail_zero_taper(y, ratio=0.06):
    yy = np.asarray(y, dtype=np.float64).copy()
    n = len(yy)
    if n < 6:
        if n >= 1:
            yy[0] = 0.0
            yy[-1] = 0.0
        return yy
    k = int(max(3, round(float(ratio) * n)))
    k = min(k, n // 2)
    if k >= 2:
        left = np.linspace(0.0, 1.0, k)
        right = np.linspace(1.0, 0.0, k)
        yy[:k] *= left
        yy[-k:] *= right
    yy[0] = 0.0
    yy[-1] = 0.0
    yy = np.maximum(yy, 0.0)
    return yy


def draw_curves(grouped, scene_order, speed, out_png, bins=24, kde_points=300, title='轨迹安全性统计图', dpi=300, x_min=np.nan, x_max=np.nan, tail_zero=True, tail_taper_ratio=0.06):
    if not grouped:
        raise RuntimeError('没有可用数据，请检查目录、速度或标签过滤条件')

    scenes_raw = list(grouped.keys())
    scenes = [s for s in scene_order if s in scenes_raw] + [s for s in scenes_raw if s not in scene_order]

    all_vals = np.concatenate([np.asarray(grouped[s]['values'], dtype=np.float64) for s in scenes if len(grouped[s]['values']) > 0])
    if len(all_vals) == 0:
        raise RuntimeError('未提取到 min_inter_drone_distance 数据')

    xmin_data = float(np.min(all_vals))
    xmax_data = float(np.max(all_vals))
    span = float(xmax_data - xmin_data)
    std = float(np.std(all_vals))
    # 给尾部留出更多空间，避免曲线在边界被截断后看起来“不收敛”
    xpad = max(0.12, 0.25 * span, 0.8 * std)
    xmin = (max(0.0, xmin_data - xpad)) if np.isnan(x_min) else float(x_min)
    xmax = (xmax_data + xpad) if np.isnan(x_max) else float(x_max)
    if xmax <= xmin:
        xmax = xmin + 1.0

    x_grid = np.linspace(xmin, xmax, int(max(120, kde_points)))

    colors = {
        '空地': '#1f77b4',
        '障碍': '#d62728',
        '摆脱机动': '#2ca02c',
    }
    fallback = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#8c564b']

    fig, ax = plt.subplots(figsize=(9.2, 5.6), dpi=dpi)
    for i, scene in enumerate(scenes):
        vals = np.asarray(grouped[scene]['values'], dtype=np.float64)
        if len(vals) == 0:
            continue
        c = colors.get(scene, fallback[i % len(fallback)])

        if gaussian_kde is not None and len(vals) >= 3 and np.std(vals) > 1e-8:
            try:
                kde = gaussian_kde(vals)
                y = kde(x_grid)
            except Exception:
                y = smooth_hist_density(vals, x_grid, bins=bins)
        else:
            y = smooth_hist_density(vals, x_grid, bins=bins)

        y = np.maximum(y, 0.0)
        if tail_zero:
            y = apply_tail_zero_taper(y, ratio=float(tail_taper_ratio))
        ax.plot(x_grid, y, color=c, linewidth=2.2, label='{} (n={})'.format(scene, len(vals)))
        ax.fill_between(x_grid, 0.0, y, color=c, alpha=0.14)

    ax.set_xlabel('机间最小距离 (m)', fontsize=11)
    ax.set_ylabel('概率密度', fontsize=11)
    ax.set_title('{}（{} m/s）'.format(title, ('{:g}'.format(speed))), fontsize=13)
    ax.grid(True, linestyle='--', linewidth=0.6, alpha=0.45)
    ax.legend(loc='upper right', framealpha=0.92)
    fig.tight_layout()
    fig.savefig(str(out_png), dpi=dpi, bbox_inches='tight')
    plt.close(fig)


def write_csv(grouped, out_csv):
    lines = ['场景,样本数,均值,标准差,最小值,中位数,95分位,最大值,来源标签统计,来源目录']
    for scene in sorted(grouped.keys()):
        vals = np.asarray(grouped[scene]['values'], dtype=np.float64)
        if len(vals) == 0:
            continue
        tags_text = '|'.join(['{}:{}'.format(k, v) for k, v in sorted(grouped[scene]['tag_counts'].items())])
        folders = '|'.join(grouped[scene]['folders'])
        row = [
            scene,
            str(len(vals)),
            '{:.6f}'.format(float(np.mean(vals))),
            '{:.6f}'.format(float(np.std(vals))),
            '{:.6f}'.format(float(np.min(vals))),
            '{:.6f}'.format(float(np.median(vals))),
            '{:.6f}'.format(float(np.percentile(vals, 95))),
            '{:.6f}'.format(float(np.max(vals))),
            '"{}"'.format(tags_text),
            '"{}"'.format(folders),
        ]
        lines.append(','.join(row))
    out_csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    configure_chinese_font()

    root = Path(args.root).resolve()
    if not root.exists():
        raise FileNotFoundError(str(root))

    include_tags = parse_list(args.include_tags)
    scene_order = parse_list(args.scene_order)

    grouped = load_distance_values(
        root=root,
        formation_prefix=str(args.formation_prefix),
        speed=float(args.speed),
        include_tags=include_tags,
        min_valid_distance=float(args.min_valid_distance),
    )

    out_png = Path(args.out_png).resolve() if args.out_png else (root / '轨迹安全性统计图.png')
    out_csv = Path(args.out_csv).resolve() if args.out_csv else (root / '轨迹安全性统计图.csv')

    draw_curves(
        grouped=grouped,
        scene_order=scene_order,
        speed=float(args.speed),
        out_png=out_png,
        bins=int(args.bins),
        kde_points=int(args.kde_points),
        title=str(args.title),
        dpi=int(args.dpi),
        x_min=float(args.x_min),
        x_max=float(args.x_max),
        tail_zero=bool(args.tail_zero),
        tail_taper_ratio=max(0.0, float(args.tail_taper_ratio)),
    )
    write_csv(grouped, out_csv)

    print('图像: {}'.format(out_png))
    print('表格: {}'.format(out_csv))
    print('场景: {}'.format(', '.join(sorted(grouped.keys()))))
    print('速度: {:g}'.format(float(args.speed)))


if __name__ == '__main__':
    main()
