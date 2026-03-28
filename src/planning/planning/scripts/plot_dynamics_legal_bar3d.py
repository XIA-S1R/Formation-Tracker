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
from matplotlib import cm
from matplotlib import colors as mcolors
from matplotlib import font_manager
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


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
        description='汇总 experiment_results 并绘制轨迹合法度 3D 柱状图'
    )
    parser.add_argument('--root', default='experiment_results', help='实验结果根目录')
    parser.add_argument('--formation-prefix', default='三角编队', help='目录前缀，例如 三角编队')
    parser.add_argument(
        '--include-tags',
        default='真实速度,Pro,Extra',
        help='参与统计的目录后缀，逗号分隔（默认: 真实速度,Pro,Extra）',
    )
    parser.add_argument(
        '--metric-key',
        default='dynamics_legal_ratio',
        help='统计指标键名（默认: dynamics_legal_ratio）',
    )
    parser.add_argument(
        '--scene-order',
        default='空地,密集障碍,摆脱机动',
        help='场景显示顺序，逗号分隔',
    )
    parser.add_argument(
        '--speed-values',
        default='',
        help='仅绘制指定速度，逗号分隔（如 2.5 或 1.5,2,2.5；默认自动从数据提取）',
    )
    parser.add_argument(
        '--out-png',
        default='',
        help='输出图片路径，默认 root/dynamics_legal_ratio_bar3d.png',
    )
    parser.add_argument(
        '--out-csv',
        default='',
        help='输出CSV路径，默认 root/dynamics_legal_ratio_bar3d.csv',
    )
    parser.add_argument('--dpi', type=int, default=280, help='输出DPI')
    parser.add_argument('--bar-width', type=float, default=0.62, help='柱体宽度')
    parser.add_argument('--bar-depth', type=float, default=0.62, help='柱体深度')
    parser.add_argument(
        '--title',
        default='',
        help='标题（默认不显示）',
    )
    return parser.parse_args()


def parse_list(raw):
    txt = str(raw).strip()
    if not txt:
        return []
    return [x.strip() for x in txt.split(',') if x.strip()]


def parse_float_list(raw):
    vals = []
    for t in parse_list(raw):
        vals.append(float(t))
    return vals


def parse_scene_and_tag(rest, include_tags):
    # 优先匹配更长后缀，避免歧义
    for tag in sorted(include_tags, key=len, reverse=True):
        if rest.endswith(tag):
            scene = rest[:-len(tag)]
            return scene, tag
    return rest, ''


def iter_metric_rows(summary_data, metric_key):
    if isinstance(summary_data, dict):
        rows = [summary_data]
    elif isinstance(summary_data, list):
        rows = summary_data
    else:
        rows = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        v = row.get(metric_key, None)
        if isinstance(v, (int, float)):
            yield float(v)


def load_grouped(root: Path, formation_prefix: str, include_tags, metric_key: str):
    pat = re.compile(r'^{}_逃速([0-9.]+)_(.+)$'.format(re.escape(formation_prefix)))
    grouped = defaultdict(lambda: {'values': [], 'folders': [], 'tags': defaultdict(int)})

    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        m = pat.match(folder.name)
        if not m:
            continue
        speed = float(m.group(1))
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

        vals = list(iter_metric_rows(data, metric_key))
        if not vals:
            continue

        key = (scene, speed)
        grouped[key]['values'].extend(vals)
        grouped[key]['folders'].append(str(folder))
        grouped[key]['tags'][tag if tag else '(none)'] += len(vals)

    return grouped


def choose_orders(grouped_keys, scene_order_hint, speed_values):
    scenes_raw = sorted({k[0] for k in grouped_keys})
    speeds_raw = sorted({k[1] for k in grouped_keys})

    scenes = [s for s in scene_order_hint if s in scenes_raw] + [s for s in scenes_raw if s not in scene_order_hint]
    if speed_values:
        speeds = [s for s in speed_values if s in speeds_raw]
    else:
        speeds = speeds_raw
    return scenes, speeds


def draw_bar3d(grouped, scenes, speeds, out_png: Path, dpi=280, bar_w=0.62, bar_d=0.62, title=''):
    fig = plt.figure(figsize=(10.6, 6.8), dpi=dpi)
    ax = fig.add_subplot(111, projection='3d')

    # 准备矩阵
    mat = np.full((len(scenes), len(speeds)), np.nan, dtype=float)
    cnt = np.zeros((len(scenes), len(speeds)), dtype=int)
    for i, sc in enumerate(scenes):
        for j, sp in enumerate(speeds):
            key = (sc, sp)
            if key not in grouped:
                continue
            vals = grouped[key]['values']
            if len(vals) == 0:
                continue
            mat[i, j] = float(np.mean(vals))
            cnt[i, j] = len(vals)

    valid_vals = mat[np.isfinite(mat)]
    if len(valid_vals) == 0:
        raise RuntimeError('没有可绘制的有效数据')

    norm = mcolors.Normalize(vmin=max(0.0, float(np.min(valid_vals)) - 0.02), vmax=min(1.0, float(np.max(valid_vals)) + 0.02))
    cmap = cm.get_cmap('YlGnBu')

    # 绘制柱体
    for i in range(len(scenes)):
        for j in range(len(speeds)):
            if not np.isfinite(mat[i, j]):
                continue
            x = float(i)
            y = float(j)
            h = float(mat[i, j])
            color = cmap(norm(h))
            ax.bar3d(
                x, y, 0.0,
                float(bar_w), float(bar_d), h,
                color=color,
                shade=True,
                edgecolor=(0.25, 0.25, 0.25, 0.35),
                linewidth=0.35,
            )
            ax.text(
                x + bar_w * 0.5,
                y + bar_d * 0.5,
                h + 0.015,
                '{:.1f}%'.format(h * 100.0),
                ha='center',
                va='bottom',
                fontsize=9,
            )

    ax.set_xticks(np.arange(len(scenes)) + bar_w * 0.5)
    ax.set_xticklabels(scenes, fontsize=10)
    ax.set_yticks(np.arange(len(speeds)) + bar_d * 0.5)
    ax.set_yticklabels([('{:g}'.format(s)) for s in speeds], fontsize=10)
    ax.set_zlabel('轨迹合法度比率', fontsize=11, labelpad=8)
    ax.set_xlabel('任务场景', fontsize=11, labelpad=10)
    ax.set_ylabel(r'目标速度 $/m \cdot s^{-1}$', fontsize=11, labelpad=12)
    ax.set_zlim(0.0, 1.02)
    ax.view_init(elev=23, azim=-56)

    if title:
        ax.set_title(title, fontsize=13, pad=10)

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array(valid_vals)
    cbar = fig.colorbar(sm, ax=ax, fraction=0.035, pad=0.08)
    cbar.set_label('比率', fontsize=10)

    fig.tight_layout()
    fig.savefig(str(out_png), dpi=dpi, bbox_inches='tight')
    plt.close(fig)

    return mat, cnt


def write_csv(grouped, scenes, speeds, out_csv: Path):
    lines = ['场景,速度(m/s),轨迹合法度均值,样本数,来源标签统计,来源目录']
    for sc in scenes:
        for sp in speeds:
            key = (sc, sp)
            if key not in grouped:
                continue
            vals = grouped[key]['values']
            if not vals:
                continue
            mean_v = float(np.mean(vals))
            tags = grouped[key]['tags']
            tags_text = '|'.join(['{}:{}'.format(k, v) for k, v in sorted(tags.items())])
            folders = '|'.join(grouped[key]['folders'])
            line = [
                sc,
                '{:g}'.format(sp),
                '{:.6f}'.format(mean_v),
                str(len(vals)),
                '"{}"'.format(tags_text),
                '"{}"'.format(folders),
            ]
            lines.append(','.join(line))
    out_csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    configure_chinese_font()

    root = Path(args.root).resolve()
    if not root.exists():
        raise FileNotFoundError(str(root))

    include_tags = parse_list(args.include_tags)
    scene_order_hint = parse_list(args.scene_order)
    speed_values = parse_float_list(args.speed_values) if args.speed_values.strip() else []

    grouped = load_grouped(
        root=root,
        formation_prefix=args.formation_prefix,
        include_tags=include_tags,
        metric_key=args.metric_key,
    )
    if not grouped:
        raise RuntimeError('未读取到可用数据，请检查 --root/--formation-prefix/--include-tags')

    scenes, speeds = choose_orders(grouped.keys(), scene_order_hint, speed_values)
    if not scenes or not speeds:
        raise RuntimeError('筛选后无可绘制的场景或速度')

    out_png = Path(args.out_png).resolve() if args.out_png else (root / 'dynamics_legal_ratio_bar3d.png')
    out_csv = Path(args.out_csv).resolve() if args.out_csv else (root / 'dynamics_legal_ratio_bar3d.csv')

    mat, cnt = draw_bar3d(
        grouped=grouped,
        scenes=scenes,
        speeds=speeds,
        out_png=out_png,
        dpi=int(args.dpi),
        bar_w=float(args.bar_width),
        bar_d=float(args.bar_depth),
        title=str(args.title),
    )
    write_csv(grouped, scenes, speeds, out_csv)

    print('图像: {}'.format(out_png))
    print('表格: {}'.format(out_csv))
    print('场景: {}'.format(', '.join(scenes)))
    print('速度: {}'.format(', '.join(['{:g}'.format(s) for s in speeds])))
    valid_n = int(np.sum(np.isfinite(mat)))
    print('有效柱体数: {}'.format(valid_n))


if __name__ == '__main__':
    main()
