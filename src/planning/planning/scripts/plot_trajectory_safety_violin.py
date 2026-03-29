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
        description='轨迹安全性统计图（小提琴图）：按场景合并样本'
    )
    parser.add_argument('--root', default='experiment_results', help='实验结果根目录')
    parser.add_argument('--formation-prefix', default='三角编队', help='目录前缀')
    parser.add_argument('--speed', type=float, default=2.5, help='目标速度')
    parser.add_argument(
        '--include-tags',
        default='_避撞,_避撞2',
        help='参与统计的目录后缀（逗号分隔）',
    )
    parser.add_argument(
        '--scene-order',
        default='空地,障碍,摆脱机动',
        help='场景显示顺序',
    )
    parser.add_argument(
        '--min-valid-distance',
        type=float,
        default=0.0,
        help='剔除小于该值的样本',
    )
    parser.add_argument(
        '--title',
        default='轨迹安全性统计图',
        help='图标题',
    )
    parser.add_argument('--out-png', default='', help='输出PNG路径')
    parser.add_argument('--out-csv', default='', help='输出CSV路径')
    parser.add_argument('--dpi', type=int, default=320, help='输出DPI')
    parser.add_argument('--point-alpha', type=float, default=0.85, help='异常值散点透明度')
    parser.add_argument(
        '--example-style',
        action='store_true',
        default=True,
        help='采用接近示例图的灰底+箱线内嵌风格（默认开启）',
    )
    parser.add_argument(
        '--no-example-style',
        dest='example_style',
        action='store_false',
        help='关闭示例风格',
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
            scene = rest[:-len(tag)].rstrip('_- ')
            return scene, tag
    return rest, ''


def iter_rows(summary_data):
    if isinstance(summary_data, dict):
        return [summary_data]
    if isinstance(summary_data, list):
        return summary_data
    return []


def load_values(root: Path, formation_prefix: str, speed: float, include_tags, min_valid_distance: float):
    pat = re.compile(r'^{}_逃速([0-9.]+)_(.+)$'.format(re.escape(formation_prefix)))
    grouped = defaultdict(lambda: {'values': [], 'folders': [], 'tags': defaultdict(int)})

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

        vals = []
        for row in iter_rows(data):
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
        grouped[scene]['tags'][tag if tag else '(none)'] += len(vals)

    return grouped


def draw_violin(
    grouped,
    scene_order,
    out_png: Path,
    dpi=320,
    title='轨迹安全性统计图',
    speed=2.5,
    point_alpha=0.85,
    example_style=True,
):
    if not grouped:
        raise RuntimeError('没有可绘制数据')

    scenes_raw = list(grouped.keys())
    scenes = [s for s in scene_order if s in scenes_raw] + [s for s in scenes_raw if s not in scene_order]
    data = [np.asarray(grouped[s]['values'], dtype=np.float64) for s in scenes]
    if len(data) == 0 or all(len(x) == 0 for x in data):
        raise RuntimeError('样本为空')

    fig, ax = plt.subplots(figsize=(8.8, 6.2), dpi=dpi)
    pos = np.arange(1, len(scenes) + 1, dtype=float)

    if example_style:
        # 接近示例图的灰底样式
        fig.patch.set_facecolor('#efefef')
        ax.set_facecolor('#e6e6e6')

    vp = ax.violinplot(
        dataset=data,
        positions=pos,
        widths=0.72,
        showmeans=False,
        showmedians=False,
        showextrema=False,
    )
    colors = ['#e96b56', '#4ec0dd', '#2fb3a2', '#586f9f', '#9a7fd1']
    for i, body in enumerate(vp['bodies']):
        body.set_facecolor(colors[i % len(colors)])
        body.set_edgecolor('#3d3d3d')
        body.set_alpha(0.96)
        body.set_linewidth(1.0)

    # 内嵌箱线图（示例图风格）
    bp = ax.boxplot(
        data,
        positions=pos,
        widths=0.17,
        patch_artist=True,
        showfliers=False,
        whis=1.5,
        zorder=4,
    )
    for box in bp['boxes']:
        box.set(facecolor='white', edgecolor='#333333', linewidth=1.1)
    for med in bp['medians']:
        med.set(color='#2f2f2f', linewidth=1.8)
    for whisk in bp['whiskers']:
        whisk.set(color='#333333', linewidth=1.05)
    for cap in bp['caps']:
        cap.set(color='#333333', linewidth=1.05)

    # 仅绘制异常值（1.5*IQR）
    rng = np.random.default_rng(11)
    for i, vals in enumerate(data):
        if len(vals) == 0:
            continue
        q1, q3 = np.percentile(vals, [25, 75])
        iqr = q3 - q1
        low = q1 - 1.5 * iqr
        high = q3 + 1.5 * iqr
        outliers = vals[(vals < low) | (vals > high)]
        if len(outliers) > 0:
            x = pos[i] + rng.normal(0.0, 0.012, size=len(outliers))
            ax.scatter(
                x,
                outliers,
                s=20,
                c='#3a3a3a',
                alpha=float(point_alpha),
                linewidths=0.0,
                zorder=6,
            )

    ax.set_xticks(pos)
    ax.set_xticklabels(scenes, fontsize=11)
    ax.set_xlabel('任务场景', fontsize=11)
    ax.set_ylabel('机间最小距离 (m)', fontsize=11)
    ax.set_title('{}（{} m/s）'.format(title, ('{:g}'.format(float(speed)))), fontsize=13)
    ax.grid(axis='y', linestyle='-', linewidth=0.7, alpha=0.55)

    all_vals = np.concatenate([v for v in data if len(v) > 0])
    ymin = min(0.0, float(np.min(all_vals)) - 0.08)
    ymax = float(np.max(all_vals)) + 0.12
    if ymax <= ymin:
        ymax = ymin + 1.0
    ax.set_ylim(ymin, ymax)

    fig.tight_layout()
    fig.savefig(str(out_png), dpi=dpi, bbox_inches='tight')
    plt.close(fig)


def write_csv(grouped, out_csv: Path):
    lines = ['场景,样本数,均值,标准差,最小值,中位数,95分位,最大值,来源标签统计,来源目录']
    for scene in sorted(grouped.keys()):
        vals = np.asarray(grouped[scene]['values'], dtype=np.float64)
        if len(vals) == 0:
            continue
        tags_text = '|'.join(['{}:{}'.format(k, v) for k, v in sorted(grouped[scene]['tags'].items())])
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

    grouped = load_values(
        root=root,
        formation_prefix=str(args.formation_prefix),
        speed=float(args.speed),
        include_tags=include_tags,
        min_valid_distance=float(args.min_valid_distance),
    )

    out_png = Path(args.out_png).resolve() if args.out_png else (root / '轨迹安全性统计图_小提琴图.png')
    out_csv = Path(args.out_csv).resolve() if args.out_csv else (root / '轨迹安全性统计图_小提琴图.csv')

    draw_violin(
        grouped=grouped,
        scene_order=scene_order,
        out_png=out_png,
        dpi=int(args.dpi),
        title=str(args.title),
        speed=float(args.speed),
        point_alpha=float(args.point_alpha),
        example_style=bool(args.example_style),
    )
    write_csv(grouped, out_csv)

    print('图像: {}'.format(out_png))
    print('表格: {}'.format(out_csv))
    print('场景: {}'.format(', '.join(sorted(grouped.keys()))))
    print('速度: {:g}'.format(float(args.speed)))


if __name__ == '__main__':
    main()
