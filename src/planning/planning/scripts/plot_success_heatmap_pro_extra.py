#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

import copy
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
        description='汇总 Pro+Extra 实验并绘制任务成功率热力图（中文）'
    )
    parser.add_argument(
        '--root',
        default='experiment_results',
        help='实验结果根目录（默认: experiment_results）',
    )
    parser.add_argument(
        '--formation-prefix',
        default='三角编队',
        help='目录名前缀（默认: 三角编队）',
    )
    parser.add_argument(
        '--out-png',
        default='',
        help='输出热力图路径，默认写到 root/success_rate_heatmap_pro_extra_cn.png',
    )
    parser.add_argument(
        '--out-csv',
        default='',
        help='输出统计 CSV 路径，默认写到 root/success_rate_heatmap_pro_extra_cn.csv',
    )
    parser.add_argument(
        '--dpi',
        type=int,
        default=260,
        help='输出图片 DPI（默认: 260）',
    )
    return parser.parse_args()


def load_records(root: Path, formation_prefix: str):
    # 例: 三角编队_逃速2.5_障碍Pro / 三角编队_逃速2.5_障碍Extra
    pattern = re.compile(
        r'^{}_逃速([0-9.]+)_(.+?)(Pro|Extra)$'.format(re.escape(formation_prefix))
    )

    grouped = defaultdict(
        lambda: {
            'success': 0,
            'total': 0,
            'success_pro': 0,
            'total_pro': 0,
            'success_extra': 0,
            'total_extra': 0,
            'folders': [],
        }
    )

    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        m = pattern.match(folder.name)
        if not m:
            continue

        speed = float(m.group(1))
        scene = m.group(2)
        tag = m.group(3)  # Pro / Extra
        summary_path = folder / 'summary.json'
        if not summary_path.exists():
            continue

        try:
            data = json.loads(summary_path.read_text(encoding='utf-8'))
        except Exception:
            continue
        if not isinstance(data, list):
            continue

        success = 0
        total = 0
        for row in data:
            if not isinstance(row, dict) or 'task_success' not in row:
                continue
            total += 1
            success += 1 if bool(row.get('task_success')) else 0

        if total == 0:
            continue

        key = (scene, speed)
        g = grouped[key]
        g['success'] += success
        g['total'] += total
        if tag == 'Pro':
            g['success_pro'] += success
            g['total_pro'] += total
        else:
            g['success_extra'] += success
            g['total_extra'] += total
        g['folders'].append(str(folder))

    return grouped


def draw_heatmap(grouped, out_png: Path):
    if not grouped:
        raise RuntimeError('没有匹配到可用的 Pro/Extra summary.json 数据')

    scene_pref = ['空地', '障碍', '摆脱机动']
    scenes_raw = sorted({k[0] for k in grouped.keys()})
    scenes = [s for s in scene_pref if s in scenes_raw] + [s for s in scenes_raw if s not in scene_pref]
    speeds = sorted({k[1] for k in grouped.keys()})

    mat = np.full((len(scenes), len(speeds)), np.nan, dtype=float)
    ann = [['无数据' for _ in speeds] for __ in scenes]

    for (scene, speed), v in grouped.items():
        i = scenes.index(scene)
        j = speeds.index(speed)
        rate = v['success'] / v['total'] if v['total'] > 0 else np.nan
        mat[i, j] = rate
        ann[i][j] = '{:.1f}%'.format(rate * 100.0)

    cmap = copy.copy(plt.get_cmap('YlGnBu'))
    cmap.set_bad(color='#eeeeee')

    fig, ax = plt.subplots(figsize=(8.8, 5.2), dpi=260)
    im = ax.imshow(mat, cmap=cmap, vmin=0.0, vmax=1.0, aspect='auto')

    ax.set_xticks(np.arange(len(speeds)))
    ax.set_xticklabels(['{}'.format(s).rstrip('0').rstrip('.') if isinstance(s, float) else str(s) for s in speeds], fontsize=11)
    ax.set_xlabel(r'目标速度 $\mathrm{/m \cdot s^{-1}}$', fontsize=12)

    ax.set_yticks(np.arange(len(scenes)))
    ax.set_yticklabels(scenes, fontsize=11)
    ax.set_ylabel('', fontsize=12)

    for i in range(len(scenes)):
        for j in range(len(speeds)):
            val = mat[i, j]
            text = ann[i][j]
            if np.isnan(val):
                color = '#555555'
            else:
                color = 'white' if val >= 0.55 else 'black'
            ax.text(j, i, text, ha='center', va='center', fontsize=10, color=color)

    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.tight_layout()
    fig.savefig(str(out_png), bbox_inches='tight')
    plt.close(fig)


def write_csv(grouped, out_csv: Path):
    rows = []
    for (scene, speed), v in sorted(grouped.items(), key=lambda x: (x[0][0], x[0][1])):
        total = v['total']
        success = v['success']
        rate = (success / total) if total > 0 else 0.0
        rows.append(
            {
                '场景': scene,
                '速度(m/s)': speed,
                '成功率': rate,
                '成功次数': success,
                '总次数': total,
                'Pro成功次数': v['success_pro'],
                'Pro总次数': v['total_pro'],
                'Extra成功次数': v['success_extra'],
                'Extra总次数': v['total_extra'],
                '来源目录': '|'.join(v['folders']),
            }
        )

    header = [
        '场景',
        '速度(m/s)',
        '成功率',
        '成功次数',
        '总次数',
        'Pro成功次数',
        'Pro总次数',
        'Extra成功次数',
        'Extra总次数',
        '来源目录',
    ]
    lines = [','.join(header)]
    for r in rows:
        line = [
            str(r['场景']),
            ('{}'.format(r['速度(m/s)']).rstrip('0').rstrip('.') if isinstance(r['速度(m/s)'], float) else str(r['速度(m/s)'])),
            '{:.6f}'.format(r['成功率']),
            str(r['成功次数']),
            str(r['总次数']),
            str(r['Pro成功次数']),
            str(r['Pro总次数']),
            str(r['Extra成功次数']),
            str(r['Extra总次数']),
            '"{}"'.format(r['来源目录']),
        ]
        lines.append(','.join(line))
    out_csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    configure_chinese_font()

    root = Path(args.root).resolve()
    if not root.exists():
        raise FileNotFoundError(str(root))

    out_png = Path(args.out_png).resolve() if args.out_png else (root / 'success_rate_heatmap_pro_extra_cn.png')
    out_csv = Path(args.out_csv).resolve() if args.out_csv else (root / 'success_rate_heatmap_pro_extra_cn.csv')

    grouped = load_records(root, args.formation_prefix)
    # User-specified manual correction:
    # 2.5m/s + 摆脱机动 => success rate = 18/27
    if ('摆脱机动', 2.5) in grouped:
        grouped[('摆脱机动', 2.5)]['success'] = 18
        grouped[('摆脱机动', 2.5)]['total'] = 27
    draw_heatmap(grouped, out_png)
    write_csv(grouped, out_csv)

    print('热力图: {}'.format(out_png))
    print('统计表: {}'.format(out_csv))
    print('组合数: {}'.format(len(grouped)))


if __name__ == '__main__':
    main()
