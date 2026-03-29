#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
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


def parse_speed_list(raw):
    txt = str(raw).strip()
    if not txt:
        return []
    vals = []
    for t in txt.split(','):
        t = t.strip()
        if not t:
            continue
        vals.append(float(t))
    return vals


def parse_drop_rules(raw):
    """
    Format:
      --drop-rules "2:11,15;2.5:9"
    meaning:
      speed=2.0 drop bins {11,15}; speed=2.5 drop bin {9}
    """
    rules = defaultdict(set)
    txt = str(raw).strip()
    if not txt:
        return rules
    for blk in txt.split(';'):
        blk = blk.strip()
        if not blk or ':' not in blk:
            continue
        sp_txt, vals_txt = blk.split(':', 1)
        try:
            sp = float(sp_txt.strip())
        except Exception:
            continue
        for t in vals_txt.split(','):
            t = t.strip()
            if not t:
                continue
            try:
                rules[sp].add(int(float(t)))
            except Exception:
                continue
    return rules


def apply_drop_rules(speed_values, drop_rules):
    logs = []
    for sp, drops in drop_rules.items():
        if sp not in speed_values:
            continue
        before = len(speed_values[sp])
        speed_values[sp] = [v for v in speed_values[sp] if v not in drops]
        after = len(speed_values[sp])
        logs.append('drop speed {} bins {}: {} -> {}'.format('{:g}'.format(sp), sorted(drops), before, after))
    return logs


def load_emergency_stop_data(root: Path, formation_kw: str, scenario_kw: str, speed_filter):
    pat_speed = re.compile(r'逃速([0-9]+(?:\.[0-9]+)?)')
    speed_values = defaultdict(list)  # speed -> [emergency_stop_count, ...]
    speed_folders = defaultdict(set)  # speed -> {folder, ...}

    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        name = folder.name
        if formation_kw not in name or scenario_kw not in name:
            continue

        m = pat_speed.search(name)
        if not m:
            continue
        speed = float(m.group(1))
        if speed_filter and speed not in speed_filter:
            continue

        summary_path = folder / 'summary.json'
        if not summary_path.exists():
            continue

        try:
            data = json.loads(summary_path.read_text(encoding='utf-8'))
        except Exception:
            continue

        rows = data if isinstance(data, list) else ([data] if isinstance(data, dict) else [])
        for row in rows:
            if not isinstance(row, dict):
                continue
            val = row.get('emergency_stop_count', None)
            if not isinstance(val, (int, float)):
                continue
            cnt = int(round(float(val)))
            if cnt < 0:
                continue
            speed_values[speed].append(cnt)
            speed_folders[speed].add(str(folder))

    return speed_values, speed_folders


def save_csv(out_csv: Path, x_values, speed_values, speed_folders):
    lines = ['速度(m/s),急停次数,概率,该速度样本总数,该急停次数样本数,来源目录数']
    for speed in sorted(speed_values.keys()):
        vals = speed_values[speed]
        total = len(vals)
        hist = Counter(vals)
        for x in x_values:
            c = int(hist.get(x, 0))
            p = (c / total) if total > 0 else 0.0
            lines.append(
                '{},{},{:.8f},{},{},{}'.format(
                    ('{:g}'.format(speed)),
                    x,
                    p,
                    total,
                    c,
                    len(speed_folders.get(speed, [])),
                )
            )
    out_csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def plot_curves(out_png: Path, speed_values, title, dpi):
    if not speed_values:
        raise RuntimeError('未找到可用数据。')

    max_x = 0
    for vals in speed_values.values():
        if vals:
            max_x = max(max_x, max(vals))
    x_values = list(range(0, max_x + 1))

    fig, ax = plt.subplots(figsize=(9.2, 5.2), dpi=dpi)
    markers = ['o', 's', '^', 'D', 'v', 'P', 'X', '*']

    for i, speed in enumerate(sorted(speed_values.keys())):
        vals = speed_values[speed]
        if not vals:
            continue
        hist = Counter(vals)
        total = len(vals)
        ys = [(hist.get(x, 0) / total) for x in x_values]
        ax.plot(
            x_values,
            ys,
            marker=markers[i % len(markers)],
            linewidth=2.0,
            markersize=4.5,
            color='#1a1a1a',
            markerfacecolor='white',
            markeredgewidth=1.2,
            label='{} m/s'.format('{:g}'.format(speed)),
        )

    ax.set_xlabel('急停次数', fontsize=12)
    ax.set_ylabel('概率', fontsize=12)
    ax.set_xticks(x_values)
    ax.set_ylim(0.0, 1.0)
    ax.grid(True, linestyle='--', linewidth=0.6, alpha=0.45)
    ax.legend(title='目标速度', fontsize=10, title_fontsize=10, framealpha=0.9)
    if title:
        ax.set_title(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(str(out_png), dpi=dpi, bbox_inches='tight')
    plt.close(fig)

    return x_values


def main():
    parser = argparse.ArgumentParser(
        description='绘制三角编队摆脱机动实验的“急停次数-概率”曲线图（多速度对比）'
    )
    parser.add_argument('--root', default='experiment_results', help='实验结果根目录')
    parser.add_argument('--formation-keyword', default='三角编队', help='编队关键字')
    parser.add_argument('--scenario-keyword', default='摆脱机动', help='场景关键字')
    parser.add_argument(
        '--speeds',
        default='1.5,2,2.5',
        help='要绘制的速度列表（逗号分隔），默认: 1.5,2,2.5',
    )
    parser.add_argument('--out-png', default='', help='输出图片路径')
    parser.add_argument('--out-csv', default='', help='输出CSV路径')
    parser.add_argument('--dpi', type=int, default=280, help='图片DPI')
    parser.add_argument('--title', default='轨迹安全性统计图', help='图标题')
    parser.add_argument(
        '--drop-rules',
        default='',
        help='剔除规则，例如 "2:11,15"（速度:要剔除的次数列表）',
    )
    args = parser.parse_args()

    configure_chinese_font()

    root = Path(args.root).resolve()
    if not root.exists():
        raise FileNotFoundError(str(root))

    speed_filter = parse_speed_list(args.speeds)
    speed_values, speed_folders = load_emergency_stop_data(
        root=root,
        formation_kw=args.formation_keyword,
        scenario_kw=args.scenario_keyword,
        speed_filter=speed_filter,
    )

    # 只保留请求速度中确实有数据的速度
    if speed_filter:
        speed_values = {s: speed_values.get(s, []) for s in speed_filter if len(speed_values.get(s, [])) > 0}
        speed_folders = {s: speed_folders.get(s, set()) for s in speed_values.keys()}

    if not speed_values:
        raise RuntimeError('未在指定目录中找到符合条件的数据。')

    drop_rules = parse_drop_rules(args.drop_rules)
    if drop_rules:
        logs = apply_drop_rules(speed_values, drop_rules)
        for ln in logs:
            print('[rule] {}'.format(ln))
        speed_values = {k: v for k, v in speed_values.items() if len(v) > 0}
        speed_folders = {k: speed_folders.get(k, set()) for k in speed_values.keys()}
        if not speed_values:
            raise RuntimeError('应用剔除规则后无有效数据。')

    out_png = Path(args.out_png).resolve() if args.out_png else (root / '急停次数概率曲线图.png')
    out_csv = Path(args.out_csv).resolve() if args.out_csv else (root / '急停次数概率曲线图.csv')

    x_values = plot_curves(out_png, speed_values, args.title, args.dpi)
    save_csv(out_csv, x_values, speed_values, speed_folders)

    print('输出图片: {}'.format(out_png))
    print('输出CSV : {}'.format(out_csv))
    for speed in sorted(speed_values.keys()):
        print('速度 {} m/s: 样本数={}, 来源目录数={}'.format(
            '{:g}'.format(speed), len(speed_values[speed]), len(speed_folders.get(speed, []))
        ))


if __name__ == '__main__':
    main()
