#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aggregate target_loss_durations_sec across run_*/metrics.json and plot the
conditional reacquire-probability curve.

统计约定:
  - 重捕成功 = 失锁时长 < LOSS_TIMEOUT_SEC (默认 10s)
  - 横轴 t: 已经持续了 t 秒的失锁事件
  - 纵轴:  P(最终重捕 | 已丢 ≥ t)
           = #{loss: loss >= t AND loss < LOSS_TIMEOUT} / #{loss: loss >= t}
  - 仅对"成功找回"的失锁事件(< LOSS_TIMEOUT)求 mean / max 重捕用时

用法:
  rosrun planning reacq_curve.py --dirs A B --out out.png
  或
  python3 reacq_curve.py --dirs A B --out out.png
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_losses(paths, timeout_default=10.0):
    """paths 里每一项可以是:
    - 目录: 按 run_*/metrics.json 递归聚合
    - .json 文件: 当做 summary (list[dict] 或单 dict)
    """
    losses = []
    timeout = None

    def ingest(m):
        nonlocal timeout
        durs = m.get("target_loss_durations_sec", [])
        losses.extend(float(x) for x in durs)
        t = m.get("task_success_loss_timeout_sec", None)
        if t is not None and timeout is None:
            timeout = float(t)

    for p in paths:
        if os.path.isdir(p):
            for f in sorted(glob.glob(os.path.join(p, "run_*", "metrics.json"))):
                with open(f) as fh:
                    arr = json.load(fh)
                if not isinstance(arr, list):
                    arr = [arr]
                for m in arr:
                    ingest(m)
        elif os.path.isfile(p) and p.endswith(".json"):
            with open(p) as fh:
                arr = json.load(fh)
            if not isinstance(arr, list):
                arr = [arr]
            for m in arr:
                ingest(m)
        else:
            print(f"[reacq_curve] skip (not a dir or .json): {p}", file=sys.stderr)

    if timeout is None:
        timeout = timeout_default
    return np.asarray(losses, dtype=float), float(timeout)


def reacq_curve(losses, timeout, t_max=None, n=400):
    losses = np.asarray(losses, dtype=float)
    if t_max is None:
        t_max = float(timeout)
    ts = np.linspace(0.0, t_max, n)
    prob = np.zeros_like(ts)
    count_ge_t = np.zeros_like(ts)
    for i, t in enumerate(ts):
        mask_ge = losses >= t
        n_ge = int(mask_ge.sum())
        count_ge_t[i] = n_ge
        if n_ge == 0:
            prob[i] = np.nan
            continue
        n_ok = int(((losses >= t) & (losses < timeout)).sum())
        prob[i] = n_ok / n_ge
    return ts, prob, count_ge_t


def main():
    ap = argparse.ArgumentParser()
    # 旧版单系列入口（向后兼容）
    ap.add_argument("--dirs", nargs="+", default=[])
    ap.add_argument("--extra", nargs="+", default=[])
    # 新版多系列入口：每条 --series 一条曲线
    ap.add_argument("--series", action="append", default=[], nargs="+",
                    metavar="PATH",
                    help="一条曲线的数据源 (多个目录/JSON 任意混合); "
                         "可指定多次以叠加多条曲线。首项若以 'name=' 起始则作为图例名。")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--timeout", type=float, default=None,
                    help="override task_success_loss_timeout_sec (s)")
    args = ap.parse_args()

    # 汇总 series
    series = []
    if args.dirs or args.extra:
        series.append(("run", list(args.dirs) + list(args.extra)))
    for s in args.series:
        name = None
        paths = list(s)
        if paths and paths[0].startswith("name="):
            name = paths[0][len("name="):]
            paths = paths[1:]
        if not paths:
            ap.error("--series 至少需要 1 个路径")
        series.append((name or f"series{len(series)+1}", paths))

    if not series:
        ap.error("must pass --series (or --dirs/--extra)")

    colors = ["#2c7fb8", "#e34a33", "#31a354", "#756bb1", "#d95f0e"]
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    timeout_used = args.timeout

    summaries = []
    for i, (name, paths) in enumerate(series):
        losses, t_auto = load_losses(paths)
        tt = float(args.timeout) if args.timeout is not None else t_auto
        if timeout_used is None:
            timeout_used = tt
        if losses.size == 0:
            print(f"[reacq_curve] series '{name}' has no loss events, skip")
            continue

        ok = losses[losses < tt]
        fail = losses[losses >= tt]
        mean_ok = float(ok.mean()) if ok.size else float("nan")
        max_ok = float(ok.max()) if ok.size else float("nan")
        summaries.append((name, losses.size, ok.size, fail.size, mean_ok, max_ok))

        ts, prob, _ = reacq_curve(losses, tt, t_max=tt)
        color = colors[i % len(colors)]
        ax.plot(ts, prob, color=color, lw=1.8, label=name)

    if timeout_used is None:
        timeout_used = 10.0

    # 终端打印
    print(f"{'series':<20} {'N':>5} {'ok':>5} {'fail':>5} {'mean':>8} {'max':>8}")
    for name, N, ok, fail, m, mx in summaries:
        print(f"{name:<20} {N:>5d} {ok:>5d} {fail:>5d} {m:>8.3f} {mx:>8.3f}")

    ax.axhline(1.0, color="grey", lw=0.8, ls=":")
    ax.set_xlim(0, timeout_used)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlabel("loss duration t (s)")
    ax.set_ylabel("P(finally reacquire | ongoing loss >= t)")
    ax.set_title("Conditional reacquire probability")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out, dpi=160)
    plt.close(fig)
    print(f"[reacq_curve] wrote {args.out}")


if __name__ == "__main__":
    main()
