#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
verify_pipeline.py
------------------------------------------------------------
对 bag_to_csv.py 导出的 CSV 目录做 4 项快速核验,覆盖《总体修改意见》
里最关心的几类回归点:

  [1] 帧数对齐:        /scan vs /scan_aligned         (第 1 级是否丢帧)
  [2] 第 3 级时间单调: lidar_543_stamps.csv 每行 t0 >= t1 >= t2
  [3] 修补分布:        lidar_181_fill_stats.csv 的
                       repair_ratio / front_count / severe_ratio 分布
  [4] 无效源分布:      scan_aligned_invalid_stats.csv 的
                       invalid_ratio / front_invalid_ratio 分布

用法:
    python verify_pipeline.py <csv_dir>

依赖: 纯 Python 标准库(csv/os/sys/math),Python 2/3 均可。
============================================================
"""

from __future__ import division, print_function

import csv
import os
import sys


# ------------------------------------------------------------
# utils
# ------------------------------------------------------------
def _count_rows(csv_path):
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        next(r, None)   # skip header
        return sum(1 for _ in r)


def _read_dicts(csv_path):
    """Returns list of dict rows, or None if file absent."""
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, "r") as f:
        return list(csv.DictReader(f))


def _stats_1d(values):
    """mean / p50 / p95 / min / max of a 1D list."""
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    mean = sum(s) / n
    p50  = s[n // 2]
    p95  = s[min(n - 1, int(n * 0.95))]
    return {"mean": mean, "p50": p50, "p95": p95,
            "min":  s[0], "max": s[-1], "n": n}


def _fmt_stats(label, st):
    if st is None:
        return "  %-24s [no data]" % label
    return ("  %-24s mean=%.4f  p50=%.4f  p95=%.4f  range=[%.4f, %.4f]  n=%d"
            % (label, st["mean"], st["p50"], st["p95"],
               st["min"], st["max"], st["n"]))


# ------------------------------------------------------------
# checks
# ------------------------------------------------------------
def check_frame_parity(csv_dir):
    print("\n[1] frame count parity  (/scan vs /scan_aligned)")
    raw     = _count_rows(os.path.join(csv_dir, "scan_raw.csv"))
    aligned = _count_rows(os.path.join(csv_dir, "scan_aligned.csv"))
    if raw is None or aligned is None:
        print("  [skip] missing CSV (raw=%s, aligned=%s)" % (raw, aligned))
        return
    diff = raw - aligned
    status = "OK" if diff == 0 else "MISMATCH"
    print("  /scan frames          = %d" % raw)
    print("  /scan_aligned frames  = %d" % aligned)
    print("  diff (raw - aligned)  = %d   [%s]" % (diff, status))
    if diff != 0:
        print("  hint: 正的 diff 表示第 1 级丢帧 / 节点丢帧;")
        print("        负的 diff 表示下游录制到了第 1 级没见过的帧(不应出现)。")


def check_stamp_monotonicity(csv_dir):
    print("\n[2] stage 3 stamp monotonicity  (lidar_543_stamps, t0 >= t1 >= t2)")
    rows = _read_dicts(os.path.join(csv_dir, "lidar_543_stamps.csv"))
    if rows is None:
        print("  [skip] lidar_543_stamps.csv not found")
        return
    n_total      = 0
    n_violations = 0
    first_5_viol = []
    first_frame_like = 0   # t0 == t1 == t2 (正常发生在首帧)
    for i, row in enumerate(rows):
        try:
            t0 = float(row["v_0"])
            t1 = float(row["v_1"])
            t2 = float(row["v_2"])
        except (KeyError, ValueError):
            continue
        n_total += 1
        if t0 == t1 == t2:
            first_frame_like += 1
            continue
        if t0 >= t1 >= t2:
            continue
        n_violations += 1
        if len(first_5_viol) < 5:
            first_5_viol.append((i, t0, t1, t2))
    status = "OK" if n_violations == 0 else "%d violations" % n_violations
    print("  total rows            = %d" % n_total)
    print("  first-frame-like rows = %d  (t0 == t1 == t2, 启动瞬态正常)" % first_frame_like)
    print("  violations            = %d   [%s]" % (n_violations, status))
    for i, t0, t1, t2 in first_5_viol:
        dt01 = t0 - t1
        dt12 = t1 - t2
        print("    row %d: t0=%.6f t1=%.6f t2=%.6f  (dt01=%+.4f, dt12=%+.4f)"
              % (i, t0, t1, t2, dt01, dt12))


def check_repair_summary(csv_dir):
    print("\n[3] stage 2 repair summary  (lidar_181_fill_stats)")
    rows = _read_dicts(os.path.join(csv_dir, "lidar_181_fill_stats.csv"))
    if rows is None:
        print("  [skip] lidar_181_fill_stats.csv not found")
        return
    # 列布局: bag_time, data_n, v_0=repair_ratio, v_1=repair_count,
    #         v_2=front_repair_count, v_3=severe_repair_ratio
    ratios   = []
    counts   = []
    fronts   = []
    severes  = []
    for row in rows:
        try:
            ratios.append( float(row["v_0"]))
            counts.append( float(row["v_1"]))
            fronts.append( float(row["v_2"]))
            severes.append(float(row["v_3"]))
        except (KeyError, ValueError):
            continue
    print(_fmt_stats("repair_ratio",   _stats_1d(ratios)))
    print(_fmt_stats("repair_count",   _stats_1d(counts)))
    print(_fmt_stats("front_count",    _stats_1d(fronts)))
    print(_fmt_stats("severe_ratio",   _stats_1d(severes)))
    if ratios:
        high = sum(1 for r in ratios if r > 0.30)
        if high > 0:
            print("  warn: %d 帧 (%.1f%%) 的 repair_ratio > 30%%,"
                  " 建议检查传感器是否接触不良或场景过开阔"
                  % (high, 100.0 * high / len(ratios)))


def check_invalid_summary(csv_dir):
    print("\n[4] stage 1 invalid summary  (scan_aligned_invalid_stats)")
    rows = _read_dicts(os.path.join(csv_dir, "scan_aligned_invalid_stats.csv"))
    if rows is None:
        print("  [skip] scan_aligned_invalid_stats.csv not found")
        return
    # 列布局: bag_time, data_n, v_0=invalid_count, v_1=invalid_ratio,
    #         v_2=front_invalid_count, v_3=front_invalid_ratio
    counts        = []
    ratios        = []
    front_counts  = []
    front_ratios  = []
    for row in rows:
        try:
            counts.append(       float(row["v_0"]))
            ratios.append(       float(row["v_1"]))
            front_counts.append( float(row["v_2"]))
            front_ratios.append( float(row["v_3"]))
        except (KeyError, ValueError):
            continue
    print(_fmt_stats("invalid_count",       _stats_1d(counts)))
    print(_fmt_stats("invalid_ratio",       _stats_1d(ratios)))
    print(_fmt_stats("front_invalid_count", _stats_1d(front_counts)))
    print(_fmt_stats("front_invalid_ratio", _stats_1d(front_ratios)))


# ------------------------------------------------------------
# main
# ------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        sys.stderr.write("Usage: %s <csv_dir>\n" % sys.argv[0])
        sys.exit(1)
    csv_dir = sys.argv[1]
    if not os.path.isdir(csv_dir):
        sys.stderr.write("ERROR: csv dir not found: %s\n" % csv_dir)
        sys.exit(1)

    print("[verify_pipeline] csv_dir = %s" % csv_dir)
    check_frame_parity(csv_dir)
    check_stamp_monotonicity(csv_dir)
    check_repair_summary(csv_dir)
    check_invalid_summary(csv_dir)
    print("")


if __name__ == "__main__":
    main()
