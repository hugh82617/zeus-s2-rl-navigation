#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
verify_pipeline.py  (v2)
------------------------------------------------------------
对 bag_to_csv.py 导出的 CSV 目录做**逐帧一致性**与**分布统计**双重核验。

逐帧一致性(per-frame transformation):
  [1] stage 1 rotation   : /scan_aligned[k] == /scan[(k + N/2) %% N]
                           元数据(angle_min/max/inc、range_min/max)透传一致。
  [2] stage 2 resample   : 对 repair_mask==0 的 beam,
                           lidar_181[k] == ((1-w)*scan_aligned[floor]
                                           + w*scan_aligned[ceil]) / MAX_DIST
                           完全由 src_map 推出。对 repair_mask==1 的 beam
                           只验证 lidar_181[k] ∈ [0, 1]。
  [3] stage 3 stack      : 从头模拟 [t0|t1|t2] 状态机(含 mask 门控的三帧
                           中位数稳态),逐元素比对 /rl/lidar_543。

分布统计(aggregate):
  [4] frame count parity             (/scan vs /scan_aligned)
  [5] stage 3 stamp monotonicity     (t0 >= t1 >= t2 per row)
  [6] stage 2 repair summary         (ratio / front / severe 分位数)
  [7] stage 1 invalid summary        (invalid_ratio / front_invalid_ratio)

用法:
    python verify_pipeline.py <csv_dir>
    python verify_pipeline.py <csv_dir> --sample-every=10
    python verify_pipeline.py <csv_dir> --sample-every=1        # 全量
    python verify_pipeline.py <csv_dir> --no-stabilize          # stage 3 模拟
                                                                # 不做稳态,
                                                                # 对应 launch 参数
                                                                # ~enable_repair_stabilize=false

--sample-every 控制 [1][2][3] 的抽样步长(默认 10):每 N 帧检查 1 帧。
                    [3] 的状态机始终全量模拟,只是比对的帧抽样;这样即使抽样
                    也能给出正确的 t1/t2 历史。
[4]-[7] 始终全量统计,不受此参数影响。

依赖:Python 标准库(csv/os/sys/math),Python 2/3 均可。
============================================================
"""

from __future__ import division, print_function

import csv
import math
import os
import sys


# 与 stage 2 v3_2 对齐的归一化常量
LIDAR_MAX_DISTANCE = 16.0
LIDAR_NUM_BEAMS    = 181
LIDAR_STACKED_DIM  = 543


# ================================================================
# 通用工具
# ================================================================
def _count_rows(csv_path):
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        next(r, None)
        return sum(1 for _ in r)


def _read_dicts(csv_path):
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, "r") as f:
        return list(csv.DictReader(f))


def _stats_1d(values):
    if not values:
        return None
    s = sorted(values); n = len(s)
    return {"mean": sum(s) / n, "p50": s[n // 2],
            "p95": s[min(n - 1, int(n * 0.95))],
            "min":  s[0], "max": s[-1], "n": n}


def _fmt_stats(label, st):
    if st is None:
        return "  %-24s [no data]" % label
    return ("  %-24s mean=%.4f  p50=%.4f  p95=%.4f  range=[%.4f, %.4f]  n=%d"
            % (label, st["mean"], st["p50"], st["p95"],
               st["min"], st["max"], st["n"]))


def _values_equal(a, b, abs_tol=1e-6):
    """NaN/inf 感知的浮点比较。"""
    a_nan = (a != a)
    b_nan = (b != b)
    if a_nan and b_nan: return True
    if a_nan or b_nan:  return False
    if math.isinf(a) and math.isinf(b):
        return (a > 0) == (b > 0)
    if math.isinf(a) or math.isinf(b):
        return False
    return abs(a - b) <= abs_tol


def _float_or_nan(s):
    try:
        return float(s)
    except (ValueError, TypeError):
        return float('nan')


# ================================================================
# [1] stage 1 rotation consistency
# ================================================================
def check_stage1_rotation(csv_dir, sample_every):
    print("\n[1] stage 1 rotation consistency  "
          "(/scan_aligned[k] == /scan[(k+N/2) %% N])")

    raw_rows     = _read_dicts(os.path.join(csv_dir, "scan_raw.csv"))
    aligned_rows = _read_dicts(os.path.join(csv_dir, "scan_aligned.csv"))
    if not raw_rows or not aligned_rows:
        print("  [skip] scan_raw.csv or scan_aligned.csv missing")
        return

    aligned_by_stamp = {row["stamp_sec"]: row for row in aligned_rows}

    META_KEYS = ["angle_min", "angle_max", "angle_increment",
                 "range_min", "range_max"]

    n_checked       = 0
    n_pair_missing  = 0
    n_meta_mismatch = 0
    n_rot_mismatch  = 0
    first_fails     = []

    for i, r_row in enumerate(raw_rows):
        if (i % sample_every) != 0:
            continue
        stamp = r_row["stamp_sec"]
        a_row = aligned_by_stamp.get(stamp)
        if a_row is None:
            n_pair_missing += 1
            continue

        # --- 元数据一致性 ---
        meta_fail = None
        for key in META_KEYS:
            if not _values_equal(_float_or_nan(r_row[key]),
                                 _float_or_nan(a_row[key]),
                                 abs_tol=1e-9):
                meta_fail = (key, r_row[key], a_row[key])
                break
        if meta_fail is not None:
            n_meta_mismatch += 1
            if len(first_fails) < 5:
                first_fails.append(("meta@%s" % stamp, meta_fail))
            continue

        # --- rotation: new[k] = old[(k + N/2) % N] ---
        N   = int(r_row["ranges_n"])
        N_a = int(a_row["ranges_n"])
        if N != N_a:
            n_rot_mismatch += 1
            if len(first_fails) < 5:
                first_fails.append(("len@%s" % stamp, (N, N_a)))
            continue

        half = N // 2
        rot_fail = None
        for k in range(N):
            src_k = (k + half) % N
            rv = _float_or_nan(r_row.get("r_%d" % src_k, "nan"))
            av = _float_or_nan(a_row.get("r_%d" % k,     "nan"))
            if not _values_equal(rv, av, abs_tol=1e-6):
                rot_fail = (k, src_k, rv, av)
                break

        if rot_fail is not None:
            n_rot_mismatch += 1
            if len(first_fails) < 5:
                first_fails.append(("rot@%s" % stamp, rot_fail))

        n_checked += 1

    total_fail = n_pair_missing + n_meta_mismatch + n_rot_mismatch
    status = "OK" if total_fail == 0 else "FAIL"
    print("  frames sampled           = %d  (every %d of %d)"
          % (n_checked, sample_every, len(raw_rows)))
    print("  stamp pair missing       = %d" % n_pair_missing)
    print("  meta mismatch            = %d" % n_meta_mismatch)
    print("  rotation mismatch frames = %d   [%s]" % (n_rot_mismatch, status))
    for tag, detail in first_fails[:5]:
        print("    %s  ->  %s" % (tag, detail))


# ================================================================
# [2] stage 2 resample consistency
# ================================================================
def check_stage2_resample(csv_dir, sample_every):
    print("\n[2] stage 2 resample consistency  "
          "(valid beams: lidar_181[k] == (1-w)*scan[floor] + w*scan[ceil]) / %.1f)"
          % LIDAR_MAX_DISTANCE)

    aligned  = _read_dicts(os.path.join(csv_dir, "scan_aligned.csv"))
    l181_stp = _read_dicts(os.path.join(csv_dir, "lidar_181_stamp.csv"))
    l181     = _read_dicts(os.path.join(csv_dir, "lidar_181.csv"))
    srcmap   = _read_dicts(os.path.join(csv_dir, "lidar_181_src_map.csv"))
    rmask    = _read_dicts(os.path.join(csv_dir, "lidar_181_repair_mask.csv"))

    missing = [name for name, lst in [
        ("scan_aligned", aligned), ("lidar_181_stamp", l181_stp),
        ("lidar_181", l181), ("lidar_181_src_map", srcmap),
        ("lidar_181_repair_mask", rmask),
    ] if not lst]
    if missing:
        print("  [skip] missing: %s" % ", ".join(missing))
        return

    aligned_by_stamp = {row["stamp_sec"]: row for row in aligned}

    # 主话题 lidar_181 和伴随(stamp/src_map/repair_mask)按索引配对
    # stage 2 严格同 scan_callback 内顺序发布,FIFO 不乱序。
    n_pairs = min(len(l181_stp), len(l181), len(srcmap), len(rmask))

    ABS_TOL = 1e-4          # 归一化后容差(原始 ~1.6mm)
    N_BEAMS = LIDAR_NUM_BEAMS

    n_checked             = 0
    n_stamp_missing       = 0
    n_valid_beam_mismatch = 0
    n_valid_beam_raw_inf  = 0   # mask=0 但 raw 是 inf/nan -> 映射 bug
    n_repaired_oor        = 0   # 修补 beam 归一化后超出 [0,1]
    first_fails           = []

    for i in range(n_pairs):
        if (i % sample_every) != 0:
            continue
        stamp = l181_stp[i]["stamp_sec"]
        a_row = aligned_by_stamp.get(stamp)
        if a_row is None:
            n_stamp_missing += 1
            continue

        sm_row   = srcmap[i]
        rm_row   = rmask[i]
        l181_row = l181[i]

        for k in range(N_BEAMS):
            try:
                observed = _float_or_nan(l181_row["v_%d" % k])
                repair   = int(round(_float_or_nan(rm_row["v_%d" % k])))
                floor_i  = int(round(_float_or_nan(sm_row["v_%d" % k])))
                ceil_i   = int(round(_float_or_nan(sm_row["v_%d" % (k + N_BEAMS)])))
                weight   = _float_or_nan(sm_row["v_%d" % (k + 2 * N_BEAMS)])
            except (KeyError, ValueError):
                continue

            if repair == 1:
                # 修补 beam 只做 [0,1] 范围自检
                if not (observed != observed):   # not NaN
                    if not (-1e-6 <= observed <= 1.0 + 1e-6):
                        n_repaired_oor += 1
                        if len(first_fails) < 5:
                            first_fails.append(
                                ("rep_oor@%s[k=%d]" % (stamp, k), observed))
                continue

            floor_v = _float_or_nan(a_row.get("r_%d" % floor_i, "nan"))
            ceil_v  = _float_or_nan(a_row.get("r_%d" % ceil_i,  "nan"))

            if (math.isnan(floor_v) or math.isinf(floor_v) or
                math.isnan(ceil_v)  or math.isinf(ceil_v)):
                # mask 说有效但 raw 是无效值 -> stage 2 映射 bug
                n_valid_beam_raw_inf += 1
                if len(first_fails) < 5:
                    first_fails.append(
                        ("mask0_but_raw_inf@%s[k=%d]" % (stamp, k),
                         (floor_i, ceil_i, floor_v, ceil_v)))
                continue

            expected = ((1.0 - weight) * floor_v + weight * ceil_v) / LIDAR_MAX_DISTANCE
            if not _values_equal(observed, expected, abs_tol=ABS_TOL):
                n_valid_beam_mismatch += 1
                if len(first_fails) < 5:
                    first_fails.append(
                        ("interp@%s[k=%d]" % (stamp, k),
                         "obs=%.6f expected=%.6f floor=%.3f ceil=%.3f w=%.3f"
                         % (observed, expected, floor_v, ceil_v, weight)))

        n_checked += 1

    total_fail = (n_stamp_missing + n_valid_beam_mismatch
                  + n_valid_beam_raw_inf + n_repaired_oor)
    status = "OK" if total_fail == 0 else "FAIL"
    print("  frames sampled                 = %d  (every %d of %d)"
          % (n_checked, sample_every, n_pairs))
    print("  stamp pair missing             = %d" % n_stamp_missing)
    print("  mask=0 but raw is inf/nan      = %d   (第 2 级 mask 映射 bug)"
          % n_valid_beam_raw_inf)
    print("  valid-beam interp mismatch     = %d   (tol=%.1e 归一化)"
          % (n_valid_beam_mismatch, ABS_TOL))
    print("  repaired-beam out of [0,1]     = %d   [%s]"
          % (n_repaired_oor, status))
    for tag, detail in first_fails[:5]:
        print("    %s  ->  %s" % (tag, detail))


# ================================================================
# [3] stage 3 stack consistency
# ================================================================
def check_stage3_stack(csv_dir, sample_every, enable_stabilize=True):
    print("\n[3] stage 3 stack consistency  "
          "(/rl/lidar_543 == simulate([t0|t1|t2] with mask-gated median stab))")

    l181  = _read_dicts(os.path.join(csv_dir, "lidar_181.csv"))
    l543  = _read_dicts(os.path.join(csv_dir, "lidar_543.csv"))
    rmask = _read_dicts(os.path.join(csv_dir, "lidar_181_repair_mask.csv"))

    missing = [name for name, lst in [
        ("lidar_181", l181), ("lidar_543", l543),
        ("lidar_181_repair_mask", rmask),
    ] if not lst]
    if missing:
        print("  [skip] missing: %s" % ", ".join(missing))
        return

    N_BEAMS = LIDAR_NUM_BEAMS
    ABS_TOL = 1e-6
    n_pairs = min(len(l181), len(l543), len(rmask))

    n_checked   = 0
    n_mismatch  = 0
    first_fails = []

    # 完整重现 stage 3 状态机(逐帧 rolling,即使不在采样帧也要维持)
    sim_t0 = None; sim_t1 = None; sim_t2 = None

    for i in range(n_pairs):
        try:
            cur  = [_float_or_nan(l181[i]["v_%d"  % k]) for k in range(N_BEAMS)]
            mask = [int(round(_float_or_nan(rmask[i]["v_%d" % k])))
                    for k in range(N_BEAMS)]
        except KeyError:
            continue

        if sim_t0 is None:
            sim_t0 = list(cur); sim_t1 = list(cur); sim_t2 = list(cur)
        else:
            sim_t2 = sim_t1
            sim_t1 = sim_t0
            new_t0 = list(cur)
            if enable_stabilize:
                for k in range(N_BEAMS):
                    if mask[k] == 1:
                        trio = sorted([cur[k], sim_t1[k], sim_t2[k]])
                        new_t0[k] = trio[1]   # median of 3
            sim_t0 = new_t0

        if (i % sample_every) != 0:
            continue

        l543_row = l543[i]
        expected = sim_t0 + sim_t1 + sim_t2
        fail_k = None
        for k in range(LIDAR_STACKED_DIM):
            try:
                obs = _float_or_nan(l543_row["v_%d" % k])
            except KeyError:
                continue
            if not _values_equal(obs, expected[k], abs_tol=ABS_TOL):
                fail_k = (k, obs, expected[k])
                break

        if fail_k is not None:
            n_mismatch += 1
            if len(first_fails) < 5:
                k, obs, exp = fail_k
                seg = "t0" if k < N_BEAMS else ("t1" if k < 2 * N_BEAMS else "t2")
                first_fails.append(("frame[%d] %s[k=%d]" % (i, seg, k % N_BEAMS),
                                    (obs, exp)))
        n_checked += 1

    status = "OK" if n_mismatch == 0 else "FAIL"
    stab_str = "with stabilize" if enable_stabilize else "WITHOUT stabilize"
    print("  simulation mode                = %s" % stab_str)
    print("  frames sampled                 = %d  (every %d of %d)"
          % (n_checked, sample_every, n_pairs))
    print("  stack mismatch frames          = %d   [%s]" % (n_mismatch, status))
    for tag, detail in first_fails[:5]:
        print("    %s  ->  obs=%.6f  expected=%.6f" % (tag, detail[0], detail[1]))
    if n_mismatch > 0 and enable_stabilize:
        print("  hint: 如果 stage 3 启动时 ~enable_repair_stabilize=false,"
              "请加 --no-stabilize 重跑。")


# ================================================================
# [4] frame count parity
# ================================================================
def check_frame_parity(csv_dir):
    print("\n[4] frame count parity  (/scan vs /scan_aligned)")
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
        print("        负的 diff 表示下游录到了第 1 级没见过的帧(不应出现)。")


# ================================================================
# [5] stage 3 stamp monotonicity
# ================================================================
def check_stamp_monotonicity(csv_dir):
    print("\n[5] stage 3 stamp monotonicity  "
          "(lidar_543_stamps, t0 >= t1 >= t2)")
    rows = _read_dicts(os.path.join(csv_dir, "lidar_543_stamps.csv"))
    if rows is None:
        print("  [skip] lidar_543_stamps.csv not found")
        return
    n_total = 0; n_violations = 0; first_5 = []; first_frame_like = 0
    for i, row in enumerate(rows):
        try:
            t0 = float(row["v_0"]); t1 = float(row["v_1"]); t2 = float(row["v_2"])
        except (KeyError, ValueError):
            continue
        n_total += 1
        if t0 == t1 == t2:
            first_frame_like += 1; continue
        if t0 >= t1 >= t2:
            continue
        n_violations += 1
        if len(first_5) < 5:
            first_5.append((i, t0, t1, t2))
    status = "OK" if n_violations == 0 else "%d violations" % n_violations
    print("  total rows            = %d" % n_total)
    print("  first-frame-like rows = %d  (t0 == t1 == t2, 启动瞬态正常)"
          % first_frame_like)
    print("  violations            = %d   [%s]" % (n_violations, status))
    for i, t0, t1, t2 in first_5:
        print("    row %d: t0=%.6f t1=%.6f t2=%.6f  (dt01=%+.4f, dt12=%+.4f)"
              % (i, t0, t1, t2, t0 - t1, t1 - t2))


# ================================================================
# [6] stage 2 repair summary
# ================================================================
def check_repair_summary(csv_dir):
    print("\n[6] stage 2 repair summary  (lidar_181_fill_stats)")
    rows = _read_dicts(os.path.join(csv_dir, "lidar_181_fill_stats.csv"))
    if rows is None:
        print("  [skip] lidar_181_fill_stats.csv not found")
        return
    ratios = []; counts = []; fronts = []; severes = []
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


# ================================================================
# [7] stage 1 invalid summary
# ================================================================
def check_invalid_summary(csv_dir):
    print("\n[7] stage 1 invalid summary  (scan_aligned_invalid_stats)")
    rows = _read_dicts(os.path.join(csv_dir, "scan_aligned_invalid_stats.csv"))
    if rows is None:
        print("  [skip] scan_aligned_invalid_stats.csv not found")
        return
    counts = []; ratios = []; front_counts = []; front_ratios = []
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


# ================================================================
# main
# ================================================================
def _parse_args():
    if len(sys.argv) < 2:
        sys.stderr.write(
            "Usage: %s <csv_dir> [--sample-every=N] [--no-stabilize]\n"
            % sys.argv[0])
        sys.exit(1)
    csv_dir = sys.argv[1]
    sample_every = 10
    enable_stab = True
    for arg in sys.argv[2:]:
        if arg.startswith("--sample-every="):
            try:
                sample_every = max(1, int(arg.split("=", 1)[1]))
            except ValueError:
                sys.stderr.write("ERROR: invalid --sample-every\n"); sys.exit(1)
        elif arg == "--no-stabilize":
            enable_stab = False
        else:
            sys.stderr.write("ERROR: unknown arg: %s\n" % arg); sys.exit(1)
    return csv_dir, sample_every, enable_stab


def main():
    csv_dir, sample_every, enable_stab = _parse_args()
    if not os.path.isdir(csv_dir):
        sys.stderr.write("ERROR: csv dir not found: %s\n" % csv_dir); sys.exit(1)

    print("[verify_pipeline v2] csv_dir       = %s" % csv_dir)
    print("[verify_pipeline v2] sample_every  = %d" % sample_every)
    print("[verify_pipeline v2] stage 3 stab  = %s" % enable_stab)

    print("\n========== 逐帧一致性(per-frame transformation) ==========")
    check_stage1_rotation(csv_dir, sample_every)
    check_stage2_resample(csv_dir, sample_every)
    check_stage3_stack(csv_dir, sample_every, enable_stabilize=enable_stab)

    print("\n========== 分布统计(aggregate) ==========")
    check_frame_parity(csv_dir)
    check_stamp_monotonicity(csv_dir)
    check_repair_summary(csv_dir)
    check_invalid_summary(csv_dir)
    print("")


if __name__ == "__main__":
    main()
