#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
LiDAR Observation Pipeline (Single-Node Multi-Stage)  v1
============================================================
将原 3 个 ROS 节点
    laser_switch                  v3_1   (Pipeline Stage 1/3)
    lidar_obs_single_frame        v3_2   (Pipeline Stage 2/3)
    lidar_stack_543               v2_1   (Pipeline Stage 3/3)
合并为 **一个 ROS 节点内部的分阶段处理流水线 (intra-node pipeline)**。

主入口     : /scan  ->  scan_callback  ->  Stage1 -> Stage2 -> Stage3
最终输出   : /rl/lidar_543

---------------------------------------------------------------
设计要点
---------------------------------------------------------------
1) **算法 bit-level 一致**
   三阶段的核心算法(180° 半圈重排、invalid 判定、d_space/d_time/d_frame
   动态修补、181 维线性插值与归一化、median3 时域稳态、[t0|t1|t2] 堆叠
   顺序)从原节点逐字搬过来,**未做任何数值/语义上的修改**。

2) **取消所有跨节点 ROS 通信**
   原本依赖 topic 串联:
       /scan -> /scan_aligned -> /rl/lidar_181 -> /rl/lidar_543
   重构后改为函数调用 + 字典传递:
       scan_callback() -> stage1.process() -> stage2.process() -> stage3.process()
   原 Stage 3 的 _stamp_queue / _mask_queue / _fs_queue 三个 FIFO 全部
   删除,因为同一帧的 stamp / repair_mask / fill_stats 与 lidar_181
   现在是同一个 process() 调用的输出,天然严格同帧对齐,popleft 错位
   故障模式从根源消除。

3) **统一时间戳来源**
   全链统一使用 `scan.header.stamp`,Stage 2 / Stage 3 不再有 fallback
   到 rospy.Time.now() 的路径。原 Stage 3 的 stamp_fallback_frames 计数
   器被保留(为了日志格式兼容),但在单节点模式下它**永远 = 0**。

4) **历史滑窗仍是节点内部状态**
   - Stage 2 的 _time_hist (deque maxlen=3): 维护原始索引位置的过去
     3 帧修补值,供 d_time 兜底。
   - Stage 3 的 t0/t1/t2: 维护 181 维输出的过去 3 帧,供堆叠和稳态。
   两者都是节点内部状态,与 topic 到达顺序无关。

5) **诊断话题原样保留**
   为保证下游 (rl_planner_node 等) 与既有调试工具完全兼容,本节点
   按原 3 节点的话题名/语义/类型完整发布所有诊断话题。可通过参数
   ~publish_legacy_diag 一键关闭非必要诊断 topic(默认开启)。

6) **三阶段在代码中保持逻辑分层**
       Stage1Module    方向校正 + invalid mask
       Stage2Module    动态修补 + 181 维观测 + repair_mask 映射
       Stage3Module    三帧滑窗 + median3 时域稳态 + 543 堆叠
   每个 Module 是纯 Python 类,**不直接订阅/发布任何 ROS topic**;
   订阅与发布全部由外层 LidarPipelineNode 统一处理。这样 Module 之间
   的依赖只有"上一阶段的返回字典",可以独立做单元测试 / 替换。

---------------------------------------------------------------
保持不变的硬约束(与原 3 节点对照)
---------------------------------------------------------------
Stage 1:
  - 180° 重排                     new[i] = old[(i + N/2) % N]
  - invalid 判定                  inf / nan / 负值 / <range_min / >range_max
Stage 2:
  - cur_range 融合策略           max(node.range_min, scan.range_min)
                                  min(node.max_distance, scan.range_max)
  - d_space / d_time / d_frame    权重 (0.60,0.25,0.15) / (0.35,0.35,0.30) /
                                       (0.10,0.40,0.50)
  - 181 维线性插值采样
  - normalization                 r / max_distance  (12.0 m, 与训练对齐)
  - 修补帧追加进 _time_hist       仅在非"整帧全无效"分支 + 兜底分支两处
Stage 3:
  - 三帧滑窗结构                  t0/t1/t2 + 对应 id / stamp
  - 修补 beam 稳态                cur[k] <- median(cur[k], t1[k], t2[k])
  - [t0 | t1 | t2] 拼接顺序

============================================================
"""

from __future__ import division, print_function

import math
import threading
from collections import deque

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import (
    Float32MultiArray,
    Float64MultiArray,
    MultiArrayDimension,
    Header,
)


# ============================================================
# 链路统一常量(从原 3 节点合并,数值保持一致)
# ============================================================
LIDAR_MAX_DISTANCE = 12.0      # 归一化分母 (m); 2026-04-20: 16->12 与训练对齐
LIDAR_NUM_BEAMS    = 181
LIDAR_FOV_MIN_DEG  = -135.0
LIDAR_FOV_MAX_DEG  = 135.0
LIDAR_RANGE_MIN    = 0.05      # 节点级下限,与 scan.range_min 取 max
LIDAR_STACK_LEN    = 3
LIDAR_STACKED_DIM  = LIDAR_NUM_BEAMS * LIDAR_STACK_LEN   # 543

# Stage 1 invalid 检测的前向扇区半宽(度)
INVALID_FRONT_FOV_DEG = 20.0

# Stage 2 修补配置(原 lidar_obs_single_frame v3_2)
REPAIR_HIST_LEN          = 3
REPAIR_SPACE_NEIGHBORS   = 6
REPAIR_FRONT_FOV_DEG     = 20.0
REPAIR_D_FRAME_12_DEG    = 12.0
REPAIR_D_FRAME_25_DEG    = 25.0
REPAIR_D_FRAME_LOCAL_Q   = 0.30
REPAIR_D_FRAME_GLOBAL_Q  = 0.35
REPAIR_D_FRAME_LOCAL_MIN = 5
REPAIR_D_FRAME_WIDEN_MIN = 3

REPAIR_W_SPACE_FULL    = (0.60, 0.25, 0.15)
REPAIR_W_SPACE_PARTIAL = (0.35, 0.35, 0.30)
REPAIR_W_SPACE_EMPTY   = (0.10, 0.40, 0.50)


# ============================================================
# 通用 MultiArray 构造工具
# ============================================================
def _build_f32_msg(data, label):
    msg = Float32MultiArray()
    dim = MultiArrayDimension()
    dim.label  = label
    dim.size   = len(data)
    dim.stride = len(data)
    msg.layout.dim = [dim]
    msg.layout.data_offset = 0
    msg.data = data
    return msg


def _build_f64_msg(data, label):
    msg = Float64MultiArray()
    dim = MultiArrayDimension()
    dim.label  = label
    dim.size   = len(data)
    dim.stride = len(data)
    msg.layout.dim = [dim]
    msg.layout.data_offset = 0
    msg.data = data
    return msg


# ============================================================
# Stage 1 Module : 方向校正 + invalid mask
# ------------------------------------------------------------
# 与 laser_switch v3_1 算法完全一致,只是把 ROS 通信外壳剥离。
# ============================================================
class Stage1Module(object):
    def __init__(self, params):
        self.front_fov_deg = float(params.get('front_fov_deg', INVALID_FRONT_FOV_DEG))

        # 几何缓存(首帧或 n / angle_min / angle_inc 变化时重算)
        self._front_half_idx   = None
        self._front_center_idx = None
        self._cached_n         = None
        self._cached_angle_min = None
        self._cached_angle_inc = None

        # 诊断计数器
        self.recv_count  = 0
        self.empty_count = 0

    # --------------------------------------------------------
    # 工具
    # --------------------------------------------------------
    @staticmethod
    def rotate_half_turn(data_list):
        """new[i] = old[(i + N/2) % N];与 v3_1 完全一致。"""
        n = len(data_list)
        if n == 0:
            return data_list
        half = n // 2
        if n % 2 != 0:
            rospy.logwarn_throttle(
                5.0,
                "[pipeline.stage1] scan length is odd: %d, half-turn shift "
                "will have 1-sample mismatch.",
                n,
            )
        first_half = data_list[:half]
        new_list   = data_list[half:]
        new_list.extend(first_half)
        return new_list

    @staticmethod
    def _is_invalid(r, range_min, range_max):
        """inf / nan / 负值 / <range_min / >range_max 视为 invalid。"""
        if r is None:
            return True
        if math.isnan(r) or math.isinf(r):
            return True
        if r < 0.0:
            return True
        if r < range_min:
            return True
        if r > range_max:
            return True
        return False

    @staticmethod
    def safe_value_str(value):
        try:
            if value == float("inf"):
                return "inf"
            if value != value:   # NaN
                return "nan"
            return "%.3f" % value
        except Exception:
            return str(value)

    def _update_front_zone_cache(self, n, angle_min, angle_increment,
                                 range_min=None, range_max=None):
        changed = (
            self._cached_n         != n or
            self._cached_angle_min != angle_min or
            self._cached_angle_inc != angle_increment
        )
        if not changed:
            return
        if angle_increment > 0:
            self._front_half_idx = max(
                1, int(math.radians(self.front_fov_deg) / angle_increment))
            self._front_center_idx = int(round((-angle_min) / angle_increment))
            self._front_center_idx = max(0, min(n - 1, self._front_center_idx))
        else:
            self._front_half_idx   = max(1, n // 18)   # 约 ±10° 的兜底
            self._front_center_idx = n // 2
        self._cached_n         = n
        self._cached_angle_min = angle_min
        self._cached_angle_inc = angle_increment
        r_min_str = ("%.3f" % range_min) if range_min is not None else "?"
        r_max_str = ("%.3f" % range_max) if range_max is not None else "?"
        rospy.logwarn(
            "[pipeline.stage1] scan geometry (re)locked: n=%d  center_idx=%d  "
            "half_idx=%d  fov=±%.1f°  angle_min=%.5f  angle_inc=%.5f  "
            "range=[%s, %s]",
            n, self._front_center_idx, self._front_half_idx,
            self.front_fov_deg, angle_min, angle_increment,
            r_min_str, r_max_str,
        )

    # --------------------------------------------------------
    # 主处理函数
    # --------------------------------------------------------
    def process(self, scan_msg):
        """
        接收原始 /scan,返回处理结果字典(供 Stage 2 直接消费)。
        若 scan 为空帧,返回 None。
        """
        self.recv_count += 1
        n = len(scan_msg.ranges)
        if n == 0:
            self.empty_count += 1
            rospy.logwarn_throttle(2.0, "[pipeline.stage1] empty scan received.")
            return None

        old_ranges      = list(scan_msg.ranges)
        old_intensities = list(scan_msg.intensities)

        # --- 方向校正 ---
        new_ranges = self.rotate_half_turn(old_ranges)
        if len(old_intensities) == n:
            new_intensities = self.rotate_half_turn(old_intensities)
        else:
            new_intensities = old_intensities

        # --- invalid_mask / invalid_stats (基于 post-rotation ranges)---
        r_min = scan_msg.range_min
        r_max = scan_msg.range_max
        invalid_mask = [0] * n
        invalid_count = 0
        for i in range(n):
            if self._is_invalid(new_ranges[i], r_min, r_max):
                invalid_mask[i] = 1
                invalid_count += 1

        self._update_front_zone_cache(
            n, scan_msg.angle_min, scan_msg.angle_increment,
            range_min=scan_msg.range_min, range_max=scan_msg.range_max,
        )

        lo = max(0, self._front_center_idx - self._front_half_idx)
        hi = min(n, self._front_center_idx + self._front_half_idx + 1)
        front_total = hi - lo
        front_invalid_count = sum(invalid_mask[lo:hi]) if front_total > 0 else 0
        front_invalid_ratio = (
            (front_invalid_count / float(front_total)) if front_total > 0 else 0.0
        )
        invalid_ratio = (invalid_count / float(n)) if n > 0 else 0.0

        return {
            'aligned_ranges':      new_ranges,
            'aligned_intensities': new_intensities,
            'invalid_mask':        invalid_mask,
            'invalid_stats':       [
                float(invalid_count),
                float(invalid_ratio),
                float(front_invalid_count),
                float(front_invalid_ratio),
            ],
            'header':              scan_msg.header,
            'angle_min':           scan_msg.angle_min,
            'angle_max':           scan_msg.angle_max,
            'angle_increment':     scan_msg.angle_increment,
            'time_increment':      scan_msg.time_increment,
            'scan_time':           scan_msg.scan_time,
            'range_min':           scan_msg.range_min,
            'range_max':           scan_msg.range_max,
            'n':                   n,
            # 给日志用,不参与后续处理
            'old_ranges':          old_ranges,
        }


# ============================================================
# Stage 2 Module : 动态修补 + 181 维观测 + repair_mask 映射
# ------------------------------------------------------------
# 与 lidar_obs_single_frame v3_2 算法完全一致,只是把 ROS 通信外
# 壳剥离。所有内部状态 (_time_hist / _src_map / _half_idx_* /
# _front_center_raw 等) 与原节点一一对应。
# ============================================================
class Stage2Module(object):
    def __init__(self, params):
        # --- 链路统一参数 ---
        self.fov_min_deg   = float(params.get('fov_min_deg',   LIDAR_FOV_MIN_DEG))
        self.fov_max_deg   = float(params.get('fov_max_deg',   LIDAR_FOV_MAX_DEG))
        self.num_beams     = int  (params.get('num_beams',     LIDAR_NUM_BEAMS))
        self.max_distance  = float(params.get('max_distance',  LIDAR_MAX_DISTANCE))
        self.range_min     = float(params.get('range_min',     LIDAR_RANGE_MIN))
        self.invalid_fill  = float(params.get('invalid_fill',  LIDAR_MAX_DISTANCE * 0.5))

        # --- 处理选项 ---
        self.use_linear_interpolation = bool(params.get('use_linear_interpolation', True))
        self.reverse_output           = bool(params.get('reverse_output', False))

        # --- 修补配置 ---
        self.hist_len        = int  (params.get('hist_len',        REPAIR_HIST_LEN))
        self.space_neighbors = int  (params.get('space_neighbors', REPAIR_SPACE_NEIGHBORS))
        self.front_fov_deg   = float(params.get('front_fov_deg',   REPAIR_FRONT_FOV_DEG))

        # --- 内部状态 ---
        self.target_angles = self._build_target_angles_rad(
            self.fov_min_deg, self.fov_max_deg, self.num_beams)

        # src_map 缓存(几何变化时由 _update_scan_geometry_cache 重建)
        self._src_map        = None
        self._src_map_msg    = None
        self._src_floor      = None
        self._src_ceil       = None
        self._src_weight     = None

        # 几何参数缓存
        self._cached_n_src     = None
        self._cached_angle_min = None
        self._cached_angle_inc = None

        # 修补相关的角度索引半宽缓存
        self._half_idx_12      = None
        self._half_idx_25      = None
        self._front_half_idx   = None
        self._front_center_raw = None

        # 时域历史(每元素是一帧修补后的 ranges)
        self._time_hist = deque(maxlen=self.hist_len)

        self.scan_count = 0

    # --------------------------------------------------------
    # 角度构造
    # --------------------------------------------------------
    @staticmethod
    def _build_target_angles_rad(min_deg, max_deg, num_beams):
        if num_beams <= 1:
            return [math.radians(min_deg)]
        step = (max_deg - min_deg) / float(num_beams - 1)
        return [math.radians(min_deg + i * step) for i in range(num_beams)]

    # --------------------------------------------------------
    # 有效性判断(与 Stage 1 同语义,但 cur_range_* 由本帧融合而来)
    # --------------------------------------------------------
    def _is_valid_raw(self, r, cur_range_min, cur_range_max):
        if r is None:
            return False
        if math.isnan(r) or math.isinf(r):
            return False
        if r < cur_range_min:
            return False
        if r > cur_range_max:
            return False
        return True

    # --------------------------------------------------------
    # 数学工具(纯 Python,与 v1/v2 一致的轻量风格)
    # --------------------------------------------------------
    @staticmethod
    def _median(values):
        s = sorted(values)
        n = len(s)
        mid = n // 2
        if n % 2 == 1:
            return s[mid]
        return 0.5 * (s[mid - 1] + s[mid])

    @staticmethod
    def _quantile_sorted(sorted_values, q):
        n = len(sorted_values)
        if n == 0:
            return 0.0
        if n == 1:
            return sorted_values[0]
        pos = q * (n - 1)
        lo = int(math.floor(pos))
        hi = min(n - 1, lo + 1)
        w = pos - lo
        return (1.0 - w) * sorted_values[lo] + w * sorted_values[hi]

    # --------------------------------------------------------
    # d_space / d_time / d_frame
    # --------------------------------------------------------
    def _compute_d_space(self, ranges, is_valid, i, n):
        left_vals = []
        nearest_left = None
        for off in range(1, n):
            j = i - off
            if j < 0:
                break
            if is_valid[j]:
                if nearest_left is None:
                    nearest_left = off
                left_vals.append(ranges[j])
                if len(left_vals) >= self.space_neighbors:
                    break

        right_vals = []
        nearest_right = None
        for off in range(1, n):
            j = i + off
            if j >= n:
                break
            if is_valid[j]:
                if nearest_right is None:
                    nearest_right = off
                right_vals.append(ranges[j])
                if len(right_vals) >= self.space_neighbors:
                    break

        if left_vals and right_vals:
            m_L = self._median(left_vals)
            m_R = self._median(right_vals)
            total = float(nearest_left + nearest_right)
            w_R = (nearest_left / total) if total > 0 else 0.5
            d_space = (1.0 - w_R) * m_L + w_R * m_R
            return d_space, 'full'
        if left_vals:
            return self._median(left_vals), 'partial'
        if right_vals:
            return self._median(right_vals), 'partial'
        return 0.0, 'empty'

    def _compute_d_time(self, i):
        if not self._time_hist:
            return 0.0, False
        vals = []
        for past in self._time_hist:
            if 0 <= i < len(past):
                vals.append(past[i])
        if not vals:
            return 0.0, False
        return self._median(vals), True

    def _compute_d_frame(self, ranges, is_valid, i, n, frame_q_global):
        h12 = self._half_idx_12 if self._half_idx_12 is not None else max(1, n // 30)
        h25 = self._half_idx_25 if self._half_idx_25 is not None else max(1, n // 14)

        # 一级:±12°
        lo = max(0, i - h12)
        hi = min(n, i + h12 + 1)
        local = [ranges[j] for j in range(lo, hi) if is_valid[j]]
        if len(local) >= REPAIR_D_FRAME_LOCAL_MIN:
            return self._quantile_sorted(sorted(local), REPAIR_D_FRAME_LOCAL_Q)

        # 二级:±25°
        lo = max(0, i - h25)
        hi = min(n, i + h25 + 1)
        local = [ranges[j] for j in range(lo, hi) if is_valid[j]]
        if len(local) >= REPAIR_D_FRAME_WIDEN_MIN:
            return self._quantile_sorted(sorted(local), REPAIR_D_FRAME_LOCAL_Q)

        # 三级:全帧
        return frame_q_global

    @staticmethod
    def _pick_weights(space_status):
        if space_status == 'full':
            return REPAIR_W_SPACE_FULL
        if space_status == 'partial':
            return REPAIR_W_SPACE_PARTIAL
        return REPAIR_W_SPACE_EMPTY

    # --------------------------------------------------------
    # 几何缓存维护
    # --------------------------------------------------------
    def _update_scan_geometry_cache(self, n_src, angle_min, angle_inc):
        """
        与原 v3_2 一致:仅当 n_src/angle_min/angle_inc 任一变化时重建。
        在重构后,我们直接传字段而不是 LaserScan 对象 —— 因为 src_map
        构造只需要 angle_min/angle_inc/n_src 这几个量(下面 _build_src_map
        从一个轻量上下文里取值)。
        """
        changed = (
            self._cached_n_src     != n_src or
            self._cached_angle_min != angle_min or
            self._cached_angle_inc != angle_inc
        )
        if not changed:
            return

        if n_src <= 0 or angle_inc <= 0:
            rospy.logwarn_throttle(
                5.0,
                "[pipeline.stage2] pathological scan geometry: "
                "n_src=%d angle_inc=%.6f -> geometry cache NOT rebuilt.",
                n_src, angle_inc,
            )
            return

        old_n = self._cached_n_src
        self._cached_n_src     = n_src
        self._cached_angle_min = angle_min
        self._cached_angle_inc = angle_inc

        self._half_idx_12 = max(
            1, int(math.radians(REPAIR_D_FRAME_12_DEG) / angle_inc))
        self._half_idx_25 = max(
            1, int(math.radians(REPAIR_D_FRAME_25_DEG) / angle_inc))
        self._front_half_idx = max(
            1, int(math.radians(self.front_fov_deg) / angle_inc))
        self._front_center_raw = max(
            0, min(n_src - 1, int(round((-angle_min) / angle_inc))))

        self._src_map, self._src_map_msg = self._build_src_map(
            n_src, angle_min, angle_inc)

        if old_n is not None and old_n != n_src:
            self._time_hist.clear()
            rospy.logwarn(
                "[pipeline.stage2] scan length CHANGED (%s -> %d), "
                "_time_hist cleared.",
                str(old_n), n_src,
            )

        if old_n is None:
            rospy.logwarn(
                "[pipeline.stage2] scan geometry LOCKED on first scan: "
                "n_src=%d  angle_min=%.5f  angle_inc=%.5f  "
                "half_idx_12=%d  half_idx_25=%d  "
                "front_half_idx=%d  front_center_raw=%d",
                n_src, angle_min, angle_inc,
                self._half_idx_12, self._half_idx_25,
                self._front_half_idx, self._front_center_raw,
            )
            preview_f = self._src_map[:5]
            preview_c = self._src_map[self.num_beams:self.num_beams + 5]
            preview_w = self._src_map[2 * self.num_beams:2 * self.num_beams + 5]
            rospy.logwarn(
                "[pipeline.stage2] src_map preview: floor[0:5]=%s  "
                "ceil[0:5]=%s  weight[0:5]=%s",
                ["%d" % int(v) for v in preview_f],
                ["%d" % int(v) for v in preview_c],
                ["%.3f" % v for v in preview_w],
            )
        else:
            rospy.logwarn(
                "[pipeline.stage2] scan geometry CHANGED: "
                "n_src %s->%d  angle_min->%.5f  angle_inc->%.5f  "
                "(src_map + half_idx caches rebuilt)",
                str(old_n), n_src, angle_min, angle_inc,
            )

    # --------------------------------------------------------
    # src_map 构造(与 v3_2 完全一致)
    # --------------------------------------------------------
    def _build_src_map(self, n_src, angle_min, angle_increment):
        floors  = [0.0] * self.num_beams
        ceils   = [0.0] * self.num_beams
        weights = [0.0] * self.num_beams

        floor_int = [0] * self.num_beams
        ceil_int  = [0] * self.num_beams

        for i, angle in enumerate(self.target_angles):
            if self.use_linear_interpolation:
                pos = (angle - angle_min) / angle_increment
                if pos <= 0.0:
                    f = c = 0;          w = 0.0
                elif pos >= n_src - 1:
                    f = c = n_src - 1;  w = 0.0
                else:
                    f = int(math.floor(pos))
                    c = f + 1
                    w = pos - f
            else:
                idx = int(round((angle - angle_min) / angle_increment))
                idx = max(0, min(n_src - 1, idx))
                f = c = idx;            w = 0.0
            floors[i]    = float(f)
            ceils[i]     = float(c)
            weights[i]   = float(w)
            floor_int[i] = f
            ceil_int[i]  = c

        self._src_floor  = floor_int
        self._src_ceil   = ceil_int
        self._src_weight = list(weights)

        src_map = floors + ceils + weights

        msg = Float32MultiArray()
        dim_f = MultiArrayDimension(label="floor",  size=self.num_beams, stride=self.num_beams)
        dim_c = MultiArrayDimension(label="ceil",   size=self.num_beams, stride=self.num_beams)
        dim_w = MultiArrayDimension(label="weight", size=self.num_beams, stride=self.num_beams)
        msg.layout.dim = [dim_f, dim_c, dim_w]
        msg.layout.data_offset = 0
        msg.data = src_map
        return src_map, msg

    # --------------------------------------------------------
    # 主修补函数(与 v3_2 完全一致)
    # --------------------------------------------------------
    def _repair_raw_ranges(self, ranges, cur_range_min, cur_range_max):
        n = len(ranges)
        if n == 0:
            return [], [], []

        is_valid = [self._is_valid_raw(r, cur_range_min, cur_range_max) for r in ranges]
        valid_values = [ranges[i] for i in range(n) if is_valid[i]]
        valid_count = len(valid_values)

        # 边界:整帧全无效
        if valid_count == 0:
            had_hist_before = len(self._time_hist) > 0
            if had_hist_before:
                last = list(self._time_hist[-1])
                if len(last) >= n:
                    repaired = last[:n]
                else:
                    pad = [self.invalid_fill] * (n - len(last))
                    repaired = last + pad
            else:
                repaired = [self.invalid_fill] * n
            raw_repair_mask = [1] * n
            raw_severe_mask = [1] * n
            self._time_hist.append(list(repaired))
            rospy.logwarn_throttle(
                2.0,
                "[pipeline.stage2] entire frame invalid, fell back to %s (n=%d).",
                "last frame" if had_hist_before else "invalid_fill", n,
            )
            return repaired, raw_repair_mask, raw_severe_mask

        if self._half_idx_12 is None or self._half_idx_25 is None:
            rospy.logwarn_throttle(
                5.0,
                "[pipeline.stage2] geometry cache not ready while valid values "
                "exist (unusual); using conservative sector widths.",
            )

        frame_q_global = self._quantile_sorted(
            sorted(valid_values), REPAIR_D_FRAME_GLOBAL_Q)

        repaired        = list(ranges)
        raw_repair_mask = [0] * n
        raw_severe_mask = [0] * n

        for i in range(n):
            if is_valid[i]:
                continue

            d_space, space_status = self._compute_d_space(ranges, is_valid, i, n)
            d_time,  time_ok      = self._compute_d_time(i)
            d_frame               = self._compute_d_frame(
                ranges, is_valid, i, n, frame_q_global)

            ws, wt, wf = self._pick_weights(space_status)

            numerator = 0.0
            denom     = 0.0
            if space_status != 'empty':
                numerator += ws * d_space
                denom     += ws
            if time_ok:
                numerator += wt * d_time
                denom     += wt
            numerator += wf * d_frame
            denom     += wf

            d_fill = (numerator / denom) if denom > 0 else d_frame

            if d_fill < cur_range_min:
                d_fill = cur_range_min
            elif d_fill > cur_range_max:
                d_fill = cur_range_max

            repaired[i] = d_fill
            raw_repair_mask[i] = 1
            if space_status == 'empty':
                raw_severe_mask[i] = 1

        self._time_hist.append(list(repaired))
        return repaired, raw_repair_mask, raw_severe_mask

    # --------------------------------------------------------
    # 181 维重采样(作用于修补后的 ranges)
    # --------------------------------------------------------
    def _resample_single_frame_from_repaired(self, repaired_ranges,
                                             angle_min, angle_increment):
        n_src = len(repaired_ranges)
        if n_src == 0:
            fill = self.invalid_fill / self.max_distance
            return [fill] * self.num_beams
        if angle_increment <= 0:
            fill = self.invalid_fill / self.max_distance
            return [fill] * self.num_beams

        beams = []
        for angle in self.target_angles:
            pos = (angle - angle_min) / angle_increment
            if self.use_linear_interpolation:
                if pos <= 0.0:
                    r = repaired_ranges[0]
                elif pos >= n_src - 1:
                    r = repaired_ranges[-1]
                else:
                    i0 = int(math.floor(pos))
                    i1 = i0 + 1
                    w  = pos - i0
                    r = (1.0 - w) * repaired_ranges[i0] + w * repaired_ranges[i1]
            else:
                idx = int(round(pos))
                idx = max(0, min(n_src - 1, idx))
                r = repaired_ranges[idx]
            beams.append(r / self.max_distance)

        if self.reverse_output:
            beams.reverse()
        return beams

    # --------------------------------------------------------
    # raw_repair_mask -> 181 维 repair_mask
    # --------------------------------------------------------
    def _compute_repair_mask_181(self, raw_repair_mask, raw_severe_mask, n_src):
        if self._src_floor is None:
            return [0] * self.num_beams, [0] * self.num_beams

        mask_181   = [0] * self.num_beams
        severe_181 = [0] * self.num_beams

        for k in range(self.num_beams):
            f = self._src_floor[k]
            c = self._src_ceil[k]
            if f < 0:        f = 0
            elif f >= n_src: f = n_src - 1
            if c < 0:        c = 0
            elif c >= n_src: c = n_src - 1

            if raw_repair_mask[f] or raw_repair_mask[c]:
                mask_181[k] = 1
            if raw_severe_mask[f] or raw_severe_mask[c]:
                severe_181[k] = 1

        if self.reverse_output:
            mask_181.reverse()
            severe_181.reverse()

        return mask_181, severe_181

    # --------------------------------------------------------
    # fill_stats 计算
    # --------------------------------------------------------
    def _compute_fill_stats(self, repair_mask_181, severe_mask_181):
        total = float(self.num_beams)
        repair_count = sum(repair_mask_181)
        repair_ratio = (repair_count / total) if total > 0 else 0.0

        if self.num_beams > 1:
            deg_per_beam = (self.fov_max_deg - self.fov_min_deg) / float(self.num_beams - 1)
        else:
            deg_per_beam = 1.0
        front_half = max(1, int(round(self.front_fov_deg / deg_per_beam)))
        center_181 = self.num_beams // 2

        lo = max(0, center_181 - front_half)
        hi = min(self.num_beams, center_181 + front_half + 1)
        front_repair_count = sum(repair_mask_181[lo:hi])

        severe_count = sum(severe_mask_181)
        severe_ratio = (severe_count / total) if total > 0 else 0.0

        return [
            float(repair_ratio),
            float(repair_count),
            float(front_repair_count),
            float(severe_ratio),
        ]

    # --------------------------------------------------------
    # 主入口
    # --------------------------------------------------------
    def process(self, stage1_out):
        """
        消费 Stage 1 输出,产出 181 维观测 + 修补诊断。
        若 Stage 1 输出为 None,返回 None。
        """
        if stage1_out is None:
            return None

        n_src     = stage1_out['n']
        angle_min = stage1_out['angle_min']
        angle_inc = stage1_out['angle_increment']

        # 几何缓存维护
        self._update_scan_geometry_cache(n_src, angle_min, angle_inc)

        # 本帧融合量程(与 Stage 1 同语义)
        scan_range_min = stage1_out['range_min']
        scan_range_max = stage1_out['range_max']
        cur_range_min = max(self.range_min, float(scan_range_min))
        if scan_range_max is not None and scan_range_max > 0.0:
            cur_range_max = min(self.max_distance, float(scan_range_max))
        else:
            cur_range_max = self.max_distance
        if cur_range_max <= cur_range_min:
            cur_range_min = self.range_min
            cur_range_max = self.max_distance

        # Step 1: 原始米制空间动态修补
        repaired_ranges, raw_repair_mask, raw_severe_mask = self._repair_raw_ranges(
            list(stage1_out['aligned_ranges']), cur_range_min, cur_range_max)

        # Step 2: 用修补后的 ranges 做 181 维重采样 + 归一化
        lidar_181 = self._resample_single_frame_from_repaired(
            repaired_ranges, angle_min, angle_inc)

        # Step 3: raw_repair_mask -> 181 维 mask
        if n_src > 0:
            repair_mask_181, severe_mask_181 = self._compute_repair_mask_181(
                raw_repair_mask, raw_severe_mask, n_src)
        else:
            repair_mask_181 = [1] * self.num_beams
            severe_mask_181 = [1] * self.num_beams

        # Step 4: fill_stats
        fill_stats = self._compute_fill_stats(repair_mask_181, severe_mask_181)

        self.scan_count += 1

        return {
            'lidar_181':       lidar_181,
            'repair_mask_181': repair_mask_181,
            'severe_mask_181': severe_mask_181,
            'fill_stats':      fill_stats,
            'src_map':         self._src_map,
            'src_map_msg':     self._src_map_msg,
            'cur_range_min':   cur_range_min,
            'cur_range_max':   cur_range_max,
            'n_src':           n_src,
            'header':          stage1_out['header'],   # = scan.header,沿用统一时间戳
        }


# ============================================================
# Stage 3 Module : 三帧滑窗 + median3 时域稳态 + 543 堆叠
# ------------------------------------------------------------
# 与 lidar_stack_543 v2_1 的算法完全一致。**关键变化**:
#   - 不再有 _stamp_queue / _mask_queue / _fs_queue 三个 FIFO
#   - cur_stamp / cur_mask / cur_fs 由调用者 (LidarPipelineNode)
#     从同一帧的 Stage 2 输出直接传入,严格同帧对齐
#   - 因此 stamp_fallback_frames / stab_skipped_frames 在单节点
#     模式下永远不会增长(逻辑保留作为防御性兜底,便于将来重新
#     拆分时复用)
# ============================================================
class Stage3Module(object):
    def __init__(self, params):
        self.expected_len            = int (params.get('expected_len',           LIDAR_NUM_BEAMS))
        self.enable_repair_stabilize = bool(params.get('enable_repair_stabilize', True))

        # 滑窗状态
        self.t0       = None;  self.t1       = None;  self.t2       = None
        self.t0_id    = None;  self.t1_id    = None;  self.t2_id    = None
        self.t0_stamp = None;  self.t1_stamp = None;  self.t2_stamp = None

        self.msg_count        = 0
        self.frame_id_counter = 0

        # 统计计数器(保持 v2_1 兼容)
        self.stab_applied_frames   = 0
        self.stab_skipped_frames   = 0   # 单节点模式下应恒为 0
        self.stamp_fallback_frames = 0   # 单节点模式下应恒为 0
        self.stamp_nonmono_frames  = 0   # 仅当 /scan stamp 自身回退时才会触发

    # --------------------------------------------------------
    # 工具
    # --------------------------------------------------------
    def _validate_frame(self, data):
        if len(data) != self.expected_len:
            rospy.logerr_throttle(
                1.0,
                "[pipeline.stage3] input length error: got %d, expected %d",
                len(data), self.expected_len,
            )
            return False
        return True

    @staticmethod
    def _median3(a, b, c):
        """3 个 float 的中位数(纯比较,无排序分配)。"""
        if a > b: a, b = b, a
        if b > c: b, c = c, b
        if a > b: a, b = b, a
        return b

    # --------------------------------------------------------
    # 外部 reset(从静止重启时可调用)
    # --------------------------------------------------------
    def reset(self):
        if self.t0 is not None:
            self.t1       = list(self.t0);  self.t1_id    = self.t0_id
            self.t2       = list(self.t0);  self.t2_id    = self.t0_id
            self.t1_stamp = self.t0_stamp
            self.t2_stamp = self.t0_stamp
            rospy.logwarn(
                "[pipeline.stage3] reset: refilled t1/t2 with current t0, "
                "ids=[%s,%s,%s]",
                str(self.t0_id), str(self.t1_id), str(self.t2_id),
            )

    # --------------------------------------------------------
    # 主入口
    # --------------------------------------------------------
    def process(self, stage2_out, cur_stamp):
        """
        消费 Stage 2 输出,产出 543 堆叠 + 三帧诊断。
        若 Stage 2 输出为 None 或长度不合法,返回 None。
        """
        if stage2_out is None:
            return None

        cur      = list(stage2_out['lidar_181'])
        cur_mask = stage2_out['repair_mask_181']     # 必然存在,与 cur 同帧
        cur_fs   = stage2_out['fill_stats']

        if not self._validate_frame(cur):
            return None

        self.frame_id_counter += 1
        cur_id = self.frame_id_counter

        # -------- 滑窗滚动 + 时域稳态 --------
        if self.t0 is None:
            # 首帧:用 cur 填满整个滑窗,不做稳态
            self.t0 = list(cur);  self.t1 = list(cur);  self.t2 = list(cur)
            self.t0_id = cur_id;  self.t1_id = cur_id;  self.t2_id = cur_id
            self.t0_stamp = cur_stamp
            self.t1_stamp = cur_stamp
            self.t2_stamp = cur_stamp
        else:
            # 先把 t1/t2 向后滚,为新 t0 腾位
            self.t2       = self.t1;        self.t2_id    = self.t1_id
            self.t2_stamp = self.t1_stamp
            self.t1       = self.t0;        self.t1_id    = self.t0_id
            self.t1_stamp = self.t0_stamp

            # 计算新 t0(对修补 beam 做三帧中位数稳态)
            new_t0 = list(cur)
            if (self.enable_repair_stabilize
                    and cur_mask is not None
                    and len(cur_mask) == self.expected_len):
                stab_count = 0
                for k in range(self.expected_len):
                    if cur_mask[k] == 1:
                        new_t0[k] = self._median3(
                            cur[k], self.t1[k], self.t2[k])
                        stab_count += 1
                if stab_count > 0:
                    self.stab_applied_frames += 1
            else:
                # 单节点模式下基本不会进这里;保留计数以便诊断
                if self.msg_count >= 3:
                    self.stab_skipped_frames += 1

            self.t0       = new_t0
            self.t0_id    = cur_id
            self.t0_stamp = cur_stamp

        # -------- 主输出 [t0 | t1 | t2] --------
        stacked = self.t0 + self.t1 + self.t2

        # -------- 诊断输出 --------
        ids = [float(self.t0_id), float(self.t1_id), float(self.t2_id)]
        ids_vis = (
            [float(self.t0_id)] * self.expected_len +
            [float(self.t1_id)] * self.expected_len +
            [float(self.t2_id)] * self.expected_len
        )
        stamps_sec = [
            float(self.t0_stamp.to_sec()),
            float(self.t1_stamp.to_sec()),
            float(self.t2_stamp.to_sec()),
        ]

        # 单调性自检(单节点模式下,只有 /scan 自身 stamp 回退才会触发)
        if not (stamps_sec[0] >= stamps_sec[1] >= stamps_sec[2]):
            self.stamp_nonmono_frames += 1
            rospy.logwarn_throttle(
                1.0,
                "[pipeline.stage3] non-monotonic stamps: "
                "[t0=%.6f, t1=%.6f, t2=%.6f]  (dt01=%+.4f, dt12=%+.4f)  "
                "total_nonmono=%d  (single-node mode: this means /scan stamps "
                "themselves are out of order)",
                stamps_sec[0], stamps_sec[1], stamps_sec[2],
                stamps_sec[0] - stamps_sec[1],
                stamps_sec[1] - stamps_sec[2],
                self.stamp_nonmono_frames,
            )

        self.msg_count += 1

        return {
            'lidar_543':   stacked,
            'ids':         ids,
            'ids_vis':     ids_vis,
            'stamps_sec':  stamps_sec,
            'cur_id':      cur_id,
            'cur_mask':    cur_mask,
            'cur_fs':      cur_fs,
            't0_id':       self.t0_id,
            't1_id':       self.t1_id,
            't2_id':       self.t2_id,
        }


# ============================================================
# 顶层 ROS 节点 : 唯一的订阅者 + 全部发布者
# ------------------------------------------------------------
# 关键约束:
#   - 唯一订阅 /scan
#   - 三阶段在 scan_callback 内一次性完成,严格同帧
#   - 时间戳来源唯一 = scan.header.stamp
#   - 所有原诊断 topic 完整保留(便于 rl_planner_node / 调试工具
#     无感切换)
# ============================================================
class LidarPipelineNode(object):
    def __init__(self):
        # ===== Stage 1 话题与参数 =====
        self.input_topic         = rospy.get_param("~input_scan_topic",     "/scan")
        self.scan_aligned_topic  = rospy.get_param("~output_scan_topic",    "/scan_aligned")
        self.output_frame_id     = rospy.get_param("~output_frame_id",      "laser_link_aligned")
        self.invalid_mask_topic  = rospy.get_param("~invalid_mask_topic",   "/scan_aligned_invalid_mask")
        self.invalid_stats_topic = rospy.get_param("~invalid_stats_topic",  "/scan_aligned_invalid_stats")

        # ===== Stage 2 话题与参数 =====
        self.lidar_181_topic   = rospy.get_param("~lidar_181_topic",        "/rl/lidar_181")
        self.stamp_topic       = rospy.get_param("~stamp_topic",            "/rl/lidar_181_stamp")
        self.src_map_topic     = rospy.get_param("~src_map_topic",          "/rl/lidar_181_src_map")
        self.repair_mask_topic = rospy.get_param("~repair_mask_topic",      "/rl/lidar_181_repair_mask")
        self.fill_stats_topic  = rospy.get_param("~fill_stats_topic",       "/rl/lidar_181_fill_stats")

        # ===== Stage 3 话题与参数 =====
        self.lidar_543_topic   = rospy.get_param("~lidar_543_topic",        "/rl/lidar_543")
        self.ids_topic         = rospy.get_param("~ids_topic",              "/rl/lidar_543_ids")
        self.ids_vis_topic     = rospy.get_param("~ids_vis_topic",          "/rl/lidar_543_ids_vis")
        self.stamps_out_topic  = rospy.get_param("~stamps_out_topic",       "/rl/lidar_543_stamps")

        # ===== 行为参数 =====
        self.publish_legacy_diag = bool(rospy.get_param("~publish_legacy_diag", True))
        self.publish_src_map     = bool(rospy.get_param("~publish_src_map",    True))
        self.publish_debug_log   = bool(rospy.get_param("~publish_debug_log",  True))
        self.verbose             = bool(rospy.get_param("~verbose",            False))
        self.stats_every         = int (rospy.get_param("~stats_every",        200))
        self.sub_queue_size      = int (rospy.get_param("~sub_queue_size",     50))
        self.pub_queue_size      = int (rospy.get_param("~pub_queue_size",     50))
        self.tcp_nodelay         = bool(rospy.get_param("~tcp_nodelay",        True))

        # ===== 三个阶段模块 =====
        self.stage1 = Stage1Module({
            'front_fov_deg': rospy.get_param("~invalid_front_fov_deg", INVALID_FRONT_FOV_DEG),
        })
        self.stage2 = Stage2Module({
            'fov_min_deg':              rospy.get_param("~fov_min_deg",     LIDAR_FOV_MIN_DEG),
            'fov_max_deg':              rospy.get_param("~fov_max_deg",     LIDAR_FOV_MAX_DEG),
            'num_beams':                rospy.get_param("~num_beams",       LIDAR_NUM_BEAMS),
            'max_distance':             rospy.get_param("~max_distance",    LIDAR_MAX_DISTANCE),
            'range_min':                rospy.get_param("~range_min",       LIDAR_RANGE_MIN),
            'invalid_fill':             rospy.get_param("~invalid_fill_value",
                                                         LIDAR_MAX_DISTANCE * 0.5),
            'use_linear_interpolation': rospy.get_param("~use_linear_interpolation", True),
            'reverse_output':           rospy.get_param("~reverse_output",  False),
            'hist_len':                 rospy.get_param("~repair_hist_len", REPAIR_HIST_LEN),
            'space_neighbors':          rospy.get_param("~repair_space_neighbors",
                                                         REPAIR_SPACE_NEIGHBORS),
            'front_fov_deg':            rospy.get_param("~repair_front_fov_deg",
                                                         REPAIR_FRONT_FOV_DEG),
        })
        self.stage3 = Stage3Module({
            'expected_len':            rospy.get_param("~expected_len",            LIDAR_NUM_BEAMS),
            'enable_repair_stabilize': rospy.get_param("~enable_repair_stabilize", True),
        })

        # ===== Lock(用于 reset() 与 callback 互斥) =====
        self.lock = threading.Lock()

        # ===== Publishers =====
        # Stage 1
        self.pub_aligned       = rospy.Publisher(
            self.scan_aligned_topic, LaserScan, queue_size=self.pub_queue_size)
        self.pub_invalid_mask  = rospy.Publisher(
            self.invalid_mask_topic, Float32MultiArray, queue_size=self.pub_queue_size)
        self.pub_invalid_stats = rospy.Publisher(
            self.invalid_stats_topic, Float32MultiArray, queue_size=self.pub_queue_size)
        # Stage 2
        self.pub_stamp         = rospy.Publisher(
            self.stamp_topic,       Header,            queue_size=20)
        self.pub_src_map       = rospy.Publisher(
            self.src_map_topic,     Float32MultiArray, queue_size=20)
        self.pub_repair_mask   = rospy.Publisher(
            self.repair_mask_topic, Float32MultiArray, queue_size=20)
        self.pub_fill_stats    = rospy.Publisher(
            self.fill_stats_topic,  Float32MultiArray, queue_size=20)
        self.pub_lidar_181     = rospy.Publisher(
            self.lidar_181_topic,   Float32MultiArray, queue_size=20)
        # Stage 3
        self.pub_lidar_543     = rospy.Publisher(
            self.lidar_543_topic,   Float32MultiArray, queue_size=10)
        self.pub_ids           = rospy.Publisher(
            self.ids_topic,         Float32MultiArray, queue_size=10)
        self.pub_ids_vis       = rospy.Publisher(
            self.ids_vis_topic,     Float32MultiArray, queue_size=10)
        self.pub_stamps        = rospy.Publisher(
            self.stamps_out_topic,  Float64MultiArray, queue_size=10)

        # ===== Subscriber(唯一,挂在 /scan 上)=====
        self.sub = rospy.Subscriber(
            self.input_topic, LaserScan, self.scan_callback,
            queue_size=self.sub_queue_size, tcp_nodelay=self.tcp_nodelay,
        )

        self._print_banner()

    # --------------------------------------------------------
    # 启动横幅
    # --------------------------------------------------------
    def _print_banner(self):
        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_pipeline_node v1] STARTED — single-node multi-stage")
        rospy.logwarn("  /scan -> Stage1 -> Stage2 -> Stage3 -> /rl/lidar_543")
        rospy.logwarn("  -- Stage 1 (laser_switch v3_1) --")
        rospy.logwarn("    input_scan_topic         : %s", self.input_topic)
        rospy.logwarn("    scan_aligned_topic       : %s", self.scan_aligned_topic)
        rospy.logwarn("    output_frame_id          : %s", self.output_frame_id)
        rospy.logwarn("    invalid_mask_topic       : %s", self.invalid_mask_topic)
        rospy.logwarn("    invalid_stats_topic      : %s", self.invalid_stats_topic)
        rospy.logwarn("  -- Stage 2 (lidar_obs_single_frame v3_2) --")
        rospy.logwarn("    lidar_181_topic          : %s", self.lidar_181_topic)
        rospy.logwarn("    stamp_topic              : %s", self.stamp_topic)
        rospy.logwarn("    src_map_topic            : %s", self.src_map_topic)
        rospy.logwarn("    repair_mask_topic        : %s", self.repair_mask_topic)
        rospy.logwarn("    fill_stats_topic         : %s", self.fill_stats_topic)
        rospy.logwarn("    num_beams=%d  fov=[%.1f,%.1f]  max_distance=%.2f  range_min=%.3f",
                      self.stage2.num_beams, self.stage2.fov_min_deg, self.stage2.fov_max_deg,
                      self.stage2.max_distance, self.stage2.range_min)
        rospy.logwarn("    repair: hist_len=%d  neighbors=%d  front_fov=%.1f°",
                      self.stage2.hist_len, self.stage2.space_neighbors,
                      self.stage2.front_fov_deg)
        rospy.logwarn("    weights: full=%s  partial=%s  empty=%s",
                      str(REPAIR_W_SPACE_FULL), str(REPAIR_W_SPACE_PARTIAL),
                      str(REPAIR_W_SPACE_EMPTY))
        rospy.logwarn("  -- Stage 3 (lidar_stack_543 v2_1) --")
        rospy.logwarn("    lidar_543_topic          : %s (len=%d)",
                      self.lidar_543_topic, LIDAR_STACKED_DIM)
        rospy.logwarn("    ids_topic                : %s", self.ids_topic)
        rospy.logwarn("    ids_vis_topic            : %s", self.ids_vis_topic)
        rospy.logwarn("    stamps_out_topic         : %s (len=3, Float64)",
                      self.stamps_out_topic)
        rospy.logwarn("    expected_len=%d  enable_repair_stabilize=%s",
                      self.stage3.expected_len,
                      str(self.stage3.enable_repair_stabilize))
        rospy.logwarn("  -- Common --")
        rospy.logwarn("    publish_legacy_diag      : %s", str(self.publish_legacy_diag))
        rospy.logwarn("    publish_src_map          : %s", str(self.publish_src_map))
        rospy.logwarn("    sub_queue/pub_queue      : %d / %d",
                      self.sub_queue_size, self.pub_queue_size)
        rospy.logwarn("    tcp_nodelay              : %s", str(self.tcp_nodelay))
        rospy.logwarn("    stats_every              : %d frames", self.stats_every)
        rospy.logwarn("======================================")

    # --------------------------------------------------------
    # 主 callback —— 同一帧 scan 在此走完三个阶段
    # --------------------------------------------------------
    def scan_callback(self, scan_msg):
        with self.lock:
            # ============ Stage 1 ============
            s1 = self.stage1.process(scan_msg)
            if s1 is None:
                return
            self._publish_stage1(s1)

            # ============ Stage 2 ============
            s2 = self.stage2.process(s1)
            if s2 is None:
                return
            self._publish_stage2(s1, s2)

            # ============ Stage 3 ============
            # **唯一的时间戳来源**:scan.header.stamp。
            # 不再使用 rospy.Time.now() / topic stamp fallback。
            cur_stamp = scan_msg.header.stamp
            s3 = self.stage3.process(s2, cur_stamp)
            if s3 is None:
                return
            self._publish_stage3(s3)

            # ============ 周期性 stats 日志 ============
            self._maybe_log_stats(s1, s2, s3)

    # --------------------------------------------------------
    # 发布 Stage 1 的 3 个话题(顺序与 v3_1 一致)
    # --------------------------------------------------------
    def _publish_stage1(self, s1):
        n = s1['n']

        out = LaserScan()
        out.header = s1['header']
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.angle_min       = s1['angle_min']
        out.angle_max       = s1['angle_max']
        out.angle_increment = s1['angle_increment']
        out.time_increment  = s1['time_increment']
        out.scan_time       = s1['scan_time']
        out.range_min       = s1['range_min']
        out.range_max       = s1['range_max']
        out.ranges          = s1['aligned_ranges']
        out.intensities     = s1['aligned_intensities']
        self.pub_aligned.publish(out)

        if self.publish_legacy_diag:
            mask_f = [float(v) for v in s1['invalid_mask']]
            self.pub_invalid_mask.publish(
                _build_f32_msg(mask_f, "scan_aligned_invalid_mask"))
            self.pub_invalid_stats.publish(
                _build_f32_msg(s1['invalid_stats'], "scan_aligned_invalid_stats"))

        # verbose 模式:逐帧 0/-180/+180 比对(与 v3_1 同语义)
        if self.verbose:
            inc = s1['angle_increment']
            if inc > 0:
                idx_0 = int(round((0.0 - s1['angle_min']) / inc))
                idx_0 = max(0, min(n - 1, idx_0))
            else:
                idx_0 = n // 2
            old_ranges = s1['old_ranges']
            new_ranges = s1['aligned_ranges']
            inv_count = int(s1['invalid_stats'][0])
            front_inv = int(s1['invalid_stats'][2])
            rospy.logwarn_throttle(
                1.0,
                "[pipeline.stage1] raw: 0deg=%s -180deg=%s +180deg=%s | "
                "aligned: 0deg=%s -180deg=%s +180deg=%s | inv=%d/%d front_inv=%d",
                Stage1Module.safe_value_str(old_ranges[idx_0]),
                Stage1Module.safe_value_str(old_ranges[0]),
                Stage1Module.safe_value_str(old_ranges[n - 1]),
                Stage1Module.safe_value_str(new_ranges[idx_0]),
                Stage1Module.safe_value_str(new_ranges[0]),
                Stage1Module.safe_value_str(new_ranges[n - 1]),
                inv_count, n, front_inv,
            )

    # --------------------------------------------------------
    # 发布 Stage 2 的 5 个话题(顺序与 v3_2 一致:
    #   stamp -> src_map -> repair_mask -> lidar_181 -> fill_stats)
    # 在单节点模式下顺序对下游已无影响,但保持以兼容外部观测工具。
    # --------------------------------------------------------
    def _publish_stage2(self, s1, s2):
        # 1) stamp(沿用 scan.header.stamp + frame_id)
        stamp_msg = Header()
        stamp_msg.stamp    = s1['header'].stamp
        stamp_msg.frame_id = s1['header'].frame_id
        self.pub_stamp.publish(stamp_msg)

        # 2) src_map(可选)
        if self.publish_src_map and s2['src_map_msg'] is not None:
            self.pub_src_map.publish(s2['src_map_msg'])

        # 3) repair_mask
        if self.publish_legacy_diag:
            self.pub_repair_mask.publish(_build_f32_msg(
                [float(m) for m in s2['repair_mask_181']], "lidar_181_repair_mask"))

        # 4) lidar_181 (主输出)
        self.pub_lidar_181.publish(_build_f32_msg(s2['lidar_181'], "lidar_181"))

        # 5) fill_stats
        if self.publish_legacy_diag:
            self.pub_fill_stats.publish(_build_f32_msg(
                s2['fill_stats'], "lidar_181_fill_stats"))

    # --------------------------------------------------------
    # 发布 Stage 3 的 4 个话题
    # --------------------------------------------------------
    def _publish_stage3(self, s3):
        # 1) lidar_543 (主输出)
        self.pub_lidar_543.publish(_build_f32_msg(s3['lidar_543'], "lidar_543"))
        # 2/3/4) 诊断
        if self.publish_legacy_diag:
            self.pub_ids.publish(_build_f32_msg(s3['ids'], "lidar_543_ids"))
            self.pub_ids_vis.publish(_build_f32_msg(s3['ids_vis'], "lidar_543_ids_vis"))
            self.pub_stamps.publish(_build_f64_msg(s3['stamps_sec'], "lidar_543_stamps"))

    # --------------------------------------------------------
    # 周期性日志(合并原 3 节点的 stats 日志)
    # --------------------------------------------------------
    def _maybe_log_stats(self, s1, s2, s3):
        if not self.publish_debug_log:
            return
        n_frames = self.stage3.msg_count
        if self.stats_every <= 0:
            return

        # Stage 1 风格的 stats(每 stats_every 帧)
        if self.stage1.recv_count % self.stats_every == 0:
            inv_ratio   = s1['invalid_stats'][1]
            front_inv_c = int(s1['invalid_stats'][2])
            rospy.logwarn(
                "[pipeline.stage1.stats] recv=%d  empty=%d  "
                "last_invalid_ratio=%.3f  front_invalid=%d",
                self.stage1.recv_count, self.stage1.empty_count,
                inv_ratio, front_inv_c,
            )

        # Stage 2 风格的 stats(每 20 帧,与原节点一致)
        if self.stage2.scan_count % 20 == 1:
            lidar_181 = s2['lidar_181']
            center_idx = len(lidar_181) // 2 if lidar_181 else 0
            fs = s2['fill_stats']
            rospy.logwarn(
                "[pipeline.stage2] seq=%d  raw_n=%d -> out_n=%d  range=[%.3f,%.3f]  "
                "min=%.4f max=%.4f center=%.4f  "
                "repair_ratio=%.3f count=%d front=%d severe=%.3f",
                self.stage2.scan_count, s2['n_src'], len(lidar_181),
                s2['cur_range_min'], s2['cur_range_max'],
                min(lidar_181) if lidar_181 else -1.0,
                max(lidar_181) if lidar_181 else -1.0,
                lidar_181[center_idx] if lidar_181 else -1.0,
                fs[0], int(fs[1]), int(fs[2]), fs[3],
            )

        # Stage 3 风格的 stats(每 20 帧,与原节点一致)
        if n_frames % 20 == 1:
            cur_mask = s3['cur_mask']
            mask_count = sum(cur_mask) if cur_mask is not None else -1
            cur_fs = s3['cur_fs']
            if cur_fs is not None:
                fs_desc = ("repair_ratio=%.3f front=%d severe=%.3f"
                           % (cur_fs[0], int(cur_fs[2]), cur_fs[3]))
            else:
                fs_desc = "fs=none"
            rospy.logwarn(
                "[pipeline.stage3] recv #%d  ids=[%d,%d,%d]  "
                "stamps=[%.3f,%.3f,%.3f]  mask_count=%d  %s  "
                "stab_applied=%d  stab_skipped=%d  "
                "stamp_fallback=%d  stamp_nonmono=%d",
                s3['cur_id'], s3['t0_id'], s3['t1_id'], s3['t2_id'],
                s3['stamps_sec'][0], s3['stamps_sec'][1], s3['stamps_sec'][2],
                mask_count, fs_desc,
                self.stage3.stab_applied_frames,
                self.stage3.stab_skipped_frames,
                self.stage3.stamp_fallback_frames,
                self.stage3.stamp_nonmono_frames,
            )

    # --------------------------------------------------------
    # 外部 reset(可被 service / timer 调用)
    # --------------------------------------------------------
    def reset(self):
        with self.lock:
            self.stage3.reset()


def main():
    rospy.init_node("lidar_pipeline_node", anonymous=False)
    LidarPipelineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
