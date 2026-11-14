#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 2/3 : lidar_obs_single_frame  (v3)
============================================================
职责:把方向校正后的 LaserScan 编码为策略网络可直接消费的
     181 维归一化单帧观测,并承担全链路中"无效量测动态修补"的主责任。

输入 : /scan_aligned             (sensor_msgs/LaserScan)

主输出 : /rl/lidar_181            (std_msgs/Float32MultiArray, len = 181)

伴随输出(用于跨节点诊断):
  /rl/lidar_181_stamp            (std_msgs/Header)
  /rl/lidar_181_src_map          (std_msgs/Float32MultiArray, len = 543)

v3 新增诊断输出(对应《总体修改意见》第 2 级第 4 条):
  /rl/lidar_181_repair_mask      (std_msgs/Float32MultiArray, len = 181)
      每个 beam 标记是否由修补得到:原始有效 0 / 修补 1。
      映射关系:只要该 beam 的 floor 或 ceil 源点被修补过,则标记 1。

  /rl/lidar_181_fill_stats       (std_msgs/Float32MultiArray, len = 4)
      布局:[repair_ratio, repair_count, front_repair_count, severe_repair_ratio]
        - repair_ratio        : 181 维中修补 beam 占比
        - repair_count        : 修补 beam 数量
        - front_repair_count  : 正前方 ±front_fov_deg (默认 20°) 内修补 beam 数
        - severe_repair_ratio : 无空间邻域信息、主要依赖历史/分位数兜底的 beam 占比

---------------------------------------------------------------------
处理流水(v3 重写,与 v2 的"先打 0 再插值"彻底区分):
  1) 接收 /scan_aligned 的原始 ranges (米制空间)
  2) 逐点有效性判断   : inf / nan / 负值 / 超量程 / 低于 range_min -> 无效
  3) 无效点动态修补   : 在原始米制空间、原始 scan 分辨率上做
                        d_space + d_time + d_frame 加权融合
  4) 重采样到 181 维  : 用修补后的 ranges 做线性插值
  5) 归一化            : value / max_distance  -> 输出范围 [0, 1]
  6) 181 维修补掩码   : 通过 src_map(floor/ceil)将 raw_repair_mask 映射
                        到 181 维,同时计算 fill_stats

---------------------------------------------------------------------
动态修补框架(文档第三节(二)第 3 条):

  d_fill(i, t) = clip( w_s * d_space + w_t * d_time + w_f * d_frame,
                       range_min, max_distance )

  其中:
    d_space  = 当前帧左右邻域的中位数边界插值
    d_time   = 最近 3 帧同一原始索引位置的历史值中位数
    d_frame  = 当前帧局部扇区(±12°->±25°->全帧)有效值的 30%/35% 分位数

  权重(按当前可用信息动态选择):
    邻域完整     : (w_s, w_t, w_f) = (0.60, 0.25, 0.15)
    邻域不完整   : (w_s, w_t, w_f) = (0.35, 0.35, 0.30)
    邻域几乎无   : (w_s, w_t, w_f) = (0.10, 0.40, 0.50)
  不可用分量(如邻域空、历史空)会从分子/分母中剔除并重新归一。

---------------------------------------------------------------------
链路统一常量(v2 -> v3 保持一致):
  - LIDAR_MAX_DISTANCE = 16.0 m
  - LIDAR_NUM_BEAMS    = 181
  - LIDAR_FOV_DEG      = [-135, 135]

注意:v2 中 INVALID_FILL_VALUE = 0.0 的"统一置 0"策略在 v3 中废除。
     本节点已改为动态修补,invalid_fill 仅保留用于极端退化(如 scan 完全为空且
     无任何历史)时的最终兜底路径,默认值改为 max_distance/2 而非 0.0。

CHANGELOG v2 -> v3:
  [核心] _sanitize_range() 废除,不再把无效值统一变成 0 m
  [核心] 新增 _repair_raw_ranges(),在原始米制空间做动态修补
  [核心] 新增 _compute_d_space / _compute_d_time / _compute_d_frame 子程序
  [核心] 新增 _time_hist (deque maxlen=3) 维护原始索引位置的历史值
  [核心] _resample_single_frame 流程改为"先修补后采样"
  [新增] 新增话题 /rl/lidar_181_repair_mask 、/rl/lidar_181_fill_stats
  [新增] _build_src_map 缓存 floor/ceil/weight 数组,供 repair_mask 映射
  [保留] /rl/lidar_181_stamp 、/rl/lidar_181_src_map 原样保留

管线定位:LiDAR 观测预处理链第 2 级,上游 laser_switch,下游 lidar_stack_543。
============================================================
"""

from __future__ import division, print_function

import math
import threading
from collections import deque

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header


# ============================================================
# 链路统一常量 —— 与 lidar_stack_543 / rl_planner_node 对齐
# ============================================================
LIDAR_MAX_DISTANCE = 16.0      # 感知量程上限(m),归一化分母
LIDAR_NUM_BEAMS    = 181       # 单帧观测维度
LIDAR_FOV_MIN_DEG  = -135.0
LIDAR_FOV_MAX_DEG  = 135.0
LIDAR_RANGE_MIN    = 0.05      # 小于该值视为无效(传感器盲区/异常返回)

# ============================================================
# 动态修补配置(v3 新增)
# ============================================================
REPAIR_HIST_LEN          = 3    # 时域历史长度 K
REPAIR_SPACE_NEIGHBORS   = 6    # 单侧最多取用的有效邻居数
REPAIR_FRONT_FOV_DEG     = 20.0 # front_repair_count 统计的正前方扇区半宽
REPAIR_D_FRAME_12_DEG    = 12.0 # d_frame 一级局部扇区半宽
REPAIR_D_FRAME_25_DEG    = 25.0 # d_frame 二级局部扇区半宽
REPAIR_D_FRAME_LOCAL_Q   = 0.30 # 局部扇区分位数
REPAIR_D_FRAME_GLOBAL_Q  = 0.35 # 全帧兜底分位数
REPAIR_D_FRAME_LOCAL_MIN = 5    # 一级扇区最少有效点数
REPAIR_D_FRAME_WIDEN_MIN = 3    # 二级扇区最少有效点数

# 权重(w_s, w_t, w_f)
REPAIR_W_SPACE_FULL    = (0.60, 0.25, 0.15)  # 两侧都有邻居
REPAIR_W_SPACE_PARTIAL = (0.35, 0.35, 0.30)  # 只有一侧有邻居
REPAIR_W_SPACE_EMPTY   = (0.10, 0.40, 0.50)  # 两侧都没有邻居(严重情况)


class LidarObsSingleFrameNode(object):
    def __init__(self):
        # ------------- 话题 -------------
        self.scan_topic        = rospy.get_param("~scan_topic",        "/scan_aligned")
        self.output_topic      = rospy.get_param("~output_topic",      "/rl/lidar_181")
        self.stamp_topic       = rospy.get_param("~stamp_topic",       "/rl/lidar_181_stamp")
        self.src_map_topic     = rospy.get_param("~src_map_topic",     "/rl/lidar_181_src_map")
        # v3 新增
        self.repair_mask_topic = rospy.get_param("~repair_mask_topic", "/rl/lidar_181_repair_mask")
        self.fill_stats_topic  = rospy.get_param("~fill_stats_topic",  "/rl/lidar_181_fill_stats")

        # ------------- 链路统一参数 -------------
        self.fov_min_deg  = float(rospy.get_param("~fov_min_deg",    LIDAR_FOV_MIN_DEG))
        self.fov_max_deg  = float(rospy.get_param("~fov_max_deg",    LIDAR_FOV_MAX_DEG))
        self.num_beams    = int  (rospy.get_param("~num_beams",      LIDAR_NUM_BEAMS))
        self.max_distance = float(rospy.get_param("~max_distance",   LIDAR_MAX_DISTANCE))
        self.range_min    = float(rospy.get_param("~range_min",      LIDAR_RANGE_MIN))
        # v3: invalid_fill 仅用于"帧完全空且无历史"的终极兜底;不再用于无效点修补
        self.invalid_fill = float(rospy.get_param("~invalid_fill_value",
                                                  LIDAR_MAX_DISTANCE * 0.5))

        # ------------- 处理选项 -------------
        self.use_linear_interpolation = bool(rospy.get_param("~use_linear_interpolation", True))
        self.reverse_output           = bool(rospy.get_param("~reverse_output", False))
        self.publish_debug_info       = bool(rospy.get_param("~publish_debug_info", True))
        self.publish_src_map          = bool(rospy.get_param("~publish_src_map", True))

        # ------------- 修补配置 -------------
        self.hist_len        = int  (rospy.get_param("~repair_hist_len",       REPAIR_HIST_LEN))
        self.space_neighbors = int  (rospy.get_param("~repair_space_neighbors",REPAIR_SPACE_NEIGHBORS))
        self.front_fov_deg   = float(rospy.get_param("~repair_front_fov_deg",  REPAIR_FRONT_FOV_DEG))

        # ------------- 内部状态 -------------
        self.lock = threading.Lock()
        self.scan_count = 0
        self.target_angles = self._build_target_angles_rad(
            self.fov_min_deg, self.fov_max_deg, self.num_beams
        )

        # src_map 缓存(第一帧到达时算定)
        self._src_map        = None   # list[float] len=3*num_beams
        self._src_map_msg    = None
        self._src_floor      = None   # list[int]   len=num_beams
        self._src_ceil       = None   # list[int]   len=num_beams
        self._src_weight     = None   # list[float] len=num_beams

        # 修补相关的 scan 元数据缓存(第一帧到达时算定)
        self._half_idx_12    = None
        self._half_idx_25    = None
        self._front_half_idx = None   # 仅用于 raw 域;181 维前向用另一套(见 _compute_fill_stats)
        self._front_center_raw = None

        # 时域历史:每个元素是一帧修补后的 ranges(全部有效)
        self._time_hist = deque(maxlen=self.hist_len)

        # ------------- 发布 / 订阅 -------------
        # 先建 stamp / src_map / repair_mask / fill_stats 的 publisher,
        # 最后建主数据 publisher,保证启动后首帧发送顺序稳定。
        self.pub_stamp       = rospy.Publisher(self.stamp_topic,       Header,            queue_size=20)
        self.pub_src_map     = rospy.Publisher(self.src_map_topic,     Float32MultiArray, queue_size=20)
        self.pub_repair_mask = rospy.Publisher(self.repair_mask_topic, Float32MultiArray, queue_size=20)
        self.pub_fill_stats  = rospy.Publisher(self.fill_stats_topic,  Float32MultiArray, queue_size=20)
        self.pub             = rospy.Publisher(self.output_topic,      Float32MultiArray, queue_size=20)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan,
                                    self.scan_callback,
                                    queue_size=5,
                                    tcp_nodelay=True)

        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_obs_single_frame v3] started (pipeline stage 2/3)")
        rospy.logwarn("  scan_topic         = %s", self.scan_topic)
        rospy.logwarn("  output_topic       = %s", self.output_topic)
        rospy.logwarn("  stamp_topic        = %s", self.stamp_topic)
        rospy.logwarn("  src_map_topic      = %s (len=%d)",
                      self.src_map_topic, 3 * self.num_beams)
        rospy.logwarn("  repair_mask_topic  = %s (len=%d)  [v3 NEW]",
                      self.repair_mask_topic, self.num_beams)
        rospy.logwarn("  fill_stats_topic   = %s (len=4)   [v3 NEW]",
                      self.fill_stats_topic)
        rospy.logwarn("  num_beams          = %d", self.num_beams)
        rospy.logwarn("  fov_deg            = [%.1f, %.1f]", self.fov_min_deg, self.fov_max_deg)
        rospy.logwarn("  max_distance       = %.3f m", self.max_distance)
        rospy.logwarn("  range_min          = %.3f m", self.range_min)
        rospy.logwarn("  invalid_fill (兜底) = %.3f m (仅用于无历史的极端空帧)", self.invalid_fill)
        rospy.logwarn("  repair_hist_len    = %d", self.hist_len)
        rospy.logwarn("  repair_neighbors   = %d (single side)", self.space_neighbors)
        rospy.logwarn("  front_fov_deg      = %.1f (for front_repair_count)", self.front_fov_deg)
        rospy.logwarn("  weights(full)      = %s", str(REPAIR_W_SPACE_FULL))
        rospy.logwarn("  weights(partial)   = %s", str(REPAIR_W_SPACE_PARTIAL))
        rospy.logwarn("  weights(empty)     = %s", str(REPAIR_W_SPACE_EMPTY))
        rospy.logwarn("======================================")

    # ==================================================================
    # 角度构造
    # ==================================================================
    @staticmethod
    def _build_target_angles_rad(min_deg, max_deg, num_beams):
        if num_beams <= 1:
            return [math.radians(min_deg)]
        step = (max_deg - min_deg) / float(num_beams - 1)
        return [math.radians(min_deg + i * step) for i in range(num_beams)]

    # ==================================================================
    # 有效性判断(替代 v2 的 _sanitize_range)
    # ==================================================================
    def _is_valid_raw(self, r):
        """仅判断 r 是否是可信原始测量。"""
        if r is None:
            return False
        # 注意顺序:NaN 与自身比较恒 False,所以用 math.isnan 显式判断
        if math.isnan(r) or math.isinf(r):
            return False
        if r < self.range_min:
            return False
        if r > self.max_distance:
            return False
        return True

    # ==================================================================
    # 小型数学工具(不引入 numpy,保持与 v1/v2 一致的轻量风格)
    # ==================================================================
    @staticmethod
    def _median(values):
        """非空 list 的中位数。"""
        s = sorted(values)
        n = len(s)
        mid = n // 2
        if n % 2 == 1:
            return s[mid]
        return 0.5 * (s[mid - 1] + s[mid])

    @staticmethod
    def _quantile_sorted(sorted_values, q):
        """已排序序列的 q 分位数,线性插值。"""
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

    # ==================================================================
    # d_space / d_time / d_frame
    # ==================================================================
    def _compute_d_space(self, ranges, is_valid, i, n):
        """左右各最多取 space_neighbors 个有效点,取中位数,按到最近有效点的距离
        做边界插值。

        Returns:
            (d_space, status) where status ∈ {'full', 'partial', 'empty'}
        """
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
            # 边界插值:越靠近哪一侧,哪一侧权重越大。
            # w_R = nearest_left / (nearest_left + nearest_right)
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
        """最近 hist_len 帧同一原始索引 i 的值的中位数。
        历史中都是已修补(或原本就有效)的值,因此不再需要额外有效性判断。
        """
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
        """局部扇区分位数,三级退化:
            一级: ±12° 内有效点 >= 5 个,取 30% 分位数
            二级: ±25° 内有效点 >= 3 个,取 30% 分位数
            三级: 全帧 35% 分位数(已预先算好传入)
        """
        # 一级:±12°
        lo = max(0, i - self._half_idx_12)
        hi = min(n, i + self._half_idx_12 + 1)
        local = [ranges[j] for j in range(lo, hi) if is_valid[j]]
        if len(local) >= REPAIR_D_FRAME_LOCAL_MIN:
            return self._quantile_sorted(sorted(local), REPAIR_D_FRAME_LOCAL_Q)

        # 二级:±25°
        lo = max(0, i - self._half_idx_25)
        hi = min(n, i + self._half_idx_25 + 1)
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

    # ==================================================================
    # 主修补函数
    # ==================================================================
    def _repair_raw_ranges(self, ranges, angle_min, angle_increment):
        """在原始米制空间、原始 scan 分辨率上对无效点做动态修补。

        Returns:
            repaired_ranges : list[float] len=n,全部合法
            raw_repair_mask : list[int]   len=n,1 表示该点由修补得到
            raw_severe_mask : list[int]   len=n,1 表示该点无任何空间邻居信息,
                              主要依赖 d_time + d_frame 兜底
        """
        n = len(ranges)
        if n == 0:
            return [], [], []

        # 1) 有效性扫描
        is_valid = [self._is_valid_raw(r) for r in ranges]
        valid_values = [ranges[i] for i in range(n) if is_valid[i]]
        valid_count = len(valid_values)

        # 边界情况:整帧全无效
        if valid_count == 0:
            if self._time_hist:
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
                "[lidar_obs_single_frame v3] entire frame invalid, "
                "fell back to %s (n=%d).",
                "last frame" if self._time_hist else "invalid_fill", n,
            )
            return repaired, raw_repair_mask, raw_severe_mask

        # 2) 首次锁定角度相关的索引半宽
        if self._half_idx_12 is None:
            if angle_increment > 0:
                self._half_idx_12 = max(
                    1, int(math.radians(REPAIR_D_FRAME_12_DEG) / angle_increment))
                self._half_idx_25 = max(
                    1, int(math.radians(REPAIR_D_FRAME_25_DEG) / angle_increment))
                self._front_half_idx = max(
                    1, int(math.radians(self.front_fov_deg) / angle_increment))
                self._front_center_raw = int(round((-angle_min) / angle_increment))
                self._front_center_raw = max(0, min(n - 1, self._front_center_raw))
                rospy.logwarn(
                    "[lidar_obs_single_frame v3] angle params locked: "
                    "half_idx_12=%d  half_idx_25=%d  front_half_idx=%d  "
                    "front_center_raw=%d",
                    self._half_idx_12, self._half_idx_25,
                    self._front_half_idx, self._front_center_raw,
                )
            else:
                # 异常 scan 元数据的保险值(极少见)
                self._half_idx_12 = 5
                self._half_idx_25 = 11
                self._front_half_idx = 8
                self._front_center_raw = n // 2
                rospy.logwarn_throttle(
                    5.0,
                    "[lidar_obs_single_frame v3] scan.angle_increment<=0, "
                    "using fallback half_idx values.",
                )

        # 3) 全帧兜底分位数(d_frame 三级)
        frame_q_global = self._quantile_sorted(sorted(valid_values),
                                                REPAIR_D_FRAME_GLOBAL_Q)

        # 4) 逐点修补
        repaired = list(ranges)
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

            # 融合:不可用分量从分子分母中剔除并重新归一
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

            # 夹紧到 [range_min, max_distance]
            if d_fill < self.range_min:
                d_fill = self.range_min
            elif d_fill > self.max_distance:
                d_fill = self.max_distance

            repaired[i] = d_fill
            raw_repair_mask[i] = 1
            if space_status == 'empty':
                raw_severe_mask[i] = 1

        # 5) 更新时域历史
        self._time_hist.append(list(repaired))

        return repaired, raw_repair_mask, raw_severe_mask

    # ==================================================================
    # 181 维重采样(作用于修补后的 ranges,不再含 sanitize)
    # ==================================================================
    def _resample_single_frame_from_repaired(self, repaired_ranges,
                                             angle_min, angle_increment):
        n_src = len(repaired_ranges)
        if n_src == 0:
            fill = self.invalid_fill / self.max_distance
            return [fill] * self.num_beams
        if angle_increment <= 0:
            # 病态 scan,退化为均匀填充
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

    # ==================================================================
    # raw_repair_mask -> 181 维 repair_mask(通过缓存的 src_map)
    # ==================================================================
    def _compute_repair_mask_181(self, raw_repair_mask, raw_severe_mask, n_src):
        if self._src_floor is None:
            return [0] * self.num_beams, [0] * self.num_beams

        mask_181   = [0] * self.num_beams
        severe_181 = [0] * self.num_beams

        for k in range(self.num_beams):
            f = self._src_floor[k]
            c = self._src_ceil[k]
            if f < 0: f = 0
            elif f >= n_src: f = n_src - 1
            if c < 0: c = 0
            elif c >= n_src: c = n_src - 1

            # 只要 floor 或 ceil 任一端被修补过,这一 beam 就标记为修补
            if raw_repair_mask[f] or raw_repair_mask[c]:
                mask_181[k] = 1
            if raw_severe_mask[f] or raw_severe_mask[c]:
                severe_181[k] = 1

        if self.reverse_output:
            mask_181.reverse()
            severe_181.reverse()

        return mask_181, severe_181

    # ==================================================================
    # fill_stats 计算
    # ==================================================================
    def _compute_fill_stats(self, repair_mask_181, severe_mask_181):
        """返回 [repair_ratio, repair_count, front_repair_count, severe_repair_ratio]。

        front_repair_count 在 181 维空间计算:
          - 181 维的 FOV 中心(angle=0°)对应 index = num_beams // 2 = 90。
          - 每个 beam 对应角宽 = (fov_max - fov_min) / (num_beams - 1) 度。
          - 半宽 = round(front_fov_deg / deg_per_beam)。
        无论 reverse_output,中心 index 均为 90(181 为奇数)。
        """
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

        return [float(repair_ratio),
                float(repair_count),
                float(front_repair_count),
                float(severe_ratio)]

    # ==================================================================
    # src_map 构造(与 v2 同语义,v3 额外缓存 floor/ceil/weight 数组)
    # ==================================================================
    def _build_src_map(self, scan):
        n_src = len(scan.ranges)
        floors  = [0.0] * self.num_beams
        ceils   = [0.0] * self.num_beams
        weights = [0.0] * self.num_beams

        floor_int = [0] * self.num_beams
        ceil_int  = [0] * self.num_beams

        for i, angle in enumerate(self.target_angles):
            if self.use_linear_interpolation:
                pos = (angle - scan.angle_min) / scan.angle_increment
                if pos <= 0.0:
                    f = c = 0; w = 0.0
                elif pos >= n_src - 1:
                    f = c = n_src - 1; w = 0.0
                else:
                    f = int(math.floor(pos))
                    c = f + 1
                    w = pos - f
            else:
                idx = int(round((angle - scan.angle_min) / scan.angle_increment))
                idx = max(0, min(n_src - 1, idx))
                f = c = idx; w = 0.0
            floors[i]    = float(f)
            ceils[i]     = float(c)
            weights[i]   = float(w)
            floor_int[i] = f
            ceil_int[i]  = c

        # v3 缓存:供 repair_mask 映射快速查 (O(num_beams))
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

    # ==================================================================
    # MultiArray 构造
    # ==================================================================
    @staticmethod
    def _make_multiarray(data, dim_label):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label=dim_label, size=len(data), stride=len(data))
        ]
        msg.layout.data_offset = 0
        msg.data = data
        return msg

    # ==================================================================
    # 主回调
    # ==================================================================
    def scan_callback(self, scan_msg):
        with self.lock:
            n_src = len(scan_msg.ranges)

            # 第一帧到达时构造 src_map(repair_mask 映射依赖它)
            if self._src_map_msg is None and n_src > 0 and scan_msg.angle_increment > 0:
                self._src_map, self._src_map_msg = self._build_src_map(scan_msg)
                preview_f = self._src_map[:5]
                preview_c = self._src_map[self.num_beams:self.num_beams + 5]
                preview_w = self._src_map[2 * self.num_beams:2 * self.num_beams + 5]
                rospy.logwarn(
                    "[lidar_obs_single_frame v3] src_map built from first scan "
                    "(n_src=%d)  floor[0:5]=%s  ceil[0:5]=%s  weight[0:5]=%s",
                    n_src,
                    ["%d" % int(v) for v in preview_f],
                    ["%d" % int(v) for v in preview_c],
                    ["%.3f" % v for v in preview_w],
                )

            # --- Step 1: 原始米制空间动态修补 ---
            repaired_ranges, raw_repair_mask, raw_severe_mask = self._repair_raw_ranges(
                list(scan_msg.ranges), scan_msg.angle_min, scan_msg.angle_increment)

            # --- Step 2: 用修补后的 ranges 做 181 维重采样 + 归一化 ---
            lidar_181 = self._resample_single_frame_from_repaired(
                repaired_ranges, scan_msg.angle_min, scan_msg.angle_increment)

            # --- Step 3: raw_repair_mask -> 181 维 mask ---
            if n_src > 0:
                repair_mask_181, severe_mask_181 = self._compute_repair_mask_181(
                    raw_repair_mask, raw_severe_mask, n_src)
            else:
                repair_mask_181 = [1] * self.num_beams
                severe_mask_181 = [1] * self.num_beams

            # --- Step 4: fill_stats ---
            fill_stats = self._compute_fill_stats(repair_mask_181, severe_mask_181)

            # --- Step 5: 发布(stamp 优先,保证下游时序配对) ---
            stamp_msg = Header()
            stamp_msg.stamp    = scan_msg.header.stamp
            stamp_msg.frame_id = scan_msg.header.frame_id
            self.pub_stamp.publish(stamp_msg)

            if self.publish_src_map and self._src_map_msg is not None:
                self.pub_src_map.publish(self._src_map_msg)

            self.pub.publish(self._make_multiarray(lidar_181, "lidar_181"))
            self.pub_repair_mask.publish(self._make_multiarray(
                [float(m) for m in repair_mask_181], "lidar_181_repair_mask"))
            self.pub_fill_stats.publish(self._make_multiarray(
                fill_stats, "lidar_181_fill_stats"))

            # --- 调试日志 ---
            self.scan_count += 1
            if self.publish_debug_info and self.scan_count % 20 == 1:
                center_idx = len(lidar_181) // 2
                rospy.logwarn(
                    "[lidar_obs_single_frame v3] seq=%d  raw_n=%d -> out_n=%d | "
                    "min=%.4f  max=%.4f  center=%.4f | "
                    "repair_ratio=%.3f  count=%d  front=%d  severe_ratio=%.3f",
                    self.scan_count, n_src, len(lidar_181),
                    min(lidar_181) if lidar_181 else -1.0,
                    max(lidar_181) if lidar_181 else -1.0,
                    lidar_181[center_idx] if lidar_181 else -1.0,
                    fill_stats[0], int(fill_stats[1]),
                    int(fill_stats[2]), fill_stats[3],
                )


def main():
    rospy.init_node("lidar_obs_single_frame", anonymous=False)
    LidarObsSingleFrameNode()
    rospy.spin()


if __name__ == "__main__":
    main()
