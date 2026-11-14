#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 1/3 : laser_switch  (v3)
============================================================
职责:原始激光方向校正入口 + **无效量测分布标记器**。
     保持 v2 的"只做 180° 半圈索引重排,不做数值修补"这一职责不变,
     在此基础上新增两个诊断话题,用于在链路源头记录无效量测分布。

输入 : /scan                             (sensor_msgs/LaserScan)

主输出 : /scan_aligned                   (sensor_msgs/LaserScan)
         - 180° 半圈平移重排:new[i] = old[(i + N/2) % N]
         - 元数据(angle_min/max/increment/scan_time/range_*)原样透传
         - 可选重写 frame_id 为 laser_link_aligned

v3 新增诊断输出:
  /scan_aligned_invalid_mask             (std_msgs/Float32MultiArray, len = N)
      与 ranges 等长,逐点 0/1 标记。
      - 有效值:0
      - inf / nan / 负值 / < range_min / > range_max : 1

  /scan_aligned_invalid_stats            (std_msgs/Float32MultiArray, len = 4)
      [invalid_count, invalid_ratio, front_invalid_count, front_invalid_ratio]
      - invalid_count        : 整帧无效点数量
      - invalid_ratio        : invalid_count / N
      - front_invalid_count  : 正前方核心区(±front_fov_deg,默认 ±20°)
                               在 post-rotation 索引空间内的无效点数量
      - front_invalid_ratio  : front_invalid_count / front_zone_width

---------------------------------------------------------------
设计要点(文档第三节(一)):
  1) 第 1 级本身仍然不做任何数值修补,inf/nan 等在本节点透传进入
     /scan_aligned。数值修补是第 2 级的职责。
  2) 计算 invalid_mask 时使用 post-rotation 后的 ranges,使之与
     /scan_aligned 的索引空间直接对应,下游工具 join 时零偏移。
  3) front 扇区中心 = 机器人前向(angle=0)对应的 post-rotation 索引。
     在本节点里,rotation 只改变数据内容位置,angle_min/angle_increment 原样
     透传,所以 post-rotation 下 angle θ 在索引 round((θ - angle_min)/inc)
     这一点,与原始 /scan 的索引映射一致。

CHANGELOG v2 -> v3:
  [新增] 发布 /scan_aligned_invalid_mask (Float32MultiArray, len=N)
  [新增] 发布 /scan_aligned_invalid_stats (Float32MultiArray, len=4)
  [新增] 首帧锁定正前方扇区索引(front_center_idx / front_half_idx),
         扫描长度变化时自动重算
  [新增] 周期性 stats 日志里补充 last invalid_ratio / front_invalid_count,
         与原有 recv/pub/empty/dropped 计数并列
  [保留] v2 的队列大小、tcp_nodelay、stats_every、rotate_half_turn 等
         全部不动。

管线定位:LiDAR 观测预处理链第 1 级,下游为 lidar_obs_single_frame(v3)。
============================================================
"""

from __future__ import division, print_function

import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


# 正前方核心区默认半宽(°)—— 与第 2 级 v3 的 front_fov_deg 保持一致
INVALID_FRONT_FOV_DEG = 20.0


class LaserSwitch(object):
    def __init__(self):
        # ---------- 主话题 ----------
        self.input_topic     = rospy.get_param("~input_scan_topic",  "/scan")
        self.output_topic    = rospy.get_param("~output_scan_topic", "/scan_aligned")
        self.output_frame_id = rospy.get_param("~output_frame_id",   "laser_link_aligned")

        # ---------- v3 新增诊断话题 ----------
        self.invalid_mask_topic  = rospy.get_param(
            "~invalid_mask_topic",  "/scan_aligned_invalid_mask")
        self.invalid_stats_topic = rospy.get_param(
            "~invalid_stats_topic", "/scan_aligned_invalid_stats")
        self.front_fov_deg       = float(rospy.get_param(
            "~invalid_front_fov_deg", INVALID_FRONT_FOV_DEG))

        # ---------- 通用参数 ----------
        self.verbose         = bool(rospy.get_param("~verbose", True))
        self.sub_queue_size  = int (rospy.get_param("~sub_queue_size", 50))
        self.pub_queue_size  = int (rospy.get_param("~pub_queue_size", 50))
        self.tcp_nodelay     = bool(rospy.get_param("~tcp_nodelay",    True))
        self.stats_every     = int (rospy.get_param("~stats_every",    200))

        # ---------- 计数器 ----------
        self.recv_count  = 0
        self.pub_count   = 0
        self.empty_count = 0

        # ---------- v3 缓存的前向扇区索引 ----------
        self._front_half_idx   = None
        self._front_center_idx = None
        self._cached_n         = None
        self._cached_angle_min = None
        self._cached_angle_inc = None

        # ---------- 发布 ----------
        self.pub = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=self.pub_queue_size)
        self.pub_invalid_mask = rospy.Publisher(
            self.invalid_mask_topic, Float32MultiArray, queue_size=self.pub_queue_size)
        self.pub_invalid_stats = rospy.Publisher(
            self.invalid_stats_topic, Float32MultiArray, queue_size=self.pub_queue_size)

        # ---------- 订阅 ----------
        self.sub = rospy.Subscriber(
            self.input_topic, LaserScan, self.scan_callback,
            queue_size=self.sub_queue_size, tcp_nodelay=self.tcp_nodelay,
        )

        rospy.logwarn("======================================")
        rospy.logwarn("[laser_switch v3] node started (pipeline stage 1/3)")
        rospy.logwarn("  input_topic              : %s", self.input_topic)
        rospy.logwarn("  output_topic             : %s", self.output_topic)
        rospy.logwarn("  output_frame_id          : %s", self.output_frame_id)
        rospy.logwarn("  invalid_mask_topic       : %s  [v3 NEW]", self.invalid_mask_topic)
        rospy.logwarn("  invalid_stats_topic      : %s  [v3 NEW]", self.invalid_stats_topic)
        rospy.logwarn("  invalid_front_fov_deg    : ±%.1f°", self.front_fov_deg)
        rospy.logwarn("  sub_queue / pub_queue    : %d / %d",
                      self.sub_queue_size, self.pub_queue_size)
        rospy.logwarn("  tcp_nodelay              : %s", str(self.tcp_nodelay))
        rospy.logwarn("  stats_every              : %d frames", self.stats_every)
        rospy.logwarn("======================================")

    # ==================================================================
    # 工具函数
    # ==================================================================
    @staticmethod
    def rotate_half_turn(data_list):
        """对长度为 N 的数组做 180° 索引重排 new[i] = old[(i + N/2) % N]。"""
        n = len(data_list)
        if n == 0:
            return data_list
        half = n // 2
        if n % 2 != 0:
            rospy.logwarn_throttle(
                5.0,
                "[laser_switch v3] scan length is odd: %d, "
                "half-turn shift will have 1-sample mismatch.",
                n,
            )
        first_half = data_list[:half]
        new_list   = data_list[half:]
        new_list.extend(first_half)
        return new_list

    @staticmethod
    def _is_invalid(r, range_min, range_max):
        """与文档约定的"无效量测"语义一致:inf / nan / 负值 / < range_min / > range_max。"""
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

    @staticmethod
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

    # ==================================================================
    # 前向扇区索引缓存(首帧或扫描长度/角度参数变化时重算)
    # ==================================================================
    def _update_front_zone_cache(self, n, angle_min, angle_increment):
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
            # 病态扫描元数据的保险值
            self._front_half_idx = max(1, n // 18)   # 约 ±10°
            self._front_center_idx = n // 2
        self._cached_n         = n
        self._cached_angle_min = angle_min
        self._cached_angle_inc = angle_increment
        rospy.logwarn(
            "[laser_switch v3] front zone params (re)computed: "
            "n=%d  center_idx=%d  half_idx=%d  (fov=±%.1f°,  inc=%.5f rad)",
            n, self._front_center_idx, self._front_half_idx,
            self.front_fov_deg, angle_increment,
        )

    # ==================================================================
    # 主回调
    # ==================================================================
    def scan_callback(self, msg):
        self.recv_count += 1
        n = len(msg.ranges)
        if n == 0:
            self.empty_count += 1
            rospy.logwarn_throttle(2.0, "[laser_switch v3] Empty scan received.")
            return

        old_ranges      = list(msg.ranges)
        old_intensities = list(msg.intensities)

        # --- 方向校正 ---
        new_ranges = self.rotate_half_turn(old_ranges)
        if len(old_intensities) == n:
            new_intensities = self.rotate_half_turn(old_intensities)
        else:
            new_intensities = old_intensities   # 长度不一致则原样透传

        # --- 构造并发布 /scan_aligned ---
        out = LaserScan()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = new_ranges
        out.intensities     = new_intensities

        self.pub.publish(out)
        self.pub_count += 1

        # --- v3: 计算 invalid_mask / invalid_stats (基于 post-rotation ranges)---
        invalid_mask = [0] * n
        invalid_count = 0
        r_min = msg.range_min
        r_max = msg.range_max
        for i in range(n):
            if self._is_invalid(new_ranges[i], r_min, r_max):
                invalid_mask[i] = 1
                invalid_count += 1

        # 前向扇区索引缓存
        self._update_front_zone_cache(n, msg.angle_min, msg.angle_increment)

        lo = max(0, self._front_center_idx - self._front_half_idx)
        hi = min(n, self._front_center_idx + self._front_half_idx + 1)
        front_total = hi - lo
        front_invalid_count = sum(invalid_mask[lo:hi]) if front_total > 0 else 0
        front_invalid_ratio = (
            (front_invalid_count / float(front_total)) if front_total > 0 else 0.0
        )
        invalid_ratio = (invalid_count / float(n)) if n > 0 else 0.0

        # 发布诊断话题
        mask_f = [float(v) for v in invalid_mask]
        self.pub_invalid_mask.publish(
            self._build_f32_msg(mask_f, "scan_aligned_invalid_mask"))

        stats = [
            float(invalid_count),
            float(invalid_ratio),
            float(front_invalid_count),
            float(front_invalid_ratio),
        ]
        self.pub_invalid_stats.publish(
            self._build_f32_msg(stats, "scan_aligned_invalid_stats"))

        # --- 周期性 stats 日志(v2 的丢帧诊断 + v3 的无效分布)---
        if self.stats_every > 0 and self.recv_count % self.stats_every == 0:
            dropped = self.recv_count - self.pub_count - self.empty_count
            rospy.logwarn(
                "[laser_switch v3][stats] recv=%d pub=%d empty=%d node_dropped=%d | "
                "last_invalid_ratio=%.3f  front_invalid=%d/%d (%.3f)",
                self.recv_count, self.pub_count, self.empty_count, dropped,
                invalid_ratio, int(front_invalid_count), front_total,
                front_invalid_ratio,
            )

        # --- verbose 日志(保留 v2 风格)---
        if self.verbose:
            idx_0 = int(round((0.0 - msg.angle_min) / msg.angle_increment)) if msg.angle_increment > 0 else n // 2
            idx_0 = max(0, min(n - 1, idx_0))
            idx_neg180 = 0
            idx_pos180 = n - 1
            rospy.logwarn_throttle(
                1.0,
                "[laser_switch v3] raw: 0deg=%s -180deg=%s +180deg=%s | "
                "aligned: 0deg=%s -180deg=%s +180deg=%s | inv=%d/%d front_inv=%d",
                self.safe_value_str(old_ranges[idx_0]),
                self.safe_value_str(old_ranges[idx_neg180]),
                self.safe_value_str(old_ranges[idx_pos180]),
                self.safe_value_str(new_ranges[idx_0]),
                self.safe_value_str(new_ranges[idx_neg180]),
                self.safe_value_str(new_ranges[idx_pos180]),
                invalid_count, n, int(front_invalid_count),
            )


if __name__ == "__main__":
    rospy.init_node("laser_switch")
    LaserSwitch()
    rospy.spin()
