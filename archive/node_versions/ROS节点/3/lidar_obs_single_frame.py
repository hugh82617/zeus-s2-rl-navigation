#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 2/3 : lidar_obs_single_frame
============================================================
职责:把方向校正后的 LaserScan 编码为策略网络可直接消费的
     181 维归一化单帧观测。

输入 : /scan_aligned       (sensor_msgs/LaserScan)
输出 : /rl/lidar_181       (std_msgs/Float32MultiArray, len = 181)

处理流水:
  1) 视场裁剪   : 仅保留 [fov_min_deg, fov_max_deg] = [-135°, +135°]
  2) 角度重采样 : 均匀采样 num_beams = 181 个角度(默认线性插值)
  3) 异常值处理 : inf / nan / 负值 / 越界值 / 空帧 -> INVALID_FILL_VALUE(0.0 m)
                  -- 全部统一表征为"近距离障碍",形成保守避障语义
  4) 归一化     : value / LIDAR_MAX_DISTANCE  -> 输出范围 [0, 1]

链路统一参数:
  - LIDAR_MAX_DISTANCE = 16.0 m   (归一化分母,正式固化为量程基准)
  - LIDAR_NUM_BEAMS    = 181
  - LIDAR_FOV_DEG      = [-135, 135]
  - INVALID_FILL_VALUE = 0.0 m   (异常/无效量测的填充值,归一化前)

输出频率严格跟随 /scan_aligned 的发布频率,不做节流与平滑。

管线定位:LiDAR 观测预处理链第 2 级,上游 laser_switch,下游 lidar_stack_543。
============================================================
"""

from __future__ import division, print_function

import math
import threading

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


# ============================================================
# 链路统一常量 —— 请与 lidar_stack_543 / rl_planner_node 保持一致
# ============================================================
LIDAR_MAX_DISTANCE = 16.0      # 感知量程上限(m),归一化分母
LIDAR_NUM_BEAMS    = 181       # 单帧观测维度
LIDAR_FOV_MIN_DEG  = -135.0
LIDAR_FOV_MAX_DEG  = 135.0
INVALID_FILL_VALUE = 0.0       # 异常量测 -> "近距离障碍"(保守避障)


class LidarObsSingleFrameNode(object):
    def __init__(self):
        # --- 话题 ---
        self.scan_topic   = rospy.get_param("~scan_topic",   "/scan_aligned")
        self.output_topic = rospy.get_param("~output_topic", "/rl/lidar_181")

        # --- 链路统一参数(可由 launch 覆写,但默认值即链路规范) ---
        self.fov_min_deg  = float(rospy.get_param("~fov_min_deg",   LIDAR_FOV_MIN_DEG))
        self.fov_max_deg  = float(rospy.get_param("~fov_max_deg",   LIDAR_FOV_MAX_DEG))
        self.num_beams    = int  (rospy.get_param("~num_beams",     LIDAR_NUM_BEAMS))
        self.max_distance = float(rospy.get_param("~max_distance",  LIDAR_MAX_DISTANCE))
        self.invalid_fill = float(rospy.get_param("~invalid_fill_value", INVALID_FILL_VALUE))

        # --- 处理选项 ---
        self.use_linear_interpolation = bool(rospy.get_param("~use_linear_interpolation", True))
        self.reverse_output           = bool(rospy.get_param("~reverse_output", False))
        self.publish_debug_info       = bool(rospy.get_param("~publish_debug_info", True))

        # --- 内部状态 ---
        self.lock = threading.Lock()
        self.scan_count = 0
        self.target_angles = self._build_target_angles_rad(
            self.fov_min_deg, self.fov_max_deg, self.num_beams
        )

        # --- 发布/订阅 ---
        self.pub = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan,
                                    self.scan_callback, queue_size=1)

        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_obs_single_frame] started (pipeline stage 2/3)")
        rospy.logwarn("  scan_topic   = %s", self.scan_topic)
        rospy.logwarn("  output_topic = %s", self.output_topic)
        rospy.logwarn("  num_beams    = %d", self.num_beams)
        rospy.logwarn("  fov_deg      = [%.1f, %.1f]", self.fov_min_deg, self.fov_max_deg)
        rospy.logwarn("  max_distance = %.3f m (normalization denominator)", self.max_distance)
        rospy.logwarn("  invalid_fill = %.3f m (raw, pre-normalization)", self.invalid_fill)
        rospy.logwarn("  interp       = %s", str(self.use_linear_interpolation))
        rospy.logwarn("  reverse_out  = %s", str(self.reverse_output))
        rospy.logwarn("======================================")

    # ------------------------------------------------------------------
    # 角度与取值
    # ------------------------------------------------------------------
    @staticmethod
    def _build_target_angles_rad(min_deg, max_deg, num_beams):
        if num_beams <= 1:
            return [math.radians(min_deg)]
        step = (max_deg - min_deg) / float(num_beams - 1)
        return [math.radians(min_deg + i * step) for i in range(num_beams)]

    def _sanitize_range(self, r):
        """异常/无效量测 -> invalid_fill(默认 0.0 m,即"近距离障碍")。"""
        if r is None:
            return self.invalid_fill
        if math.isnan(r) or math.isinf(r):
            return self.invalid_fill
        if r < 0.0:
            return self.invalid_fill
        if r > self.max_distance:
            # 越量程:视为不可信 -> 保守处理(而非截断为 max_distance)
            return self.invalid_fill
        return r

    def _get_range_nearest(self, scan, angle):
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(len(scan.ranges) - 1, idx))
        return self._sanitize_range(scan.ranges[idx])

    def _get_range_interp(self, scan, angle):
        pos = (angle - scan.angle_min) / scan.angle_increment
        if pos <= 0.0:
            return self._sanitize_range(scan.ranges[0])
        if pos >= len(scan.ranges) - 1:
            return self._sanitize_range(scan.ranges[-1])

        i0 = int(math.floor(pos))
        i1 = i0 + 1
        w  = pos - i0

        r0 = self._sanitize_range(scan.ranges[i0])
        r1 = self._sanitize_range(scan.ranges[i1])
        return (1.0 - w) * r0 + w * r1

    # ------------------------------------------------------------------
    # 单帧编码
    # ------------------------------------------------------------------
    def _resample_single_frame(self, scan):
        # 空帧 -> 全部按 invalid_fill 归一化输出,即全视场"近距离障碍"
        if not scan.ranges:
            fill = self.invalid_fill / self.max_distance
            return [fill] * self.num_beams

        beams = []
        for angle in self.target_angles:
            if self.use_linear_interpolation:
                r = self._get_range_interp(scan, angle)
            else:
                r = self._get_range_nearest(scan, angle)
            beams.append(r / self.max_distance)   # 归一化到 [0, 1]

        if self.reverse_output:
            beams.reverse()
        return beams

    @staticmethod
    def _make_multiarray(data, dim_label):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label=dim_label, size=len(data), stride=len(data))
        ]
        msg.layout.data_offset = 0
        msg.data = data
        return msg

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def scan_callback(self, scan_msg):
        with self.lock:
            lidar_181 = self._resample_single_frame(scan_msg)
            self.pub.publish(self._make_multiarray(lidar_181, "lidar_181"))

            self.scan_count += 1
            if self.publish_debug_info and self.scan_count % 20 == 1:
                center_idx = len(lidar_181) // 2
                rospy.logwarn(
                    "[lidar_obs_single_frame] seq=%d raw_n=%d -> out_n=%d | min=%.4f max=%.4f center=%.4f",
                    self.scan_count,
                    len(scan_msg.ranges),
                    len(lidar_181),
                    min(lidar_181) if lidar_181 else -1.0,
                    max(lidar_181) if lidar_181 else -1.0,
                    lidar_181[center_idx] if lidar_181 else -1.0,
                )


def main():
    rospy.init_node("lidar_obs_single_frame", anonymous=False)
    LidarObsSingleFrameNode()
    rospy.spin()


if __name__ == "__main__":
    main()
