#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 2/3 : lidar_obs_single_frame  (v2)
============================================================
职责:把方向校正后的 LaserScan 编码为策略网络可直接消费的
     181 维归一化单帧观测。

输入 : /scan_aligned       (sensor_msgs/LaserScan)
输出 : /rl/lidar_181       (std_msgs/Float32MultiArray, len = 181)

伴随输出(用于跨节点时序诊断):
  /rl/lidar_181_stamp      (std_msgs/Header)
    - stamp     = 输入 /scan_aligned 的 header.stamp(源时间,与 laser_switch 保留一致)
    - frame_id  = 输入的 frame_id(保留溯源)

  /rl/lidar_181_src_map    (std_msgs/Float32MultiArray, len = 181*3 = 543)  [v2]
    精确描述本节点用的降采样 / 插值映射,供离线验证工具(pipeline_
    conformance_verifier)精确复现 C2.3 的异常填零语义,而不再依赖
    "均匀 linspace 采样"的假设。
    数据布局(三段各 181 个元素):
      [0   :181]  floor_indices  —— 每个目标光束在源 /scan_aligned 中的 floor 索引
      [181 :362]  ceil_indices   —— 同上,ceil 索引(最近邻模式下 == floor)
      [362 :543]  weights        —— 对 ceil 的插值权重 w ∈ [0,1];
                                   最终值 = (1-w)*scan[floor] + w*scan[ceil]
    该映射由第一帧到达时根据 scan.angle_min / angle_increment 算定,
    之后每帧都重新推送一份(rosbag 可直接录,每行一样,verifier 读第一行即可)。

处理流水:
  1) 视场裁剪   : 仅保留 [fov_min_deg, fov_max_deg] = [-135°, +135°]
  2) 角度重采样 : 均匀采样 num_beams = 181 个角度(默认线性插值)
  3) 异常值处理 : inf / nan / 负值 / 越界值 / 空帧 -> INVALID_FILL_VALUE(0.0 m)
                  注意:sanitize 发生在"插值前",所以如果 floor=inf, ceil=5m,
                  sanitize 后 floor=0, ceil=5, 插值结果 w*5(非零小数)。
                  lidar_181 的 zero 只出现在 floor 与 ceil **都** abnormal 的位置。
  4) 归一化     : value / LIDAR_MAX_DISTANCE  -> 输出范围 [0, 1]

CHANGELOG v2:
  * 新增 /rl/lidar_181_src_map 诊断话题(解决 C2.3 中工具 linspace 假设
    与节点实际插值映射不一致导致的 0.17% 伪违规)
  * 订阅队列从 1 提到 5,发布队列从 10 提到 20;加 tcp_nodelay=True
  * 启动日志里额外输出 src_map 的前几项,便于肉眼核对

链路统一参数:
  - LIDAR_MAX_DISTANCE = 16.0 m
  - LIDAR_NUM_BEAMS    = 181
  - LIDAR_FOV_DEG      = [-135, 135]
  - INVALID_FILL_VALUE = 0.0 m

管线定位:LiDAR 观测预处理链第 2 级,上游 laser_switch,下游 lidar_stack_543。
============================================================
"""

from __future__ import division, print_function

import math
import threading

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header


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
        self.scan_topic    = rospy.get_param("~scan_topic",    "/scan_aligned")
        self.output_topic  = rospy.get_param("~output_topic",  "/rl/lidar_181")
        self.stamp_topic   = rospy.get_param("~stamp_topic",   "/rl/lidar_181_stamp")
        # v2: src_map 诊断话题
        self.src_map_topic = rospy.get_param("~src_map_topic", "/rl/lidar_181_src_map")

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
        # v2: 诊断话题开关(默认开)
        self.publish_src_map          = bool(rospy.get_param("~publish_src_map", True))

        # --- 内部状态 ---
        self.lock = threading.Lock()
        self.scan_count = 0
        self.target_angles = self._build_target_angles_rad(
            self.fov_min_deg, self.fov_max_deg, self.num_beams
        )
        # v2: src_map 缓存,第一帧到达时由 scan.angle_min/increment 算定
        self._src_map = None          # list[float] len=3*num_beams
        self._src_map_msg = None      # 预构造的 Float32MultiArray,避免每帧重建

        # --- 发布/订阅 ---
        # 先建 stamp 发布器,再建主数据发布器:保证启动后首帧的发送顺序是 stamp-first
        self.pub_stamp   = rospy.Publisher(self.stamp_topic,  Header,            queue_size=20)
        self.pub_src_map = rospy.Publisher(self.src_map_topic, Float32MultiArray, queue_size=20)
        self.pub         = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=20)
        self.sub         = rospy.Subscriber(self.scan_topic, LaserScan,
                                            self.scan_callback,
                                            queue_size=5,
                                            tcp_nodelay=True)

        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_obs_single_frame] started (pipeline stage 2/3, v2)")
        rospy.logwarn("  scan_topic     = %s", self.scan_topic)
        rospy.logwarn("  output_topic   = %s", self.output_topic)
        rospy.logwarn("  stamp_topic    = %s (std_msgs/Header, companion)", self.stamp_topic)
        rospy.logwarn("  src_map_topic  = %s (Float32MultiArray, len=%d, companion)",
                      self.src_map_topic, 3 * self.num_beams)
        rospy.logwarn("  num_beams      = %d", self.num_beams)
        rospy.logwarn("  fov_deg        = [%.1f, %.1f]", self.fov_min_deg, self.fov_max_deg)
        rospy.logwarn("  max_distance   = %.3f m (normalization denominator)", self.max_distance)
        rospy.logwarn("  invalid_fill   = %.3f m (raw, pre-normalization)", self.invalid_fill)
        rospy.logwarn("  interp         = %s", str(self.use_linear_interpolation))
        rospy.logwarn("  reverse_out    = %s", str(self.reverse_output))
        rospy.logwarn("  publish_src_map= %s", str(self.publish_src_map))
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
    # v2: src_map 构造
    # ------------------------------------------------------------------
    def _build_src_map(self, scan):
        """根据 scan.angle_min / angle_increment + target_angles,
        算出每个目标光束在源 /scan_aligned 中的 (floor, ceil, weight)。
        返回 list[float] len=3*num_beams,布局为 [floor..., ceil..., weight...]。
        最近邻模式下 floor == ceil,weight = 0。"""
        n_src = len(scan.ranges)
        floors  = [0.0] * self.num_beams
        ceils   = [0.0] * self.num_beams
        weights = [0.0] * self.num_beams

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
            floors[i]  = float(f)
            ceils[i]   = float(c)
            weights[i] = float(w)

        src_map = floors + ceils + weights   # 长度 3 * num_beams

        # 预构造 Float32MultiArray(带三段 dim label,便于订阅者切片)
        msg = Float32MultiArray()
        dim_f = MultiArrayDimension(label="floor",  size=self.num_beams, stride=self.num_beams)
        dim_c = MultiArrayDimension(label="ceil",   size=self.num_beams, stride=self.num_beams)
        dim_w = MultiArrayDimension(label="weight", size=self.num_beams, stride=self.num_beams)
        msg.layout.dim = [dim_f, dim_c, dim_w]
        msg.layout.data_offset = 0
        msg.data = src_map
        return src_map, msg

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def scan_callback(self, scan_msg):
        with self.lock:
            lidar_181 = self._resample_single_frame(scan_msg)

            # v2: 第一帧到达时构造 src_map(依赖 scan.angle_min/increment);
            # 之后每帧继续推送同一份,让 rosbag 的每一行都带上,verifier 读任意行即可。
            if self._src_map_msg is None and len(scan_msg.ranges) > 0:
                self._src_map, self._src_map_msg = self._build_src_map(scan_msg)
                # 启动日志里吐出前 5 项,便于肉眼核对
                preview_f = self._src_map[:5]
                preview_c = self._src_map[self.num_beams:self.num_beams + 5]
                preview_w = self._src_map[2 * self.num_beams:2 * self.num_beams + 5]
                rospy.logwarn(
                    "[lidar_obs_single_frame] src_map built from first scan "
                    "(n_src=%d)  floor[0:5]=%s  ceil[0:5]=%s  weight[0:5]=%s",
                    len(scan_msg.ranges),
                    ["%d" % int(v) for v in preview_f],
                    ["%d" % int(v) for v in preview_c],
                    ["%.3f" % v for v in preview_w],
                )

            # 先发伴随 stamp(Header),再发主数据(Float32MultiArray),
            # 尽量让下游在处理主数据时已拿到配对的源时间戳。
            stamp_msg = Header()
            stamp_msg.stamp    = scan_msg.header.stamp
            stamp_msg.frame_id = scan_msg.header.frame_id
            self.pub_stamp.publish(stamp_msg)

            # v2: 发布诊断 src_map(若启用)
            if self.publish_src_map and self._src_map_msg is not None:
                self.pub_src_map.publish(self._src_map_msg)

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
