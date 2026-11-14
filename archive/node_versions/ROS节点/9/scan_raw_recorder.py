#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Diagnostic Node: scan_raw_recorder
============================================================
职责:逐帧记录 sensor_msgs/LaserScan 的原始量测与分类统计,
     是 LiDAR 观测预处理链的现场数据采集工具。

典型用途:
  - 事后对比部署现场与训练/仿真的数据分布
  - 定位 inf / nan / 越界 比例异常的传感器或驱动问题
  - 为 lidar_obs_single_frame 的异常填充策略(异常 -> 0.0 m 近距离障碍)
    提供现场数据支撑——可直接答"全场域异常比例占比多少、会不会过度触发保守避障"

输入:
  任意 sensor_msgs/LaserScan 话题。
  默认 /scan(原始),也可配置为 /scan_aligned(方向校正后)以对比差异。

输出文件(位于 record_dir 下):
  1) <stem>.jsonl         每帧一行 JSON,含:
                            - 时间对齐字段(stamp / recv_time / wall_time / transport_delay_ms)
                            - header / 扫描元数据
                            - 分类计数(valid/inf/nan/neg/under/over)
                            - 有效量测 min-max
                            - 7 个关键角度采样值(-180/-135/-90/0/90/135/180)
  2) <stem>.summary.txt   关停时写出整个运行周期的累积分类计数、比例与延迟分布
  其中 <stem> = record_basename[__<launch_timestamp>],
  默认附加启动时间戳,与现有 snapshot 文件风格一致,避免多次启动互相覆盖。

时间字段说明(用于跨节点比对):
  - stamp              : sensor header 时间,是跨节点 JOIN 的**主键**;
                         laser_switch 显式保留 header,所以 /scan 与
                         /scan_aligned 的同一帧 stamp 相同。
  - recv_time          : 本节点回调触发时的 ROS 时间;与其它节点的 recv_time
                         对比可得到端到端链路每一跳的延迟。
  - wall_time          : 系统墙钟 (time.time()),独立于 ROS 时钟;
                         当 /use_sim_time 开启时作为独立参照。
  - transport_delay_ms : recv_time - stamp,已转为毫秒。

可选特性:
  - log_every_n        : 每 N 帧采样记录一次,降低磁盘占用
  - flush_every_n      : 每 N 条 flush 一次,平衡崩溃安全性与 I/O 成本
  - max_file_size_mb   : 超过阈值自动滚转到 .partK 分卷(0 表示不限)
  - record_full_ranges : 把整条 ranges 也写入(调试用,文件会很大)

管线定位:LiDAR 诊断工具,位于主链路之外,可并联挂接到 /scan 或 /scan_aligned,
         不影响 laser_switch / lidar_obs_single_frame / lidar_stack_543 的实时性。
============================================================
"""

from __future__ import division, print_function

import os
import math
import json
import time
import errno

import rospy
from sensor_msgs.msg import LaserScan


SAMPLE_ANGLES_DEG = [-180, -135, -90, 0, 90, 135, 180]


class ScanRawRecorder(object):
    def __init__(self):
        # ===== 参数 =====
        self.scan_topic           = rospy.get_param("~scan_topic",            "/scan")
        self.record_dir           = rospy.get_param("~record_dir",            "/home/ros/save_dir/lidar")
        self.record_basename      = rospy.get_param("~record_basename",       "scan_raw_log")
        self.use_launch_timestamp = bool (rospy.get_param("~use_launch_timestamp", True))
        self.record_full_ranges   = bool (rospy.get_param("~record_full_ranges",   False))
        self.log_every_n          = max(1, int(rospy.get_param("~log_every_n",  1)))
        self.flush_every_n        = max(1, int(rospy.get_param("~flush_every_n", 10)))
        self.print_summary        = bool (rospy.get_param("~print_summary",      True))
        self.max_file_size_mb     = float(rospy.get_param("~max_file_size_mb",   0.0))

        # ===== 内部状态 =====
        self.msg_count = 0            # 收到的帧数(不受 log_every_n 影响)
        self.write_count = 0          # 实际写入的帧数
        self.rotate_counter = 0
        self.launch_timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())

        self.cumulative = {
            "total_msgs":   0,
            "total_ranges": 0,
            "valid": 0, "inf": 0, "nan": 0, "neg": 0, "under": 0, "over": 0,
            # 传输延迟 (recv_time - header.stamp),单位秒,用于跨节点时序诊断
            "delay_sum": 0.0,
            "delay_min": None,
            "delay_max": None,
        }

        self.fp = None
        self.record_path = None

        # ===== 创建输出路径 =====
        self._ensure_dir(self.record_dir)
        self.base_stem = self._compose_base_stem()
        self.summary_path = os.path.join(self.record_dir, self.base_stem + ".summary.txt")
        self._open_new_file(rotate_idx=0)

        rospy.logwarn("======================================")
        rospy.logwarn("[scan_raw_recorder] node started (diagnostic)")
        rospy.logwarn("  scan_topic        : %s", self.scan_topic)
        rospy.logwarn("  record_path       : %s", self.record_path)
        rospy.logwarn("  summary_path      : %s", self.summary_path)
        rospy.logwarn("  full_ranges       : %s", str(self.record_full_ranges))
        rospy.logwarn("  log_every_n       : %d", self.log_every_n)
        rospy.logwarn("  flush_every_n     : %d", self.flush_every_n)
        rospy.logwarn("  max_file_size_mb  : %.1f (0 = unlimited)", self.max_file_size_mb)
        rospy.logwarn("======================================")

        self.sub = rospy.Subscriber(self.scan_topic, LaserScan,
                                    self.scan_callback, queue_size=10)

    # ------------------------------------------------------------------
    # 文件管理
    # ------------------------------------------------------------------
    @staticmethod
    def _ensure_dir(path):
        try:
            if not os.path.exists(path):
                os.makedirs(path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                rospy.logerr("[scan_raw_recorder] makedirs failed for %s: %s", path, str(e))
                raise

    def _compose_base_stem(self):
        stem = self.record_basename
        if self.use_launch_timestamp:
            stem = "%s__%s" % (stem, self.launch_timestamp)
        return stem

    def _open_new_file(self, rotate_idx):
        if rotate_idx > 0:
            fname = "%s.part%d.jsonl" % (self.base_stem, rotate_idx)
        else:
            fname = self.base_stem + ".jsonl"
        self.record_path = os.path.join(self.record_dir, fname)
        try:
            self.fp = open(self.record_path, "a")
        except IOError as e:
            rospy.logerr("[scan_raw_recorder] open file failed: %s (%s)",
                         self.record_path, str(e))
            self.fp = None
            raise

    def _maybe_rotate(self):
        if self.max_file_size_mb <= 0.0 or self.fp is None:
            return
        try:
            size_bytes = os.path.getsize(self.record_path)
        except OSError:
            return
        if size_bytes < self.max_file_size_mb * 1024 * 1024:
            return

        old_path = self.record_path
        self.rotate_counter += 1
        try:
            self.fp.close()
        except Exception:
            pass
        self._open_new_file(rotate_idx=self.rotate_counter)
        rospy.logwarn("[scan_raw_recorder] rotated: %s -> %s", old_path, self.record_path)

    # ------------------------------------------------------------------
    # 数值工具
    # ------------------------------------------------------------------
    @staticmethod
    def safe_value(value):
        """将特殊浮点值转换成可写入 JSON 的形式(inf/nan 用字符串表示)。"""
        try:
            if value is None:
                return None
            if math.isnan(value):
                return "nan"
            if math.isinf(value):
                return "inf" if value > 0 else "-inf"
            return float(value)
        except Exception:
            return str(value)

    @staticmethod
    def _format_time(sec):
        """把 float 秒-自-epoch 格式化为人类可读串,便于 tail / less 眼看对齐。"""
        try:
            if sec is None:
                return None
            whole = int(sec)
            frac = sec - whole
            base = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(whole))
            return "%s.%06d" % (base, int(round(frac * 1e6)))
        except Exception:
            return str(sec)

    @staticmethod
    def get_angle_value(ranges, angle_min, angle_increment, target_deg):
        if not ranges or angle_increment == 0.0:
            return None
        target_rad = math.radians(target_deg)
        idx = int(round((target_rad - angle_min) / angle_increment))
        idx = max(0, min(len(ranges) - 1, idx))
        return ranges[idx]

    # ------------------------------------------------------------------
    # 单帧摘要
    # ------------------------------------------------------------------
    def summarize_scan(self, msg, recv_time_sec, wall_time_sec):
        ranges = list(msg.ranges)
        n = len(ranges)

        valid_count = 0
        inf_count   = 0
        nan_count   = 0
        neg_count   = 0
        under_count = 0
        over_count  = 0
        finite_values = []

        for r in ranges:
            if r is None:
                nan_count += 1
                continue
            if math.isnan(r):
                nan_count += 1
            elif math.isinf(r):
                inf_count += 1
            elif r < 0.0:
                neg_count += 1
            elif r < msg.range_min:
                under_count += 1
            elif r > msg.range_max:
                over_count += 1
            else:
                valid_count += 1
                finite_values.append(r)

        # 累积统计(只对真正写入的帧累积)
        self.cumulative["total_msgs"]   += 1
        self.cumulative["total_ranges"] += n
        self.cumulative["valid"]        += valid_count
        self.cumulative["inf"]          += inf_count
        self.cumulative["nan"]          += nan_count
        self.cumulative["neg"]          += neg_count
        self.cumulative["under"]        += under_count
        self.cumulative["over"]         += over_count

        # 时间对齐:源时间戳、本地接收时间、墙钟、传输延迟
        stamp_sec = msg.header.stamp.to_sec()
        delay_sec = recv_time_sec - stamp_sec
        self.cumulative["delay_sum"] += delay_sec
        if (self.cumulative["delay_min"] is None) or (delay_sec < self.cumulative["delay_min"]):
            self.cumulative["delay_min"] = delay_sec
        if (self.cumulative["delay_max"] is None) or (delay_sec > self.cumulative["delay_max"]):
            self.cumulative["delay_max"] = delay_sec

        sample_deg = {}
        for deg in SAMPLE_ANGLES_DEG:
            sample_deg[str(deg)] = self.safe_value(
                self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, deg)
            )

        summary = {
            "seq":             self.msg_count,
            # --- 时间对齐字段(跨节点 JOIN 以 stamp 为准) ---
            "stamp":               stamp_sec,                           # 源时间(sensor header),跨节点 JOIN 键
            "stamp_str":           self._format_time(stamp_sec),        # 源时间的人类可读版
            "recv_time":           recv_time_sec,                        # 本节点回调触发时的 ROS 时间
            "wall_time":           wall_time_sec,                        # 系统墙钟,独立于 ROS 时钟
            "transport_delay_ms":  delay_sec * 1000.0,                   # 传输延迟(毫秒)
            # --- 扫描元数据 ---
            "frame_id":        msg.header.frame_id,
            "count":           n,
            "angle_min":       msg.angle_min,
            "angle_max":       msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment":  msg.time_increment,
            "scan_time":       msg.scan_time,
            "range_min":       msg.range_min,
            "range_max":       msg.range_max,
            # --- 分类计数 ---
            "valid_count":     valid_count,
            "inf_count":       inf_count,
            "nan_count":       nan_count,
            "neg_count":       neg_count,
            "under_count":     under_count,
            "over_count":      over_count,
            "min_valid":       self.safe_value(min(finite_values)) if finite_values else None,
            "max_valid":       self.safe_value(max(finite_values)) if finite_values else None,
            "sample_deg":      sample_deg,
        }

        if self.record_full_ranges:
            summary["ranges"] = [self.safe_value(x) for x in ranges]

        return summary

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        self.msg_count += 1

        if self.log_every_n > 1 and (self.msg_count % self.log_every_n != 0):
            return
        if self.fp is None:
            return

        # 尽量在进入 I/O 之前先抓接收时间,减少 summarize 本身的计算偏差
        recv_time_sec = rospy.Time.now().to_sec()
        wall_time_sec = time.time()

        summary = self.summarize_scan(msg, recv_time_sec, wall_time_sec)

        try:
            self.fp.write(json.dumps(summary, ensure_ascii=False) + "\n")
            self.write_count += 1
            if self.write_count % self.flush_every_n == 0:
                self.fp.flush()
        except IOError as e:
            rospy.logerr_throttle(
                2.0,
                "[scan_raw_recorder] write failed: %s (%s). Stopping further writes.",
                self.record_path, str(e),
            )
            try:
                self.fp.close()
            except Exception:
                pass
            self.fp = None
            return

        self._maybe_rotate()

        if self.print_summary:
            rospy.logwarn_throttle(
                1.0,
                "[scan_raw_recorder] seq=%d stamp=%.3f delay=%.1fms count=%d valid=%d inf=%d nan=%d neg=%d under=%d over=%d 0deg=%s",
                self.msg_count,
                summary["stamp"],
                summary["transport_delay_ms"],
                summary["count"],
                summary["valid_count"],
                summary["inf_count"],
                summary["nan_count"],
                summary["neg_count"],
                summary["under_count"],
                summary["over_count"],
                str(summary["sample_deg"]["0"]),
            )

    # ------------------------------------------------------------------
    # 关停:写累积 summary
    # ------------------------------------------------------------------
    def _write_aggregate_summary(self):
        try:
            with open(self.summary_path, "w") as sfp:
                sfp.write("scan_raw_recorder summary\n")
                sfp.write("=========================\n")
                sfp.write("launch_timestamp : %s\n" % self.launch_timestamp)
                sfp.write("scan_topic       : %s\n" % self.scan_topic)
                sfp.write("record_dir       : %s\n" % self.record_dir)
                sfp.write("record_basename  : %s\n" % self.record_basename)
                sfp.write("rotate_parts     : %d\n" % self.rotate_counter)
                sfp.write("-------------------------\n")

                total_msgs   = self.cumulative["total_msgs"]
                total_ranges = self.cumulative["total_ranges"]
                sfp.write("received_msgs    : %d\n" % self.msg_count)
                sfp.write("recorded_msgs    : %d\n" % total_msgs)
                sfp.write("total_ranges     : %d\n" % total_ranges)
                sfp.write("-------------------------\n")

                if total_ranges > 0:
                    sfp.write("classification (per-beam, across whole run):\n")
                    for key in ["valid", "inf", "nan", "neg", "under", "over"]:
                        c = self.cumulative[key]
                        pct = 100.0 * c / total_ranges
                        sfp.write("  %-6s : %12d  (%7.4f %%)\n" % (key, c, pct))

                    # 对后续 invalid_fill 策略的意义提示
                    abnormal = (self.cumulative["inf"] + self.cumulative["nan"]
                                + self.cumulative["neg"] + self.cumulative["over"])
                    abnormal_pct = 100.0 * abnormal / total_ranges
                    sfp.write("-------------------------\n")
                    sfp.write("abnormal (inf+nan+neg+over) : %d  (%.4f %%)\n"
                              % (abnormal, abnormal_pct))
                    sfp.write("note: these will be mapped to invalid_fill (0.0 m by default)\n")
                    sfp.write("      in lidar_obs_single_frame downstream.\n")
                else:
                    sfp.write("  (no ranges recorded)\n")

                # --- 传输延迟统计(跨节点时序诊断) ---
                if total_msgs > 0 and self.cumulative["delay_min"] is not None:
                    mean_delay = self.cumulative["delay_sum"] / total_msgs
                    sfp.write("-------------------------\n")
                    sfp.write("transport delay (recv_time - header.stamp, whole run):\n")
                    sfp.write("  min  : %8.2f ms\n" % (self.cumulative["delay_min"] * 1000.0))
                    sfp.write("  mean : %8.2f ms\n" % (mean_delay * 1000.0))
                    sfp.write("  max  : %8.2f ms\n" % (self.cumulative["delay_max"] * 1000.0))
                    sfp.write("note: for cross-node JOIN, use 'stamp' field (sensor header time).\n")
                    sfp.write("      'recv_time' is this node's local ROS clock at callback entry.\n")

            rospy.logwarn("[scan_raw_recorder] aggregate summary written: %s",
                          self.summary_path)
        except IOError as e:
            rospy.logerr("[scan_raw_recorder] aggregate summary write failed: %s",
                         str(e))

    def shutdown(self):
        try:
            if self.fp is not None:
                self.fp.flush()
                self.fp.close()
                rospy.logwarn("[scan_raw_recorder] file closed: %s", self.record_path)
        except Exception as e:
            rospy.logerr("[scan_raw_recorder] close file failed: %s", str(e))
        self._write_aggregate_summary()


if __name__ == "__main__":
    rospy.init_node("scan_raw_recorder")
    node = ScanRawRecorder()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
