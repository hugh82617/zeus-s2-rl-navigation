#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import json
import math
import threading
import rospy
from std_msgs.msg import Float32MultiArray


class Lidar543Recorder(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/rl/lidar_543")
        self.ids_topic = rospy.get_param("~ids_topic", "/rl/lidar_543_ids")
        self.ids_vis_topic = rospy.get_param("~ids_vis_topic", "/rl/lidar_543_ids_vis")

        self.record_dir = rospy.get_param("~record_dir", "/home/ros/lidar/record")
        self.record_file = rospy.get_param("~record_file", "lidar_543_log.jsonl")
        self.record_full_data = bool(rospy.get_param("~record_full_data", True))
        self.log_every_n = int(rospy.get_param("~log_every_n", 1))
        self.print_summary = bool(rospy.get_param("~print_summary", True))
        self.expected_len = int(rospy.get_param("~expected_len", 543))
        self.segment_len = int(rospy.get_param("~segment_len", 181))

        self.msg_count = 0
        self.fp = None
        self.lock = threading.Lock()
        self.latest_ids = None
        self.latest_ids_vis = None

        if not os.path.exists(self.record_dir):
            os.makedirs(self.record_dir)

        self.record_path = os.path.join(self.record_dir, self.record_file)
        self.fp = open(self.record_path, "a")

        rospy.logwarn("======================================")
        rospy.logwarn(" lidar_543_recorder node started")
        rospy.logwarn(" input_topic    : %s", self.input_topic)
        rospy.logwarn(" ids_topic      : %s", self.ids_topic)
        rospy.logwarn(" ids_vis_topic  : %s", self.ids_vis_topic)
        rospy.logwarn(" record_path    : %s", self.record_path)
        rospy.logwarn(" full_data      : %s", str(self.record_full_data))
        rospy.logwarn(" expected_len   : %d", self.expected_len)
        rospy.logwarn(" segment_len    : %d", self.segment_len)
        rospy.logwarn(" log_every_n    : %d", self.log_every_n)
        rospy.logwarn("======================================")

        self.sub_main = rospy.Subscriber(self.input_topic, Float32MultiArray, self.main_callback, queue_size=1)
        self.sub_ids = rospy.Subscriber(self.ids_topic, Float32MultiArray, self.ids_callback, queue_size=1)
        self.sub_ids_vis = rospy.Subscriber(self.ids_vis_topic, Float32MultiArray, self.ids_vis_callback, queue_size=1)

    @staticmethod
    def _safe_float(value):
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

    def ids_callback(self, msg):
        with self.lock:
            self.latest_ids = list(msg.data)

    def ids_vis_callback(self, msg):
        with self.lock:
            self.latest_ids_vis = list(msg.data)

    def _segment_stats(self, data, start, end):
        seg = data[start:end]
        finite = [x for x in seg if isinstance(x, (int, float)) and not math.isnan(x) and not math.isinf(x)]
        return {
            "length": len(seg),
            "min": self._safe_float(min(finite) if finite else None),
            "max": self._safe_float(max(finite) if finite else None),
            "mean": self._safe_float(sum(finite) / float(len(finite)) if finite else None),
            "first": self._safe_float(seg[0] if len(seg) > 0 else None),
            "center": self._safe_float(seg[len(seg) // 2] if len(seg) > 0 else None),
            "last": self._safe_float(seg[-1] if len(seg) > 0 else None),
        }

    def summarize(self, msg):
        data = list(msg.data)
        n = len(data)
        finite = [x for x in data if isinstance(x, (int, float)) and not math.isnan(x) and not math.isinf(x)]

        with self.lock:
            ids = list(self.latest_ids) if self.latest_ids is not None else None
            ids_vis = list(self.latest_ids_vis) if self.latest_ids_vis is not None else None

        summary = {
            "stamp": rospy.Time.now().to_sec(),
            "length": n,
            "expected_len": self.expected_len,
            "length_ok": (n == self.expected_len),
            "min": self._safe_float(min(finite) if finite else None),
            "max": self._safe_float(max(finite) if finite else None),
            "mean": self._safe_float(sum(finite) / float(len(finite)) if finite else None),
            "layout_dim": [
                {
                    "label": d.label,
                    "size": d.size,
                    "stride": d.stride,
                }
                for d in msg.layout.dim
            ],
            "ids": [self._safe_float(x) for x in ids] if ids is not None else None,
            "ids_vis_head": [self._safe_float(x) for x in ids_vis[: min(12, len(ids_vis))]] if ids_vis is not None else None,
            "t0": self._segment_stats(data, 0, self.segment_len),
            "t1": self._segment_stats(data, self.segment_len, 2 * self.segment_len),
            "t2": self._segment_stats(data, 2 * self.segment_len, 3 * self.segment_len),
        }

        if self.record_full_data:
            summary["data"] = [self._safe_float(x) for x in data]
            if ids_vis is not None:
                summary["ids_vis"] = [self._safe_float(x) for x in ids_vis]
        return summary

    def main_callback(self, msg):
        self.msg_count += 1
        if self.log_every_n > 1 and (self.msg_count % self.log_every_n != 0):
            return

        summary = self.summarize(msg)
        self.fp.write(json.dumps(summary, ensure_ascii=False) + "\n")
        self.fp.flush()

        if self.print_summary:
            rospy.logwarn_throttle(
                1.0,
                "[lidar_543_recorder] seq=%d len=%d ok=%s ids=%s | t0_center=%s t1_center=%s t2_center=%s",
                self.msg_count,
                summary["length"],
                str(summary["length_ok"]),
                str(summary["ids"]),
                str(summary["t0"]["center"]),
                str(summary["t1"]["center"]),
                str(summary["t2"]["center"])
            )

    def shutdown(self):
        try:
            if self.fp is not None:
                self.fp.close()
                rospy.logwarn("[lidar_543_recorder] file closed: %s", self.record_path)
        except Exception as e:
            rospy.logerr("[lidar_543_recorder] close file failed: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("lidar_543_recorder")
    node = Lidar543Recorder()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
