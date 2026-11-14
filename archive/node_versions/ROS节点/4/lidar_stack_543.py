#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 3/3 : lidar_stack_543
============================================================
职责:将 /rl/lidar_181 单帧观测按时间序列堆叠为 543 维输入,
     供策略网络的 LiDAR 通道直接消费。

输入 : /rl/lidar_181       (std_msgs/Float32MultiArray, len = 181)

主输出 : /rl/lidar_543     (std_msgs/Float32MultiArray, len = 543)
         数据顺序严格为 [t0 | t1 | t2] (当前帧 | 上一帧 | 上上帧)

诊断输出:
  /rl/lidar_543_ids     (len = 3)     [t0_id, t1_id, t2_id]
  /rl/lidar_543_ids_vis (len = 543)   [t0_id]*181 | [t1_id]*181 | [t2_id]*181
  -- 用于在 rostopic echo 中直观验证三段编号边界。

初始化策略:
  首帧到达时,t0 = t1 = t2 = cur,所有 id 均为 cur_id。
  之后每一帧:t2 <- t1 <- t0 <- cur(id 同步滚动)。

管线定位:LiDAR 观测预处理链第 3 级,上游 lidar_obs_single_frame,
         下游为 rl_planner_node(obs 拼接 + ONNX 推理)。
============================================================
"""

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


# 与 lidar_obs_single_frame 对齐的链路常量
LIDAR_NUM_BEAMS = 181
LIDAR_STACK_LEN = 3
LIDAR_STACKED_DIM = LIDAR_NUM_BEAMS * LIDAR_STACK_LEN  # 543


class LidarStack543Node(object):
    def __init__(self):
        # --- 话题 ---
        self.input_topic   = rospy.get_param("~input_topic",   "/rl/lidar_181")
        self.output_topic  = rospy.get_param("~output_topic",  "/rl/lidar_543")
        self.ids_topic     = rospy.get_param("~ids_topic",     "/rl/lidar_543_ids")
        self.ids_vis_topic = rospy.get_param("~ids_vis_topic", "/rl/lidar_543_ids_vis")

        # --- 参数 ---
        self.expected_len   = int (rospy.get_param("~expected_len", LIDAR_NUM_BEAMS))
        self.publish_debug  = bool(rospy.get_param("~publish_debug_log", True))

        # --- 滑窗状态 ---
        self.t0 = None
        self.t1 = None
        self.t2 = None
        self.t0_id = None
        self.t1_id = None
        self.t2_id = None

        self.msg_count = 0
        self.frame_id_counter = 0

        # --- 发布/订阅 ---
        self.pub         = rospy.Publisher(self.output_topic,  Float32MultiArray, queue_size=10)
        self.pub_ids     = rospy.Publisher(self.ids_topic,     Float32MultiArray, queue_size=10)
        self.pub_ids_vis = rospy.Publisher(self.ids_vis_topic, Float32MultiArray, queue_size=10)

        self.sub = rospy.Subscriber(self.input_topic, Float32MultiArray,
                                    self.cb, queue_size=10)

        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_stack_543] started (pipeline stage 3/3)")
        rospy.logwarn("  input_topic    = %s", self.input_topic)
        rospy.logwarn("  output_topic   = %s (len=%d)", self.output_topic, LIDAR_STACKED_DIM)
        rospy.logwarn("  ids_topic      = %s", self.ids_topic)
        rospy.logwarn("  ids_vis_topic  = %s", self.ids_vis_topic)
        rospy.logwarn("  expected_len   = %d", self.expected_len)
        rospy.logwarn("======================================")

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _validate_frame(self, data):
        if len(data) != self.expected_len:
            rospy.logerr_throttle(
                1.0,
                "[lidar_stack_543] input length error: got %d, expected %d",
                len(data), self.expected_len,
            )
            return False
        return True

    @staticmethod
    def _build_msg(data, label):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = len(data)
        dim.stride = len(data)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = data
        return msg

    def reset(self):
        """外部可调用:将 t1/t2 重置为当前 t0(例如从静止重启时)。"""
        if self.t0 is not None:
            self.t1 = list(self.t0)
            self.t2 = list(self.t0)
            self.t1_id = self.t0_id
            self.t2_id = self.t0_id
            rospy.logwarn(
                "[lidar_stack_543] reset: refilled t1/t2 with current t0, ids=[%s,%s,%s]",
                str(self.t0_id), str(self.t1_id), str(self.t2_id),
            )

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def cb(self, msg):
        cur = list(msg.data)
        if not self._validate_frame(cur):
            return

        self.frame_id_counter += 1
        cur_id = self.frame_id_counter

        if self.t0 is None:
            # 首帧:用 cur 填满整个滑窗
            self.t0 = list(cur); self.t1 = list(cur); self.t2 = list(cur)
            self.t0_id = cur_id; self.t1_id = cur_id; self.t2_id = cur_id
        else:
            # 滑窗滚动:t2 <- t1 <- t0 <- cur
            self.t2 = list(self.t1); self.t2_id = self.t1_id
            self.t1 = list(self.t0); self.t1_id = self.t0_id
            self.t0 = list(cur);     self.t0_id = cur_id

        # 主输出:[t0 | t1 | t2]
        stacked = self.t0 + self.t1 + self.t2
        self.pub.publish(self._build_msg(stacked, "lidar_543"))

        # 诊断输出 1:三段编号
        ids = [float(self.t0_id), float(self.t1_id), float(self.t2_id)]
        self.pub_ids.publish(self._build_msg(ids, "lidar_543_ids"))

        # 诊断输出 2:543 维编号可视化
        ids_vis = (
            [float(self.t0_id)] * self.expected_len +
            [float(self.t1_id)] * self.expected_len +
            [float(self.t2_id)] * self.expected_len
        )
        self.pub_ids_vis.publish(self._build_msg(ids_vis, "lidar_543_ids_vis"))

        self.msg_count += 1
        if self.publish_debug:
            rospy.logwarn(
                "[lidar_stack_543] recv #%d -> stack ids [t0,t1,t2]=[%d,%d,%d]",
                cur_id, self.t0_id, self.t1_id, self.t2_id,
            )


if __name__ == "__main__":
    rospy.init_node("lidar_stack_543_node")
    LidarStack543Node()
    rospy.spin()
