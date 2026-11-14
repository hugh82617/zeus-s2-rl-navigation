#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 3/3 : lidar_stack_543  (v2)
============================================================
职责:将 /rl/lidar_181 单帧观测按时间序列堆叠为 543 维输入,供策略网络
     的 LiDAR 通道直接消费。v2 在 v1 的三帧堆叠骨架基础上新增:
       (1) 三帧时间戳诊断输出 /rl/lidar_543_stamps
       (2) 仅对"被修补的 beam"做三帧中位数时域稳态
     用于抑制修补误差在 543 维上连续放大。

---------------------------------------------------------------
输入主话题:
  /rl/lidar_181               (std_msgs/Float32MultiArray, len = 181)
                              触发主回调。
输入伴随话题(v2 新增订阅):
  /rl/lidar_181_stamp         (std_msgs/Header)
      来自第 2 级,携带源 /scan_aligned 的 header.stamp。
      用于拼出 /rl/lidar_543_stamps(三帧时间诊断)。
  /rl/lidar_181_repair_mask   (std_msgs/Float32MultiArray, len = 181)
      来自第 2 级,逐 beam 的 0/1 掩码。
      决定本帧哪些 beam 进入"时域稳态"处理。
  /rl/lidar_181_fill_stats    (std_msgs/Float32MultiArray, len = 4)
      仅用于调试日志,不进主链路。
      注意(v2_1): 该话题在第 2 级发布顺序里排在 lidar_181 之后,因此第 3 级
                  主回调触发时它可能尚未到达,popleft() 取到的可能是上一帧
                  或队列为空。仅作为近似调试观察,不保证与当前帧严格同帧
                  配对,也不参与主链路数据处理。

---------------------------------------------------------------
主输出:
  /rl/lidar_543               (std_msgs/Float32MultiArray, len = 543)
      严格 [t0 | t1 | t2] 顺序(当前帧 | 上一帧 | 上上帧)。

诊断输出(v1 保留):
  /rl/lidar_543_ids           (len = 3)     [t0_id, t1_id, t2_id]
  /rl/lidar_543_ids_vis       (len = 543)   [t0_id]*181 | [t1_id]*181 | [t2_id]*181

诊断输出(v2 新增):
  /rl/lidar_543_stamps        (std_msgs/Float64MultiArray, len = 3)
      [t0_sec, t1_sec, t2_sec]
      - 数值为源 /scan_aligned 的 header.stamp.to_sec()(Unix 秒),
        使用 Float64 以保留微秒级精度(Float32 只有 ~7 位有效数字,
        不足以表达完整的 Unix 秒 + 小数)。
      - 用于离线核验:三帧是否时间单调、是否存在重复帧/乱序帧/跳帧。

---------------------------------------------------------------
时域稳态策略(文档第三节(三)第 3 条):
  对原始有效 beam : 保留 cur[k](不动)
  对修补 beam     : cur[k] <- median(cur[k], t1[k], t2[k])
                     其中 t1/t2 是滑窗中的上一帧/上上帧同位 beam。
  说明:
    - 稳态只作用在 t0 进入滑窗之前,不回溯修改 t1/t2。
    - t1/t2 本身可能已是"历史稳态值"(如上一帧该 beam 也被修补过),
      这在持续修补区域构成递归稳态,有助于进一步抑制抖动。
    - 对动态环境变化完全没有影响——因为 mask==0 的 beam 绕过整个路径。

---------------------------------------------------------------
三路伴随话题的 FIFO 对齐:
  第 2 级 v3 的发布顺序(补丁后)是:
      stamp -> src_map -> repair_mask -> lidar_181 -> fill_stats
  因此第 3 级在 /rl/lidar_181 主回调被触发时,同帧的 stamp 与 mask
  通常已经在本地队列里就绪。本节点用三个 collections.deque FIFO 队列,
  主回调 popleft 取当前帧对应的 stamp 与 mask。
  若某路队列在主回调时仍为空(网络乱序、启动瞬态),降级行为为:
    - stamp 缺失 : 退回 rospy.Time.now(),throttled 警告
    - mask 缺失  : 本帧不做稳态,直接用 cur,throttled 警告
    - fs   缺失  : 本帧调试日志少一些字段,不警告

---------------------------------------------------------------
CHANGELOG v1 -> v2:
  [新增] 订阅 /rl/lidar_181_stamp,发布 /rl/lidar_543_stamps (Float64)
  [新增] 订阅 /rl/lidar_181_repair_mask,对修补 beam 做三帧中位数稳态
  [新增] 订阅 /rl/lidar_181_fill_stats,仅供调试日志
  [新增] 三路伴随话题 FIFO 队列,与主话题 popleft 对齐
  [新增] _median3() / _median3_robust() 小工具
  [保留] /rl/lidar_543, /rl/lidar_543_ids, /rl/lidar_543_ids_vis 的语义
  [保留] [t0|t1|t2] 堆叠顺序、滑窗滚动逻辑、首帧填充语义
  [参数] 新增 ~enable_repair_stabilize (默认 True),~aux_queue_maxlen (默认 10)
         可通过 launch 禁用时域稳态,用于 A/B 对比实验。

CHANGELOG v2 -> v2_1 (上机前诊断增强):
  [新增] /rl/lidar_543_stamps 发布后即时做 t0>=t1>=t2 单调性自检,
         一旦乱序立刻 throttled logwarn,便于上机时第一时间发现
         FIFO 对齐异常/网络乱序/队列堆积。计入 stamp_nonmono_frames
         统计并周期打印。
  [文档] 明确 fill_stats 不与 lidar_181 严格同帧(方案 A):保留现有
         FIFO popleft 逻辑,不改第 2 级发布顺序,仅在注释/日志中
         标注"fill_stats 为近似调试",避免引入新的时序副作用。

管线定位:LiDAR 观测预处理链第 3 级,上游 lidar_obs_single_frame(v3),
         下游为 rl_planner_node(obs 拼接 + ONNX 推理)。
============================================================
"""

from __future__ import division, print_function

import threading
from collections import deque

import rospy
from std_msgs.msg import (
    Float32MultiArray,
    Float64MultiArray,
    MultiArrayDimension,
    Header,
)


# 与 lidar_obs_single_frame v3 对齐的链路常量
LIDAR_NUM_BEAMS   = 181
LIDAR_STACK_LEN   = 3
LIDAR_STACKED_DIM = LIDAR_NUM_BEAMS * LIDAR_STACK_LEN   # 543

# 伴随话题 FIFO 队列长度上限
AUX_QUEUE_MAXLEN_DEFAULT = 10


class LidarStack543Node(object):
    def __init__(self):
        # ---------- 主话题 ----------
        self.input_topic   = rospy.get_param("~input_topic",   "/rl/lidar_181")
        self.output_topic  = rospy.get_param("~output_topic",  "/rl/lidar_543")
        self.ids_topic     = rospy.get_param("~ids_topic",     "/rl/lidar_543_ids")
        self.ids_vis_topic = rospy.get_param("~ids_vis_topic", "/rl/lidar_543_ids_vis")

        # ---------- v2 新增伴随话题 ----------
        self.stamp_in_topic       = rospy.get_param("~stamp_in_topic",       "/rl/lidar_181_stamp")
        self.repair_mask_in_topic = rospy.get_param("~repair_mask_in_topic", "/rl/lidar_181_repair_mask")
        self.fill_stats_in_topic  = rospy.get_param("~fill_stats_in_topic",  "/rl/lidar_181_fill_stats")
        self.stamps_out_topic     = rospy.get_param("~stamps_out_topic",     "/rl/lidar_543_stamps")

        # ---------- 参数 ----------
        self.expected_len           = int (rospy.get_param("~expected_len",           LIDAR_NUM_BEAMS))
        self.publish_debug          = bool(rospy.get_param("~publish_debug_log",      True))
        # v2 新增配置
        self.enable_repair_stabilize = bool(rospy.get_param("~enable_repair_stabilize", True))
        self.aux_queue_maxlen        = int (rospy.get_param("~aux_queue_maxlen",        AUX_QUEUE_MAXLEN_DEFAULT))

        # ---------- 滑窗状态 ----------
        self.t0 = None;        self.t1 = None;        self.t2 = None
        self.t0_id = None;     self.t1_id = None;     self.t2_id = None
        self.t0_stamp = None;  self.t1_stamp = None;  self.t2_stamp = None

        self.msg_count = 0
        self.frame_id_counter = 0

        # v2 统计
        self.stab_applied_frames = 0      # 累计做过稳态的帧数
        self.stab_skipped_frames = 0      # mask 空队列导致未稳态的帧数(非首帧)
        self.stamp_fallback_frames = 0    # 用 rospy.Time.now() 兜底的帧数(非首帧)
        # v2_1 统计
        self.stamp_nonmono_frames = 0     # t0>=t1>=t2 违反的累计帧数

        # ---------- v2 FIFO 队列(伴随话题缓冲) ----------
        self.lock = threading.Lock()
        self._stamp_queue = deque(maxlen=self.aux_queue_maxlen)     # element: rospy.Time
        self._mask_queue  = deque(maxlen=self.aux_queue_maxlen)     # element: list[int] len=181
        self._fs_queue    = deque(maxlen=self.aux_queue_maxlen)     # element: list[float] len=4

        # ---------- 发布 ----------
        self.pub         = rospy.Publisher(self.output_topic,      Float32MultiArray, queue_size=10)
        self.pub_ids     = rospy.Publisher(self.ids_topic,         Float32MultiArray, queue_size=10)
        self.pub_ids_vis = rospy.Publisher(self.ids_vis_topic,     Float32MultiArray, queue_size=10)
        self.pub_stamps  = rospy.Publisher(self.stamps_out_topic,  Float64MultiArray, queue_size=10)

        # ---------- 订阅 ----------
        # 注意:先建伴随订阅,最后建主订阅,尽量保证伴随消息到达的先手优势。
        self.sub_stamp = rospy.Subscriber(
            self.stamp_in_topic, Header, self._stamp_cb,
            queue_size=20, tcp_nodelay=True,
        )
        self.sub_mask = rospy.Subscriber(
            self.repair_mask_in_topic, Float32MultiArray, self._mask_cb,
            queue_size=20, tcp_nodelay=True,
        )
        self.sub_fs = rospy.Subscriber(
            self.fill_stats_in_topic, Float32MultiArray, self._fs_cb,
            queue_size=20, tcp_nodelay=True,
        )
        self.sub = rospy.Subscriber(
            self.input_topic, Float32MultiArray, self.cb,
            queue_size=10, tcp_nodelay=True,
        )

        rospy.logwarn("======================================")
        rospy.logwarn("[lidar_stack_543 v2] started (pipeline stage 3/3)")
        rospy.logwarn("  input_topic              = %s", self.input_topic)
        rospy.logwarn("  output_topic             = %s (len=%d)",
                      self.output_topic, LIDAR_STACKED_DIM)
        rospy.logwarn("  ids_topic                = %s", self.ids_topic)
        rospy.logwarn("  ids_vis_topic            = %s", self.ids_vis_topic)
        rospy.logwarn("  stamp_in_topic           = %s  [v2 NEW sub]", self.stamp_in_topic)
        rospy.logwarn("  repair_mask_in_topic     = %s  [v2 NEW sub]", self.repair_mask_in_topic)
        rospy.logwarn("  fill_stats_in_topic      = %s  [v2 NEW sub, debug only]",
                      self.fill_stats_in_topic)
        rospy.logwarn("  stamps_out_topic         = %s (len=3, Float64)  [v2 NEW pub]",
                      self.stamps_out_topic)
        rospy.logwarn("  expected_len             = %d", self.expected_len)
        rospy.logwarn("  enable_repair_stabilize  = %s", str(self.enable_repair_stabilize))
        rospy.logwarn("  aux_queue_maxlen         = %d", self.aux_queue_maxlen)
        rospy.logwarn("======================================")

    # ==================================================================
    # 小工具
    # ==================================================================
    def _validate_frame(self, data):
        if len(data) != self.expected_len:
            rospy.logerr_throttle(
                1.0,
                "[lidar_stack_543 v2] input length error: got %d, expected %d",
                len(data), self.expected_len,
            )
            return False
        return True

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

    @staticmethod
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

    @staticmethod
    def _median3(a, b, c):
        """Median of 3 floats (branch-only, no sort allocation)."""
        if a > b:
            a, b = b, a
        if b > c:
            b, c = c, b
        if a > b:
            a, b = b, a
        return b

    # ==================================================================
    # 伴随话题回调(仅入队,不做重负载)
    # ==================================================================
    def _stamp_cb(self, msg):
        with self.lock:
            self._stamp_queue.append(msg.stamp)

    def _mask_cb(self, msg):
        # 181 维 0/1 掩码。消息内是 Float,这里转 int 供下游快速判断。
        if len(msg.data) != self.expected_len:
            rospy.logwarn_throttle(
                2.0,
                "[lidar_stack_543 v2] repair_mask length mismatch: got %d, expected %d",
                len(msg.data), self.expected_len,
            )
            return
        mask = [1 if v >= 0.5 else 0 for v in msg.data]
        with self.lock:
            self._mask_queue.append(mask)

    def _fs_cb(self, msg):
        if len(msg.data) < 4:
            return
        fs = list(msg.data[:4])
        with self.lock:
            self._fs_queue.append(fs)

    # ==================================================================
    # 外部 reset(从静止重启时可调用)
    # ==================================================================
    def reset(self):
        with self.lock:
            if self.t0 is not None:
                self.t1 = list(self.t0);       self.t1_id = self.t0_id
                self.t2 = list(self.t0);       self.t2_id = self.t0_id
                self.t1_stamp = self.t0_stamp
                self.t2_stamp = self.t0_stamp
                rospy.logwarn(
                    "[lidar_stack_543 v2] reset: refilled t1/t2 with current t0, "
                    "ids=[%s,%s,%s]",
                    str(self.t0_id), str(self.t1_id), str(self.t2_id),
                )

    # ==================================================================
    # 主回调
    # ==================================================================
    def cb(self, msg):
        cur = list(msg.data)
        if not self._validate_frame(cur):
            return

        with self.lock:
            self.frame_id_counter += 1
            cur_id = self.frame_id_counter

            # -------- Step 1: 从 FIFO 队列 popleft 对齐的伴随数据 --------
            if self._stamp_queue:
                cur_stamp = self._stamp_queue.popleft()
            else:
                cur_stamp = rospy.Time.now()
                if self.msg_count >= 3:   # 避免启动瞬态连续报警
                    self.stamp_fallback_frames += 1
                    rospy.logwarn_throttle(
                        2.0,
                        "[lidar_stack_543 v2] stamp queue empty, "
                        "falling back to rospy.Time.now() (total=%d frames)",
                        self.stamp_fallback_frames,
                    )

            if self._mask_queue:
                cur_mask = self._mask_queue.popleft()
            else:
                cur_mask = None
                if self.msg_count >= 3:
                    self.stab_skipped_frames += 1
                    rospy.logwarn_throttle(
                        2.0,
                        "[lidar_stack_543 v2] mask queue empty, "
                        "skip stabilization this frame (total=%d frames)",
                        self.stab_skipped_frames,
                    )

            cur_fs = self._fs_queue.popleft() if self._fs_queue else None

            # -------- Step 2: 滑窗滚动 + 时域稳态 --------
            if self.t0 is None:
                # 首帧:用 cur 填满整个滑窗,不做稳态
                self.t0 = list(cur); self.t1 = list(cur); self.t2 = list(cur)
                self.t0_id = cur_id; self.t1_id = cur_id; self.t2_id = cur_id
                self.t0_stamp = cur_stamp
                self.t1_stamp = cur_stamp
                self.t2_stamp = cur_stamp
            else:
                # 先把 t1/t2 向后滚,为新 t0 腾位
                self.t2       = self.t1;        self.t2_id = self.t1_id
                self.t2_stamp = self.t1_stamp
                self.t1       = self.t0;        self.t1_id = self.t0_id
                self.t1_stamp = self.t0_stamp
                # 此时 self.t1 = 帧 N-1 的值, self.t2 = 帧 N-2 的值

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
                # 不管稳态有没有生效,都把新帧装进 t0
                self.t0       = new_t0
                self.t0_id    = cur_id
                self.t0_stamp = cur_stamp

            # -------- Step 3: 主输出 [t0 | t1 | t2] --------
            stacked = self.t0 + self.t1 + self.t2
            self.pub.publish(self._build_f32_msg(stacked, "lidar_543"))

            # -------- Step 4: 诊断输出 --------
            ids = [float(self.t0_id), float(self.t1_id), float(self.t2_id)]
            self.pub_ids.publish(self._build_f32_msg(ids, "lidar_543_ids"))

            ids_vis = (
                [float(self.t0_id)] * self.expected_len +
                [float(self.t1_id)] * self.expected_len +
                [float(self.t2_id)] * self.expected_len
            )
            self.pub_ids_vis.publish(self._build_f32_msg(ids_vis, "lidar_543_ids_vis"))

            # v2 新增:三帧时间戳
            stamps_sec = [
                float(self.t0_stamp.to_sec()),
                float(self.t1_stamp.to_sec()),
                float(self.t2_stamp.to_sec()),
            ]
            self.pub_stamps.publish(self._build_f64_msg(stamps_sec, "lidar_543_stamps"))

            # v2_1 新增:发布后立即做 t0 >= t1 >= t2 单调性自检。
            # 首帧三者相等(t0==t1==t2)属于启动瞬态,不计为违反;此外,第 2
            # 帧 t1 与 t2 也可能相等(t2 还没收到真正的历史),同样不违反。
            # 一旦出现严格逆序(t0<t1 或 t1<t2),计入 stamp_nonmono_frames
            # 并 throttled logwarn,便于上机时第一时间定位 FIFO 乱序。
            if not (stamps_sec[0] >= stamps_sec[1] >= stamps_sec[2]):
                self.stamp_nonmono_frames += 1
                rospy.logwarn_throttle(
                    1.0,
                    "[lidar_stack_543 v2_1] non-monotonic stamps: "
                    "[t0=%.6f, t1=%.6f, t2=%.6f]  (dt01=%+.4f, dt12=%+.4f)  "
                    "total_nonmono=%d",
                    stamps_sec[0], stamps_sec[1], stamps_sec[2],
                    stamps_sec[0] - stamps_sec[1],
                    stamps_sec[1] - stamps_sec[2],
                    self.stamp_nonmono_frames,
                )

            self.msg_count += 1

            # -------- Step 5: 调试日志(throttled) --------
            if self.publish_debug and self.msg_count % 20 == 1:
                if cur_mask is not None:
                    mask_count = sum(cur_mask)
                else:
                    mask_count = -1  # sentinel
                if cur_fs is not None:
                    fs_desc = ("repair_ratio=%.3f front=%d severe=%.3f"
                               % (cur_fs[0], int(cur_fs[2]), cur_fs[3]))
                else:
                    fs_desc = "fs=none"
                rospy.logwarn(
                    "[lidar_stack_543 v2_1] recv #%d  ids=[%d,%d,%d]  "
                    "stamps=[%.3f,%.3f,%.3f]  mask_count=%d  %s  "
                    "stab_applied=%d  stab_skipped=%d  stamp_nonmono=%d",
                    cur_id, self.t0_id, self.t1_id, self.t2_id,
                    stamps_sec[0], stamps_sec[1], stamps_sec[2],
                    mask_count, fs_desc,
                    self.stab_applied_frames, self.stab_skipped_frames,
                    self.stamp_nonmono_frames,
                )


if __name__ == "__main__":
    rospy.init_node("lidar_stack_543_node")
    LidarStack543Node()
    rospy.spin()
