#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
pipeline_verifier_node.py
------------------------------------------------------------
在线 LiDAR 预处理链路验证 + 实验后自动分析报告生成节点。

目标:不再走 "录制 bag -> CSV 导出 -> 离线脚本" 这条流程,
      而是在实验过程中同时做实时校验,实验结束(Ctrl+C 或 rosnode kill)
      后直接在磁盘上生成一份可读的 .txt 分析报告。

订阅话题(13):
  Stage 1:
    /scan                           sensor_msgs/LaserScan
    /scan_aligned                   sensor_msgs/LaserScan
    /scan_aligned_invalid_mask      std_msgs/Float32MultiArray (len=N_raw)
    /scan_aligned_invalid_stats     std_msgs/Float32MultiArray (len=4)
  Stage 2:
    /rl/lidar_181                   std_msgs/Float32MultiArray (len=181)
    /rl/lidar_181_stamp             std_msgs/Header
    /rl/lidar_181_src_map           std_msgs/Float32MultiArray (len=543)
    /rl/lidar_181_repair_mask       std_msgs/Float32MultiArray (len=181)
    /rl/lidar_181_fill_stats        std_msgs/Float32MultiArray (len=4)
  Stage 3:
    /rl/lidar_543                   std_msgs/Float32MultiArray (len=543)
    /rl/lidar_543_ids               std_msgs/Float32MultiArray (len=3)
    /rl/lidar_543_ids_vis           std_msgs/Float32MultiArray (len=543)
    /rl/lidar_543_stamps            std_msgs/Float64MultiArray (len=3)

核验锚点:/rl/lidar_543。每到一帧 543:
    1) 从 FIFO 队列配出本帧的 4 路 Stage 3 伴随数据
    2) 用 /rl/lidar_543_stamps 的 [t0,t1,t2] 作为 key,回查三个
       Stage 2 frame(缓冲字典 self.s2_frames[stamp_key])
    3) 每个 Stage 2 frame 的 stamp 反查同 key 的 Stage 1 frame
       (Stage 1 透传 header.stamp,两者同 key)
    4) 跨级一致性逐项校验,结果追加写入 frames 临时文件
    5) 发现异常即时 logwarn_throttle

Stage 1 / Stage 2 的独立校验在 frame 首次 "5 路 / 4 路全部到齐"
时做一次,结果缓存在 frame 对象上。Stage 3 锚定时只查表,不再重算。

实时告警(rospy.logwarn_throttle):
  - 三帧时间戳反跳
  - 543 缺少 stage 2/stage 1 反向链接
  - repair_ratio 超阈值(默认 0.30)
  - 任一话题 > silence_timeout 秒无新消息(默认 3s)
  - 长度/类型不符、数值越界、mask 非 0/1 等格式异常

报告输出:
  ~/lidar/data/pipeline_report_<timestamp>.txt

  Part 1:
    - 话题接收计数与近似 Hz
    - 三级帧数对齐情况
    - 跨级追溯覆盖率(543 -> stage2 / stage1 / scan)
    - 三级验证通过率
    - 异常类型明细计数
    - 总体结论(PASS / ACCEPTABLE / FAIL)

  Part 2:
    - 每一帧 /rl/lidar_543 的逐帧全链路追溯
    - 三段(t0/t1/t2)各自的 stage 2 / stage 1 指标
    - 该帧各级异常列表(若有)
    - 每帧 verdict

部署位置:~/catkin_ws/src/rl_planner/scripts/pipeline_verifier_node.py
赋权:    chmod +x pipeline_verifier_node.py
启动:    rosrun rl_planner pipeline_verifier_node.py
         或加到 launch:
           <node pkg="rl_planner" type="pipeline_verifier_node.py"
                 name="pipeline_verifier" output="screen"/>

参数(ROS param):
  ~report_dir                 : '~/lidar/data'
  ~buffer_seconds             : 5.0     stage 1/2 frame 缓冲保留秒数
  ~repair_ratio_warn          : 0.30    单帧 repair_ratio 告警阈值
  ~severe_ratio_warn          : 0.10    单帧 severe_ratio 告警阈值
  ~interp_tol_normalized      : 1e-4    stage 2 插值一致性容差(归一化空间)
  ~rotation_check_sample_beams: 20      stage 1 rotation 抽样 beam 数
  ~silence_timeout_seconds    : 3.0     话题静默告警阈值
  ~max_frames_written         : 20000   Part 2 最多记录帧数(上限保护)
  ~enable_stabilize_recheck   : true    stage 3 是否做 median 稳态反推
  ~verdict_thresholds         :
        anomaly_rate_pass     = 0.005   任一异常率 <= 此值 -> 贡献 PASS 判定
        anomaly_rate_acceptable = 0.05  <= 此值 -> 贡献 ACCEPTABLE
Python 2/3 兼容(ROS Melodic / Noetic 均可)。

CHANGELOG v1 -> v1.1 (订阅时序修正):
  [必须] 新增 _s1/_s2/_s3_sync_armed 启动同步屏障:
         - 首条 /scan_aligned 到达时清 invmask/invstats 的 fallback queue
         - 首条 /rl/lidar_181_stamp 到达时清 srcmap/rmask/l181/fstats queue
         - 首条 /rl/lidar_543 到达时清 ids/ids_vis/stamps queue
         原因:rospy 各路 topic 的订阅 TCP 握手不同步,先就绪的伴随 topic
         订阅会抢到锚之前已发布的旧消息,堆在 fallback queue 里。如果不丢弃,
         第 1 帧的配对会跨一两帧错位,并永久保持错位,进而触发海量虚假异常:
           S2/fstats_*_inconsistent (~300 次)
           S2/mask0_but_raw_inf (~350 次)
           S3/t*_block_mismatch (~350 次) [级联]
  [新增] 最终报告新增第 7 节 Startup Sync Barriers,打印三级分别丢弃了多少
         pre-anchor 消息,便于核对屏障是否触发。
============================================================
"""

from __future__ import division, print_function

import math
import os
import sys
import time
import threading
from collections import deque, defaultdict

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Header


# ============================================================
# 常量
# ============================================================
NUM_BEAMS = 181
STACKED_DIM = 543
LIDAR_MAX_DISTANCE = 16.0   # 与 stage 2 v3_2 对齐

# 话题名
T_SCAN              = "/scan"
T_SCAN_ALIGNED      = "/scan_aligned"
T_SCAN_INVMASK      = "/scan_aligned_invalid_mask"
T_SCAN_INVSTATS     = "/scan_aligned_invalid_stats"
T_L181              = "/rl/lidar_181"
T_L181_STAMP        = "/rl/lidar_181_stamp"
T_L181_SRCMAP       = "/rl/lidar_181_src_map"
T_L181_RMASK        = "/rl/lidar_181_repair_mask"
T_L181_FSTATS       = "/rl/lidar_181_fill_stats"
T_L543              = "/rl/lidar_543"
T_L543_IDS          = "/rl/lidar_543_ids"
T_L543_IDSVIS       = "/rl/lidar_543_ids_vis"
T_L543_STAMPS       = "/rl/lidar_543_stamps"

ALL_TOPICS = [
    T_SCAN, T_SCAN_ALIGNED, T_SCAN_INVMASK, T_SCAN_INVSTATS,
    T_L181, T_L181_STAMP, T_L181_SRCMAP, T_L181_RMASK, T_L181_FSTATS,
    T_L543, T_L543_IDS, T_L543_IDSVIS, T_L543_STAMPS,
]


# ============================================================
# 工具函数
# ============================================================
def _stamp_key(t):
    """将 rospy.Time 或浮点秒统一成微秒精度字符串作为字典 key。"""
    if hasattr(t, "to_sec"):
        s = t.to_sec()
    else:
        s = float(t)
    return "%.6f" % s


def _values_equal(a, b, abs_tol=1e-6):
    a_nan = (a != a); b_nan = (b != b)
    if a_nan and b_nan: return True
    if a_nan or b_nan:  return False
    if math.isinf(a) and math.isinf(b):
        return (a > 0) == (b > 0)
    if math.isinf(a) or math.isinf(b):
        return False
    return abs(a - b) <= abs_tol


def _is_invalid_raw(r, rmin, rmax):
    if r is None: return True
    if math.isnan(r) or math.isinf(r): return True
    if r < 0: return True
    if r < rmin: return True
    if r > rmax: return True
    return False


def _safe_div(a, b, default=0.0):
    return (a / b) if b else default


# ============================================================
# Frame 数据类
# ============================================================
class Stage1Frame(object):
    __slots__ = ("stamp_key", "stamp_sec",
                 "scan", "aligned", "invmask", "invstats",
                 "anomalies", "validated")

    def __init__(self, stamp_key, stamp_sec):
        self.stamp_key = stamp_key
        self.stamp_sec = stamp_sec
        self.scan = None
        self.aligned = None
        self.invmask = None
        self.invstats = None
        self.anomalies = []
        self.validated = False


class Stage2Frame(object):
    __slots__ = ("stamp_key", "stamp_sec",
                 "l181", "stamp_msg", "srcmap", "rmask", "fstats",
                 "anomalies", "linked_s1", "validated")

    def __init__(self):
        self.stamp_key = None
        self.stamp_sec = None
        self.l181 = None
        self.stamp_msg = None
        self.srcmap = None
        self.rmask = None
        self.fstats = None
        self.anomalies = []
        self.linked_s1 = None
        self.validated = False


class Stage3Frame(object):
    __slots__ = ("recv_time", "local_idx",
                 "l543", "ids", "ids_vis", "stamps",
                 "linked_s2_t0", "linked_s2_t1", "linked_s2_t2",
                 "anomalies")

    def __init__(self):
        self.recv_time = None
        self.local_idx = None
        self.l543 = None
        self.ids = None
        self.ids_vis = None
        self.stamps = None
        self.linked_s2_t0 = None
        self.linked_s2_t1 = None
        self.linked_s2_t2 = None
        self.anomalies = []


# ============================================================
# 主节点
# ============================================================
class PipelineVerifierNode(object):
    def __init__(self):
        # ---------- 参数 ----------
        self.report_dir = os.path.expanduser(
            rospy.get_param("~report_dir", "~/lidar/data"))
        self.buffer_seconds = float(rospy.get_param("~buffer_seconds", 5.0))
        self.repair_ratio_warn = float(rospy.get_param("~repair_ratio_warn", 0.30))
        self.severe_ratio_warn = float(rospy.get_param("~severe_ratio_warn", 0.10))
        self.interp_tol = float(rospy.get_param("~interp_tol_normalized", 1e-4))
        self.rotation_sample_beams = int(rospy.get_param("~rotation_check_sample_beams", 20))
        self.silence_timeout = float(rospy.get_param("~silence_timeout_seconds", 3.0))
        self.max_frames_written = int(rospy.get_param("~max_frames_written", 20000))
        self.enable_stabilize_recheck = bool(rospy.get_param("~enable_stabilize_recheck", True))
        self.pass_rate = float(rospy.get_param("~anomaly_rate_pass", 0.005))
        self.acceptable_rate = float(rospy.get_param("~anomaly_rate_acceptable", 0.05))

        # ---------- 报告文件 ----------
        if not os.path.isdir(self.report_dir):
            try:
                os.makedirs(self.report_dir)
            except OSError:
                pass
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.report_path = os.path.join(
            self.report_dir, "pipeline_report_%s.txt" % ts)
        self.frames_tmp_path = self.report_path + ".frames_tmp"
        try:
            self.frames_file = open(self.frames_tmp_path, "w")
        except IOError as e:
            rospy.logerr("[verifier] cannot open frames tmp file %s: %s",
                         self.frames_tmp_path, e)
            raise

        # ---------- 状态 ----------
        self.lock = threading.RLock()
        self.start_wall = time.time()
        self.start_ros = None   # first-message time

        # 话题接收计数与最后接收时间
        self.topic_msg_count = defaultdict(int)
        self.topic_last_recv = {}
        self.topic_first_stamp = {}
        self.topic_last_stamp = {}

        # Stage 1/2 frame 字典(stamp_key -> frame)
        self.s1_frames = {}
        self.s2_frames = {}

        # Stage 1 FIFO 队列(invmask/invstats 无 header,用 FIFO 配对)
        self.s1_invmask_q = deque()
        self.s1_invstats_q = deque()

        # Stage 2 FIFO(除 stamp 外 4 路无 header)
        self.s2_l181_q = deque()
        self.s2_srcmap_q = deque()
        self.s2_rmask_q = deque()
        self.s2_fstats_q = deque()
        # stamp 独立,来时直接建 Stage2Frame 骨架
        self.s2_pending_stamps = deque()

        # Stage 3 FIFO(全 4 路同 callback 发布)
        self.s3_l543_q = deque()
        self.s3_ids_q = deque()
        self.s3_idsvis_q = deque()
        self.s3_stamps_q = deque()

        # Stage 3 锚点计数
        self.s3_local_counter = 0
        self.s3_anchored_count = 0
        self.s3_frames_written = 0

        # 异常累计
        self.anomaly_counts = defaultdict(int)

        # 上一帧 stage3 ids(用于连续性判定)
        self.last_s3_ids = None
        self.last_s3_stamps = None

        # v1.1 启动同步屏障:rospy 订阅的 TCP 握手完成时间不同,
        #   各路伴随 topic 的订阅可能比主"锚"(stamp / l543)先就绪,
        #   从而抢到了锚之前就已发布过的旧消息,堆在 fallback queue 里。
        #   如果不丢弃,第 1 帧的配对会错位,并永久保持错位。
        #   因此:
        #     - stage 1 收到第一条 /scan_aligned 时清 invmask/invstats 队列
        #     - stage 2 收到第一条 lidar_181_stamp 时清 4 条伴随队列
        #     - stage 3 收到第一条 lidar_543 时清 3 条伴随队列
        self._s1_sync_armed = False
        self._s2_sync_armed = False
        self._s3_sync_armed = False
        self._s1_sync_dropped = None   # (invmask, invstats)
        self._s2_sync_dropped = None   # (srcmap, rmask, l181, fstats)
        self._s3_sync_dropped = None   # (ids, ids_vis, stamps)

        # 三级通过计数
        self.s1_validated_pass = 0; self.s1_validated_fail = 0
        self.s2_validated_pass = 0; self.s2_validated_fail = 0
        self.s3_validated_pass = 0; self.s3_validated_fail = 0

        # 覆盖率:543 帧中能反向链接到 stage2/stage1/scan 的数量
        self.trace_full_s2 = 0   # 三个 stage 2 frame 全找到
        self.trace_miss_s2 = 0
        self.trace_full_s1 = 0
        self.trace_miss_s1 = 0
        self.trace_full_scan = 0
        self.trace_miss_scan = 0

        # ---------- 订阅 ----------
        queue_s = 50
        self.subs = []
        def mksub(topic, msg_type, cb):
            s = rospy.Subscriber(topic, msg_type, cb,
                                 queue_size=queue_s, tcp_nodelay=True)
            self.subs.append(s)
            self.topic_msg_count[topic] = 0
        mksub(T_SCAN,          LaserScan,          self.cb_scan)
        mksub(T_SCAN_ALIGNED,  LaserScan,          self.cb_scan_aligned)
        mksub(T_SCAN_INVMASK,  Float32MultiArray,  self.cb_scan_invmask)
        mksub(T_SCAN_INVSTATS, Float32MultiArray,  self.cb_scan_invstats)
        mksub(T_L181,          Float32MultiArray,  self.cb_l181)
        mksub(T_L181_STAMP,    Header,             self.cb_l181_stamp)
        mksub(T_L181_SRCMAP,   Float32MultiArray,  self.cb_l181_srcmap)
        mksub(T_L181_RMASK,    Float32MultiArray,  self.cb_l181_rmask)
        mksub(T_L181_FSTATS,   Float32MultiArray,  self.cb_l181_fstats)
        mksub(T_L543,          Float32MultiArray,  self.cb_l543)
        mksub(T_L543_IDS,      Float32MultiArray,  self.cb_l543_ids)
        mksub(T_L543_IDSVIS,   Float32MultiArray,  self.cb_l543_idsvis)
        mksub(T_L543_STAMPS,   Float64MultiArray,  self.cb_l543_stamps)

        # 静默检查定时器(每秒跑一次)
        self.silence_timer = rospy.Timer(
            rospy.Duration(1.0), self._on_silence_check)

        # shutdown hook
        rospy.on_shutdown(self._on_shutdown)

        rospy.logwarn("======================================")
        rospy.logwarn("[pipeline_verifier] started")
        rospy.logwarn("  report_dir         = %s", self.report_dir)
        rospy.logwarn("  report_path        = %s", self.report_path)
        rospy.logwarn("  buffer_seconds     = %.1f", self.buffer_seconds)
        rospy.logwarn("  repair_ratio_warn  = %.3f", self.repair_ratio_warn)
        rospy.logwarn("  interp_tol_norm    = %.1e", self.interp_tol)
        rospy.logwarn("  silence_timeout    = %.1fs", self.silence_timeout)
        rospy.logwarn("  subscribed topics  = %d", len(ALL_TOPICS))
        rospy.logwarn("======================================")

    # ==========================================================
    # 话题统计帮助
    # ==========================================================
    def _note_recv(self, topic, stamp_sec=None):
        self.topic_msg_count[topic] += 1
        now = rospy.get_time()
        self.topic_last_recv[topic] = now
        if self.start_ros is None:
            self.start_ros = now
        if stamp_sec is not None:
            if topic not in self.topic_first_stamp:
                self.topic_first_stamp[topic] = stamp_sec
            self.topic_last_stamp[topic] = stamp_sec

    # ==========================================================
    # Stage 1 回调
    # ==========================================================
    def cb_scan(self, msg):
        with self.lock:
            self._note_recv(T_SCAN, msg.header.stamp.to_sec())
            key = _stamp_key(msg.header.stamp)
            f = self.s1_frames.get(key)
            if f is None:
                f = Stage1Frame(key, msg.header.stamp.to_sec())
                self.s1_frames[key] = f
            f.scan = msg
            self._maybe_finalize_s1(f)
            self._trim_buffers()

    def cb_scan_aligned(self, msg):
        with self.lock:
            self._note_recv(T_SCAN_ALIGNED, msg.header.stamp.to_sec())
            # v1.1 sync barrier:首条 scan_aligned 到达时丢弃 fallback
            # 里沉积的 pre-aligned 伴随消息(订阅时序抢到的旧帧残留)
            if not self._s1_sync_armed:
                self._s1_sync_armed = True
                dropped = (len(self.s1_invmask_q), len(self.s1_invstats_q))
                self.s1_invmask_q.clear()
                self.s1_invstats_q.clear()
                self._s1_sync_dropped = dropped
                if any(d > 0 for d in dropped):
                    rospy.logwarn(
                        "[verifier] S1 sync barrier: dropped pre-aligned "
                        "leftovers (invmask=%d invstats=%d)",
                        dropped[0], dropped[1])
            key = _stamp_key(msg.header.stamp)
            f = self.s1_frames.get(key)
            if f is None:
                f = Stage1Frame(key, msg.header.stamp.to_sec())
                self.s1_frames[key] = f
            f.aligned = msg
            # 此时 scan_aligned 到了,尝试从 invmask/invstats FIFO 中配一对
            if f.invmask is None and self.s1_invmask_q:
                f.invmask = self.s1_invmask_q.popleft()
            if f.invstats is None and self.s1_invstats_q:
                f.invstats = self.s1_invstats_q.popleft()
            self._maybe_finalize_s1(f)
            self._trim_buffers()

    def cb_scan_invmask(self, msg):
        with self.lock:
            self._note_recv(T_SCAN_INVMASK)
            # 挂到最新还未配对 invmask 的 stage 1 frame 上,否则入队
            placed = False
            # 倒序找最新 invmask==None 的 frame(按 stamp_sec)
            candidate = None
            for f in self.s1_frames.values():
                if f.aligned is not None and f.invmask is None:
                    if candidate is None or f.stamp_sec > candidate.stamp_sec:
                        candidate = f
            if candidate is not None:
                candidate.invmask = msg
                placed = True
                self._maybe_finalize_s1(candidate)
            if not placed:
                self.s1_invmask_q.append(msg)
            self._trim_buffers()

    def cb_scan_invstats(self, msg):
        with self.lock:
            self._note_recv(T_SCAN_INVSTATS)
            placed = False
            candidate = None
            for f in self.s1_frames.values():
                if f.aligned is not None and f.invstats is None:
                    if candidate is None or f.stamp_sec > candidate.stamp_sec:
                        candidate = f
            if candidate is not None:
                candidate.invstats = msg
                placed = True
                self._maybe_finalize_s1(candidate)
            if not placed:
                self.s1_invstats_q.append(msg)
            self._trim_buffers()

    def _maybe_finalize_s1(self, f):
        if f.validated:
            return
        if (f.aligned is not None and f.invmask is not None
                and f.invstats is not None):
            self._validate_stage1(f)
            f.validated = True
            if not f.anomalies:
                self.s1_validated_pass += 1
            else:
                self.s1_validated_fail += 1
                for a in f.anomalies:
                    self.anomaly_counts[a[0]] += 1
                rospy.logwarn_throttle(
                    2.0, "[verifier][S1 anomaly] stamp=%.3f: %s",
                    f.stamp_sec, "; ".join([a[0] for a in f.anomalies[:3]]))

    # ==========================================================
    # Stage 2 回调
    # ==========================================================
    def cb_l181_stamp(self, msg):
        with self.lock:
            self._note_recv(T_L181_STAMP, msg.stamp.to_sec())
            # v1.1 sync barrier:首条 lidar_181_stamp 到达时丢弃 fallback
            # 里沉积的 pre-stamp 伴随消息。这是本 verifier 最关键的一处修正:
            # 如果不丢,srcmap/rmask/fstats 会永久性错位一两帧,导致
            # S2/fstats_*_inconsistent、S2/mask0_but_raw_inf、
            # S3/t*_block_mismatch 等海量虚假异常。
            if not self._s2_sync_armed:
                self._s2_sync_armed = True
                dropped = (len(self.s2_srcmap_q), len(self.s2_rmask_q),
                           len(self.s2_l181_q),   len(self.s2_fstats_q))
                self.s2_srcmap_q.clear()
                self.s2_rmask_q.clear()
                self.s2_l181_q.clear()
                self.s2_fstats_q.clear()
                self._s2_sync_dropped = dropped
                if any(d > 0 for d in dropped):
                    rospy.logwarn(
                        "[verifier] S2 sync barrier: dropped pre-stamp leftovers "
                        "(srcmap=%d rmask=%d l181=%d fstats=%d) to avoid FIFO skew",
                        dropped[0], dropped[1], dropped[2], dropped[3])
            key = _stamp_key(msg.stamp)
            f = Stage2Frame()
            f.stamp_key = key
            f.stamp_sec = msg.stamp.to_sec()
            f.stamp_msg = msg
            # 尝试从现有 FIFO 队列拉配对
            if self.s2_srcmap_q: f.srcmap = self.s2_srcmap_q.popleft()
            if self.s2_rmask_q:  f.rmask  = self.s2_rmask_q.popleft()
            if self.s2_l181_q:   f.l181   = self.s2_l181_q.popleft()
            if self.s2_fstats_q: f.fstats = self.s2_fstats_q.popleft()
            self.s2_frames[key] = f
            self.s2_pending_stamps.append(key)
            self._maybe_finalize_s2(f)
            self._trim_buffers()

    def _attach_to_latest_pending(self, attr, msg, fallback_q):
        """把一条 stage 2 伴随消息挂到最早还缺此项的 pending frame 上。"""
        # 按 pending 队列的顺序找第一个缺该 attr 的 frame
        for k in self.s2_pending_stamps:
            f = self.s2_frames.get(k)
            if f is None: continue
            if getattr(f, attr) is None:
                setattr(f, attr, msg)
                self._maybe_finalize_s2(f)
                return True
        fallback_q.append(msg)
        return False

    def cb_l181_srcmap(self, msg):
        with self.lock:
            self._note_recv(T_L181_SRCMAP)
            self._attach_to_latest_pending("srcmap", msg, self.s2_srcmap_q)
            self._trim_buffers()

    def cb_l181_rmask(self, msg):
        with self.lock:
            self._note_recv(T_L181_RMASK)
            self._attach_to_latest_pending("rmask", msg, self.s2_rmask_q)
            self._trim_buffers()

    def cb_l181(self, msg):
        with self.lock:
            self._note_recv(T_L181)
            self._attach_to_latest_pending("l181", msg, self.s2_l181_q)
            self._trim_buffers()

    def cb_l181_fstats(self, msg):
        with self.lock:
            self._note_recv(T_L181_FSTATS)
            self._attach_to_latest_pending("fstats", msg, self.s2_fstats_q)
            self._trim_buffers()

    def _maybe_finalize_s2(self, f):
        if f.validated:
            return
        if (f.l181 is not None and f.stamp_msg is not None
                and f.srcmap is not None and f.rmask is not None
                and f.fstats is not None):
            # 链接到对应 stage 1 frame(若已有)
            f.linked_s1 = self.s1_frames.get(f.stamp_key)
            self._validate_stage2(f)
            f.validated = True
            if not f.anomalies:
                self.s2_validated_pass += 1
            else:
                self.s2_validated_fail += 1
                for a in f.anomalies:
                    self.anomaly_counts[a[0]] += 1
                rospy.logwarn_throttle(
                    2.0, "[verifier][S2 anomaly] stamp=%.3f: %s",
                    f.stamp_sec, "; ".join([a[0] for a in f.anomalies[:3]]))

    # ==========================================================
    # Stage 3 回调
    # ==========================================================
    def cb_l543(self, msg):
        with self.lock:
            self._note_recv(T_L543)
            # v1.1 sync barrier:首条 lidar_543 到达时丢弃 ids/vis/stamps
            # 里沉积的 pre-l543 伴随消息(防御性;实际 stage 3 锚是 l543
            # 且 l543 在 stage 3 publish 中排第 1,通常不会累积伴随消息)
            if not self._s3_sync_armed:
                self._s3_sync_armed = True
                dropped = (len(self.s3_ids_q), len(self.s3_idsvis_q),
                           len(self.s3_stamps_q))
                self.s3_ids_q.clear()
                self.s3_idsvis_q.clear()
                self.s3_stamps_q.clear()
                self._s3_sync_dropped = dropped
                if any(d > 0 for d in dropped):
                    rospy.logwarn(
                        "[verifier] S3 sync barrier: dropped pre-l543 leftovers "
                        "(ids=%d ids_vis=%d stamps=%d)",
                        dropped[0], dropped[1], dropped[2])
            self.s3_l543_q.append(msg)
            self._try_anchor_s3()
            self._trim_buffers()

    def cb_l543_ids(self, msg):
        with self.lock:
            self._note_recv(T_L543_IDS)
            self.s3_ids_q.append(msg)
            self._try_anchor_s3()

    def cb_l543_idsvis(self, msg):
        with self.lock:
            self._note_recv(T_L543_IDSVIS)
            self.s3_idsvis_q.append(msg)
            self._try_anchor_s3()

    def cb_l543_stamps(self, msg):
        with self.lock:
            self._note_recv(T_L543_STAMPS)
            self.s3_stamps_q.append(msg)
            self._try_anchor_s3()

    def _try_anchor_s3(self):
        """当 4 路 stage 3 消息都至少有一条,各 pop 一条组帧并锚定。"""
        while (self.s3_l543_q and self.s3_ids_q
               and self.s3_idsvis_q and self.s3_stamps_q):
            f = Stage3Frame()
            f.recv_time = rospy.get_time()
            self.s3_local_counter += 1
            f.local_idx = self.s3_local_counter
            f.l543 = self.s3_l543_q.popleft()
            f.ids = self.s3_ids_q.popleft()
            f.ids_vis = self.s3_idsvis_q.popleft()
            f.stamps = self.s3_stamps_q.popleft()
            # 链接到三个 stage 2 frame
            if len(f.stamps.data) >= 3:
                k0 = _stamp_key(f.stamps.data[0])
                k1 = _stamp_key(f.stamps.data[1])
                k2 = _stamp_key(f.stamps.data[2])
                f.linked_s2_t0 = self.s2_frames.get(k0)
                f.linked_s2_t1 = self.s2_frames.get(k1)
                f.linked_s2_t2 = self.s2_frames.get(k2)
            # 校验 + 追溯 + 写入
            self._validate_stage3(f)
            self.s3_anchored_count += 1
            if not f.anomalies:
                self.s3_validated_pass += 1
            else:
                self.s3_validated_fail += 1
                for a in f.anomalies:
                    self.anomaly_counts[a[0]] += 1
            # 实时告警
            if f.anomalies:
                rospy.logwarn_throttle(
                    1.0, "[verifier][S3 anomaly] #%d stamp=%.3f: %s",
                    f.local_idx,
                    f.stamps.data[0] if len(f.stamps.data) >= 1 else -1.0,
                    "; ".join([a[0] for a in f.anomalies[:3]]))
            # 覆盖率统计
            n_s2 = sum(1 for x in [f.linked_s2_t0, f.linked_s2_t1, f.linked_s2_t2]
                       if x is not None)
            if n_s2 == 3: self.trace_full_s2 += 1
            else:         self.trace_miss_s2 += 1
            n_s1 = 0; n_scan = 0
            for s2 in [f.linked_s2_t0, f.linked_s2_t1, f.linked_s2_t2]:
                if s2 is None: continue
                s1 = s2.linked_s1
                if s1 is not None:
                    n_s1 += 1
                    if s1.scan is not None: n_scan += 1
            if n_s1 == 3: self.trace_full_s1 += 1
            else:         self.trace_miss_s1 += 1
            if n_scan == 3: self.trace_full_scan += 1
            else:           self.trace_miss_scan += 1
            # 写入 frames 文件(带上限保护)
            if self.s3_frames_written < self.max_frames_written:
                try:
                    self._write_frame_entry(f)
                    self.s3_frames_written += 1
                except Exception as e:
                    rospy.logwarn_throttle(5.0,
                        "[verifier] write frame entry failed: %s", e)

    # ==========================================================
    # Stage 1 校验
    # ==========================================================
    def _validate_stage1(self, f):
        a = f.aligned
        N = len(a.ranges)
        # --- 长度 ---
        if f.invmask is not None and len(f.invmask.data) != N:
            f.anomalies.append(("S1/invmask_len_mismatch",
                "invmask.len=%d vs aligned.ranges_n=%d" % (len(f.invmask.data), N)))
        if f.invstats is not None and len(f.invstats.data) != 4:
            f.anomalies.append(("S1/invstats_len_mismatch",
                "invstats.len=%d vs expected 4" % len(f.invstats.data)))
        # --- 元数据(vs /scan) ---
        if f.scan is not None:
            s = f.scan
            for key in ["angle_min","angle_max","angle_increment",
                        "range_min","range_max"]:
                va = getattr(a, key)
                vs = getattr(s, key)
                if not _values_equal(float(va), float(vs), abs_tol=1e-9):
                    f.anomalies.append(("S1/meta_mismatch",
                        "%s: scan=%s aligned=%s" % (key, vs, va)))
                    break
            if len(s.ranges) != N:
                f.anomalies.append(("S1/len_mismatch_raw_aligned",
                    "raw_n=%d aligned_n=%d" % (len(s.ranges), N)))
        # --- rotation 抽查 ---
        if f.scan is not None and len(f.scan.ranges) == N and N >= 4:
            half = N // 2
            step = max(1, N // max(1, self.rotation_sample_beams))
            bad = 0
            first_bad = None
            for k in range(0, N, step):
                src_k = (k + half) % N
                rv = f.scan.ranges[src_k]
                av = a.ranges[k]
                if not _values_equal(float(rv), float(av), abs_tol=1e-6):
                    bad += 1
                    if first_bad is None:
                        first_bad = (k, src_k, rv, av)
            if bad > 0:
                f.anomalies.append(("S1/rotation_mismatch",
                    "bad_beams=%d sample_of_%d  first=(k=%d src_k=%d raw=%.3f aligned=%.3f)"
                    % (bad, len(range(0,N,step)),
                       first_bad[0], first_bad[1], first_bad[2], first_bad[3])))
        # --- invalid_mask vs 实际 ranges 的自洽 ---
        if f.invmask is not None and len(f.invmask.data) == N:
            rmin = a.range_min; rmax = a.range_max
            mask_mismatch = 0
            first_mm = None
            for k in range(N):
                actual_invalid = _is_invalid_raw(a.ranges[k], rmin, rmax)
                declared_invalid = (int(round(f.invmask.data[k])) == 1)
                if actual_invalid != declared_invalid:
                    mask_mismatch += 1
                    if first_mm is None:
                        first_mm = (k, a.ranges[k], declared_invalid, actual_invalid)
            if mask_mismatch > 0:
                f.anomalies.append(("S1/invmask_actual_mismatch",
                    "mismatched_beams=%d  first=(k=%d r=%.3f decl=%s actual=%s)"
                    % (mask_mismatch, first_mm[0], first_mm[1],
                       first_mm[2], first_mm[3])))
        # --- invalid_stats 自洽 ---
        if (f.invmask is not None and f.invstats is not None
                and len(f.invmask.data) == N and len(f.invstats.data) == 4):
            m_count = sum(1 for v in f.invmask.data if int(round(v)) == 1)
            declared_count = int(round(f.invstats.data[0]))
            declared_ratio = float(f.invstats.data[1])
            if declared_count != m_count:
                f.anomalies.append(("S1/invstats_count_inconsistent",
                    "declared=%d mask_derived=%d" % (declared_count, m_count)))
            expected_ratio = _safe_div(m_count, N)
            if not _values_equal(declared_ratio, expected_ratio, abs_tol=1e-3):
                f.anomalies.append(("S1/invstats_ratio_inconsistent",
                    "declared=%.4f expected=%.4f" % (declared_ratio, expected_ratio)))

    # ==========================================================
    # Stage 2 校验
    # ==========================================================
    def _validate_stage2(self, f):
        # --- 长度 ---
        if len(f.l181.data) != NUM_BEAMS:
            f.anomalies.append(("S2/l181_len_mismatch",
                "got=%d expected=%d" % (len(f.l181.data), NUM_BEAMS)))
            return
        if len(f.srcmap.data) != 3 * NUM_BEAMS:
            f.anomalies.append(("S2/srcmap_len_mismatch",
                "got=%d expected=%d" % (len(f.srcmap.data), 3 * NUM_BEAMS)))
        if len(f.rmask.data) != NUM_BEAMS:
            f.anomalies.append(("S2/rmask_len_mismatch",
                "got=%d expected=%d" % (len(f.rmask.data), NUM_BEAMS)))
        if len(f.fstats.data) < 4:
            f.anomalies.append(("S2/fstats_len_mismatch",
                "got=%d expected>=4" % len(f.fstats.data)))

        # --- mask 是否纯 0/1 ---
        mask_int = []
        not_binary = 0
        for v in f.rmask.data:
            iv = int(round(v))
            mask_int.append(iv)
            if iv not in (0, 1) or abs(v - iv) > 1e-3:
                not_binary += 1
        if not_binary > 0:
            f.anomalies.append(("S2/rmask_not_binary",
                "non_binary_beams=%d" % not_binary))

        # --- 归一化范围 ---
        out_of_range = 0
        nan_inf = 0
        for v in f.l181.data:
            if v != v or math.isinf(v):
                nan_inf += 1
            elif not (-1e-6 <= v <= 1.0 + 1e-6):
                out_of_range += 1
        if nan_inf > 0:
            f.anomalies.append(("S2/l181_nan_inf",
                "count=%d" % nan_inf))
        if out_of_range > 0:
            f.anomalies.append(("S2/l181_out_of_01",
                "count=%d" % out_of_range))

        # --- fstats 自洽(repair_ratio 应 ≈ sum(mask)/181) ---
        declared_ratio = float(f.fstats.data[0]) if len(f.fstats.data) >= 1 else 0.0
        declared_count = float(f.fstats.data[1]) if len(f.fstats.data) >= 2 else 0.0
        mask_count = sum(mask_int)
        if not _values_equal(declared_count, float(mask_count), abs_tol=0.5):
            f.anomalies.append(("S2/fstats_count_inconsistent",
                "declared=%.1f mask_sum=%d" % (declared_count, mask_count)))
        expected_ratio = _safe_div(mask_count, NUM_BEAMS)
        if not _values_equal(declared_ratio, expected_ratio, abs_tol=1e-3):
            f.anomalies.append(("S2/fstats_ratio_inconsistent",
                "declared=%.4f expected=%.4f" % (declared_ratio, expected_ratio)))

        # --- 阈值告警(情况,不阻断) ---
        if declared_ratio > self.repair_ratio_warn:
            f.anomalies.append(("S2/repair_ratio_high",
                "ratio=%.3f > %.3f" % (declared_ratio, self.repair_ratio_warn)))
        if len(f.fstats.data) >= 4:
            severe_ratio = float(f.fstats.data[3])
            if severe_ratio > self.severe_ratio_warn:
                f.anomalies.append(("S2/severe_ratio_high",
                    "ratio=%.3f > %.3f" % (severe_ratio, self.severe_ratio_warn)))

        # --- 跨级一致性(vs linked_s1) ---
        if f.linked_s1 is not None and f.linked_s1.aligned is not None:
            self._cross_check_s1_s2(f)

    def _cross_check_s1_s2(self, s2):
        """验证 lidar_181[k] = (1-w)*aligned[floor] + w*aligned[ceil]) / MAX,
        对 repair_mask[k]==0 的 beam。"""
        s1 = s2.linked_s1
        a = s1.aligned
        N = len(a.ranges)
        interp_mismatch = 0
        mask0_raw_inf = 0
        first_mm = None
        first_mi = None
        for k in range(NUM_BEAMS):
            repair = int(round(s2.rmask.data[k]))
            if repair != 0:
                continue
            try:
                floor_i = int(round(s2.srcmap.data[k]))
                ceil_i = int(round(s2.srcmap.data[k + NUM_BEAMS]))
                weight = float(s2.srcmap.data[k + 2 * NUM_BEAMS])
            except (IndexError, ValueError):
                continue
            if floor_i < 0 or floor_i >= N or ceil_i < 0 or ceil_i >= N:
                continue
            fv = a.ranges[floor_i]; cv = a.ranges[ceil_i]
            if (math.isnan(fv) or math.isinf(fv)
                    or math.isnan(cv) or math.isinf(cv)):
                mask0_raw_inf += 1
                if first_mi is None:
                    first_mi = (k, floor_i, ceil_i, fv, cv)
                continue
            expected = ((1.0 - weight) * fv + weight * cv) / LIDAR_MAX_DISTANCE
            observed = float(s2.l181.data[k])
            if not _values_equal(observed, expected, abs_tol=self.interp_tol):
                interp_mismatch += 1
                if first_mm is None:
                    first_mm = (k, observed, expected, fv, cv, weight)
        if mask0_raw_inf > 0:
            s2.anomalies.append(("S2/mask0_but_raw_inf",
                "count=%d  first=(k=%d floor=%d ceil=%d fv=%.3f cv=%.3f)"
                % (mask0_raw_inf, first_mi[0], first_mi[1], first_mi[2],
                   first_mi[3], first_mi[4])))
        if interp_mismatch > 0:
            s2.anomalies.append(("S2/interp_mismatch",
                "count=%d tol=%.1e first=(k=%d obs=%.6f exp=%.6f fv=%.3f cv=%.3f w=%.3f)"
                % (interp_mismatch, self.interp_tol,
                   first_mm[0], first_mm[1], first_mm[2],
                   first_mm[3], first_mm[4], first_mm[5])))

    # ==========================================================
    # Stage 3 校验
    # ==========================================================
    def _validate_stage3(self, f):
        # --- 长度 ---
        if len(f.l543.data) != STACKED_DIM:
            f.anomalies.append(("S3/l543_len_mismatch",
                "got=%d expected=%d" % (len(f.l543.data), STACKED_DIM)))
        if len(f.ids.data) != 3:
            f.anomalies.append(("S3/ids_len_mismatch",
                "got=%d expected=3" % len(f.ids.data)))
        if len(f.ids_vis.data) != STACKED_DIM:
            f.anomalies.append(("S3/ids_vis_len_mismatch",
                "got=%d expected=%d" % (len(f.ids_vis.data), STACKED_DIM)))
        if len(f.stamps.data) != 3:
            f.anomalies.append(("S3/stamps_len_mismatch",
                "got=%d expected=3" % len(f.stamps.data)))
        if f.anomalies:
            return   # 长度都不对,后续检查没意义

        # --- stamps 单调 ---
        s0, s1, s2 = f.stamps.data[0], f.stamps.data[1], f.stamps.data[2]
        if not (s0 == s1 == s2):  # 非首帧瞬态
            if not (s0 >= s1 >= s2):
                f.anomalies.append(("S3/stamps_non_monotonic",
                    "[%.6f, %.6f, %.6f]  dt01=%+.4f dt12=%+.4f"
                    % (s0, s1, s2, s0 - s1, s1 - s2)))

        # --- ids 连续性(除首帧瞬态) ---
        id0 = int(round(f.ids.data[0]))
        id1 = int(round(f.ids.data[1]))
        id2 = int(round(f.ids.data[2]))
        if not (id0 == id1 == id2):   # 首帧所有 id 相同
            if not (id0 == id1 + 1 and id1 >= id2
                    and (id1 == id2 or id1 == id2 + 1)):
                f.anomalies.append(("S3/ids_not_sequential",
                    "ids=[%d, %d, %d]" % (id0, id1, id2)))
        # --- ids_vis 与 ids 一致 ---
        vis_ok = True
        for seg, idv in enumerate([id0, id1, id2]):
            for k in range(NUM_BEAMS):
                idx = seg * NUM_BEAMS + k
                obs = int(round(f.ids_vis.data[idx]))
                if obs != idv:
                    vis_ok = False
                    break
            if not vis_ok:
                break
        if not vis_ok:
            f.anomalies.append(("S3/ids_vis_inconsistent", "see frame detail"))

        # --- 反向链接完整性 ---
        if f.linked_s2_t0 is None:
            f.anomalies.append(("S3/missing_s2_t0", "stamp=%.6f" % s0))
        if f.linked_s2_t1 is None:
            f.anomalies.append(("S3/missing_s2_t1", "stamp=%.6f" % s1))
        if f.linked_s2_t2 is None:
            f.anomalies.append(("S3/missing_s2_t2", "stamp=%.6f" % s2))

        # --- [t0|t1|t2] 堆叠一致性(对 t1/t2 块) ---
        if f.linked_s2_t1 is not None and len(f.linked_s2_t1.l181.data) == NUM_BEAMS:
            bad1 = self._count_block_mismatch(
                f.l543.data, NUM_BEAMS, 2 * NUM_BEAMS, f.linked_s2_t1.l181.data)
            if bad1 > 0:
                f.anomalies.append(("S3/t1_block_mismatch",
                    "beams=%d/%d" % (bad1, NUM_BEAMS)))
        if f.linked_s2_t2 is not None and len(f.linked_s2_t2.l181.data) == NUM_BEAMS:
            bad2 = self._count_block_mismatch(
                f.l543.data, 2 * NUM_BEAMS, 3 * NUM_BEAMS, f.linked_s2_t2.l181.data)
            if bad2 > 0:
                f.anomalies.append(("S3/t2_block_mismatch",
                    "beams=%d/%d" % (bad2, NUM_BEAMS)))

        # --- t0 块稳态语义(repair=0 beam 直通;repair=1 beam 做 median) ---
        if (f.linked_s2_t0 is not None
                and f.linked_s2_t1 is not None
                and f.linked_s2_t2 is not None
                and len(f.linked_s2_t0.l181.data) == NUM_BEAMS
                and len(f.linked_s2_t0.rmask.data) == NUM_BEAMS):
            t0_valid_mm = 0
            t0_repair_mm = 0
            t0 = f.linked_s2_t0
            t1 = f.linked_s2_t1
            t2 = f.linked_s2_t2
            for k in range(NUM_BEAMS):
                stacked_v = f.l543.data[k]
                cur_v = t0.l181.data[k]
                repair = int(round(t0.rmask.data[k]))
                if repair == 0:
                    if not _values_equal(stacked_v, cur_v, abs_tol=1e-6):
                        t0_valid_mm += 1
                else:
                    if not self.enable_stabilize_recheck:
                        # 不启用稳态反推,就只检查范围
                        if not (-1e-6 <= stacked_v <= 1.0 + 1e-6):
                            t0_repair_mm += 1
                        continue
                    # 预期 median(cur, t1[k], t2[k]) 或 cur(首帧)
                    t1v = t1.l181.data[k]
                    t2v = t2.l181.data[k]
                    expected_med = sorted([cur_v, t1v, t2v])[1]
                    # 首帧情况:t1 == t2 == cur,中位数就是 cur
                    # 此外 stage 3 也可能关闭 stabilize,此时 stacked_v==cur
                    if not (_values_equal(stacked_v, expected_med, abs_tol=1e-6)
                            or _values_equal(stacked_v, cur_v, abs_tol=1e-6)):
                        t0_repair_mm += 1
            if t0_valid_mm > 0:
                f.anomalies.append(("S3/t0_valid_beam_mismatch",
                    "beams=%d/%d" % (t0_valid_mm, NUM_BEAMS)))
            if t0_repair_mm > 0:
                f.anomalies.append(("S3/t0_repair_beam_not_stabilized",
                    "beams=%d/%d (neither ==cur nor ==median)"
                    % (t0_repair_mm, NUM_BEAMS)))

        # --- 相对上一帧的全局单调(跨 543 帧) ---
        if self.last_s3_stamps is not None:
            last_t0 = self.last_s3_stamps[0]
            if s0 < last_t0 - 1e-9:
                f.anomalies.append(("S3/global_stamp_backward",
                    "cur_t0=%.6f prev_t0=%.6f  dt=%+.4f"
                    % (s0, last_t0, s0 - last_t0)))
        if self.last_s3_ids is not None:
            prev_id0 = self.last_s3_ids[0]
            if id0 <= prev_id0:
                f.anomalies.append(("S3/global_id_not_advancing",
                    "cur_id0=%d prev_id0=%d" % (id0, prev_id0)))
            elif id0 - prev_id0 > 1:
                f.anomalies.append(("S3/global_id_jump",
                    "cur_id0=%d prev_id0=%d gap=%d"
                    % (id0, prev_id0, id0 - prev_id0 - 1)))

        self.last_s3_ids = (id0, id1, id2)
        self.last_s3_stamps = (s0, s1, s2)

    @staticmethod
    def _count_block_mismatch(stacked, start, end, expected):
        bad = 0
        n = end - start
        for k in range(n):
            if k >= len(expected):
                bad += 1; continue
            if not _values_equal(float(stacked[start + k]),
                                 float(expected[k]), abs_tol=1e-6):
                bad += 1
        return bad

    # ==========================================================
    # 静默告警 + 缓冲清理
    # ==========================================================
    def _on_silence_check(self, _event):
        with self.lock:
            if self.start_ros is None:
                return   # 还没收到任何消息
            now = rospy.get_time()
            for topic in ALL_TOPICS:
                last = self.topic_last_recv.get(topic)
                if last is None:
                    if now - self.start_ros > self.silence_timeout:
                        rospy.logwarn_throttle(
                            5.0, "[verifier][silence] %s 尚未收到任何消息", topic)
                    continue
                dt = now - last
                if dt > self.silence_timeout:
                    rospy.logwarn_throttle(
                        5.0, "[verifier][silence] %s 已 %.1fs 无新消息",
                        topic, dt)

    def _trim_buffers(self):
        if not self.s1_frames and not self.s2_frames:
            return
        if self.start_ros is None:
            return
        # 以最新 stage2 stamp 的时间为参考,截掉 buffer_seconds 之前的 frames
        ref = None
        for f in self.s2_frames.values():
            if ref is None or f.stamp_sec > ref: ref = f.stamp_sec
        if ref is None:
            for f in self.s1_frames.values():
                if ref is None or f.stamp_sec > ref: ref = f.stamp_sec
        if ref is None:
            return
        cutoff = ref - self.buffer_seconds
        dead_s1 = [k for k, f in self.s1_frames.items() if f.stamp_sec < cutoff]
        for k in dead_s1: del self.s1_frames[k]
        dead_s2 = [k for k, f in self.s2_frames.items() if f.stamp_sec < cutoff]
        for k in dead_s2:
            del self.s2_frames[k]
            try: self.s2_pending_stamps.remove(k)
            except ValueError: pass
        # FIFO 队列也不能无限涨
        MAX_Q = 100
        for q in [self.s1_invmask_q, self.s1_invstats_q,
                  self.s2_l181_q, self.s2_srcmap_q,
                  self.s2_rmask_q, self.s2_fstats_q,
                  self.s3_l543_q, self.s3_ids_q,
                  self.s3_idsvis_q, self.s3_stamps_q]:
            while len(q) > MAX_Q:
                q.popleft()

    # ==========================================================
    # 逐帧写入
    # ==========================================================
    def _write_frame_entry(self, f):
        w = self.frames_file.write
        stamps = f.stamps.data if f.stamps is not None else [0.0, 0.0, 0.0]
        ids = f.ids.data if f.ids is not None else [0.0, 0.0, 0.0]
        w("\n" + "=" * 72 + "\n")
        w("[Frame #%d]  /rl/lidar_543 recv at %.3f  (wall: %s)\n"
          % (f.local_idx, f.recv_time,
             time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(f.recv_time))))
        w("\n")
        w("  Stage 3:\n")
        w("    ids          = [%d, %d, %d]\n"
          % (int(ids[0]), int(ids[1]), int(ids[2])))
        w("    stamps       = [%.6f, %.6f, %.6f]\n"
          % (stamps[0], stamps[1], stamps[2]))
        if len(f.l543.data) == STACKED_DIM:
            s = f.l543.data
            w("    l543 stats   = min=%.4f max=%.4f mean=%.4f\n"
              % (min(s), max(s), sum(s) / len(s)))
        for label, s2 in [("t0", f.linked_s2_t0),
                          ("t1", f.linked_s2_t1),
                          ("t2", f.linked_s2_t2)]:
            w("\n  %s frame (stamp=%.6f):\n"
              % (label, stamps[0] if label == "t0"
                        else stamps[1] if label == "t1"
                        else stamps[2]))
            if s2 is None:
                w("    [MISSING] 无对应 stage 2 frame(可能 stage 2 丢帧或 buffer 过期)\n")
                continue
            # Stage 2 指标
            if s2.fstats is not None and len(s2.fstats.data) >= 4:
                fs = s2.fstats.data
                w("    Stage 2:\n")
                w("      repair_ratio = %.4f  count=%d  front=%d  severe_ratio=%.4f\n"
                  % (fs[0], int(fs[1]), int(fs[2]), fs[3]))
            if s2.l181 is not None and len(s2.l181.data) == NUM_BEAMS:
                vals = s2.l181.data
                w("      l181 stats   = min=%.4f max=%.4f mean=%.4f\n"
                  % (min(vals), max(vals), sum(vals) / NUM_BEAMS))
            if s2.anomalies:
                w("      anomalies    = %d\n" % len(s2.anomalies))
                for a in s2.anomalies[:5]:
                    w("        - [%s] %s\n" % (a[0], a[1]))
            else:
                w("      anomalies    = 0\n")
            # Stage 1
            s1 = s2.linked_s1
            if s1 is None:
                w("    Stage 1 (scan_aligned): [MISSING]\n")
                continue
            if s1.aligned is not None:
                N = len(s1.aligned.ranges)
                w("    Stage 1 (scan_aligned):\n")
                w("      ranges_n     = %d  range=[%.2f, %.2f]\n"
                  % (N, s1.aligned.range_min, s1.aligned.range_max))
                if s1.invstats is not None and len(s1.invstats.data) >= 4:
                    iv = s1.invstats.data
                    w("      invalid      = count=%d  ratio=%.4f  front=%d  front_ratio=%.4f\n"
                      % (int(iv[0]), iv[1], int(iv[2]), iv[3]))
                w("      raw_linked   = %s\n"
                  % ("YES" if s1.scan is not None else "NO (only /scan_aligned)"))
                if s1.anomalies:
                    w("      anomalies    = %d\n" % len(s1.anomalies))
                    for a in s1.anomalies[:5]:
                        w("        - [%s] %s\n" % (a[0], a[1]))
                else:
                    w("      anomalies    = 0\n")
        # Stage 3 自身异常
        w("\n")
        if f.anomalies:
            w("  === frame verdict: FAIL (%d anomalies) ===\n" % len(f.anomalies))
            for a in f.anomalies:
                w("    - [%s] %s\n" % (a[0], a[1]))
        else:
            w("  === frame verdict: PASS ===\n")
        self.frames_file.flush()

    # ==========================================================
    # 报告生成(shutdown hook)
    # ==========================================================
    def _on_shutdown(self):
        try:
            self.silence_timer.shutdown()
        except Exception:
            pass
        try:
            self.frames_file.flush()
            self.frames_file.close()
        except Exception:
            pass
        # 写最终报告
        try:
            self._write_final_report()
        except Exception as e:
            rospy.logerr("[verifier] write_final_report failed: %s", e)
        rospy.logwarn("[pipeline_verifier] report written -> %s", self.report_path)

    def _write_final_report(self):
        duration = time.time() - self.start_wall
        with open(self.report_path, "w") as w:
            # ------- Header -------
            w.write("=" * 80 + "\n")
            w.write(" " * 16 + "LiDAR Pipeline Online Verification Report\n")
            w.write(" " * 16 + "Generated: %s\n"
                    % time.strftime("%Y-%m-%d %H:%M:%S"))
            w.write(" " * 16 + "Duration:  %.1f s\n" % duration)
            w.write("=" * 80 + "\n\n")

            # ------- 1. 话题接收 -------
            w.write("1. Topic Reception Summary\n")
            w.write("-" * 80 + "\n")
            for topic in ALL_TOPICS:
                c = self.topic_msg_count.get(topic, 0)
                hz = _safe_div(c, duration)
                last = self.topic_last_recv.get(topic)
                age = (rospy.get_time() - last) if last else -1.0
                fs = self.topic_first_stamp.get(topic, float("nan"))
                ls = self.topic_last_stamp.get(topic, float("nan"))
                w.write("  %-34s  msgs=%-6d  avg=%5.2fHz  last_age=%4.1fs"
                        % (topic, c, hz, age))
                if not math.isnan(fs):
                    w.write("  stamp_range=[%.3f, %.3f]" % (fs, ls))
                w.write("\n")
            w.write("\n")

            # ------- 2. 帧数对齐 -------
            w.write("2. Frame Count Parity\n")
            w.write("-" * 80 + "\n")
            n_scan = self.topic_msg_count.get(T_SCAN, 0)
            n_align = self.topic_msg_count.get(T_SCAN_ALIGNED, 0)
            n_l181 = self.topic_msg_count.get(T_L181, 0)
            n_l543 = self.topic_msg_count.get(T_L543, 0)
            def parity(a, b, al, bl):
                d = a - b
                st = "OK" if d == 0 else "DROP=%d" % d
                w.write("  %-40s %d vs %d  diff=%+d  [%s]\n"
                        % ("%s -> %s" % (al, bl), a, b, d, st))
            parity(n_scan,  n_align, T_SCAN,         T_SCAN_ALIGNED)
            parity(n_align, n_l181,  T_SCAN_ALIGNED, T_L181)
            parity(n_l181,  n_l543,  T_L181,         T_L543)
            w.write("\n")

            # ------- 3. 跨级追溯覆盖率 -------
            w.write("3. Cross-stage Traceability (anchored on /rl/lidar_543)\n")
            w.write("-" * 80 + "\n")
            total543 = self.s3_anchored_count
            w.write("  lidar_543 anchored frames                = %d\n" % total543)
            w.write("  with full 3x stage2 link                 = %d / %d  (%.1f%%)\n"
                    % (self.trace_full_s2, total543,
                       100.0 * _safe_div(self.trace_full_s2, total543)))
            w.write("  with full 3x stage1 link                 = %d / %d  (%.1f%%)\n"
                    % (self.trace_full_s1, total543,
                       100.0 * _safe_div(self.trace_full_s1, total543)))
            w.write("  with full 3x /scan link (best effort)    = %d / %d  (%.1f%%)\n"
                    % (self.trace_full_scan, total543,
                       100.0 * _safe_div(self.trace_full_scan, total543)))
            w.write("\n")

            # ------- 4. 验证通过率 -------
            w.write("4. Validation Pass Rates\n")
            w.write("-" * 80 + "\n")
            def pr(name, passed, failed):
                total = passed + failed
                rate = 100.0 * _safe_div(passed, total)
                w.write("  %-40s  %d / %d  (%.2f%%)\n"
                        % (name, passed, total, rate))
            pr("Stage 1 (rotation + invalid + meta)",
               self.s1_validated_pass, self.s1_validated_fail)
            pr("Stage 2 (resample + mask + interp cross)",
               self.s2_validated_pass, self.s2_validated_fail)
            pr("Stage 3 (stack + stamp mono + ids)",
               self.s3_validated_pass, self.s3_validated_fail)
            w.write("\n")

            # ------- 5. 异常明细 -------
            w.write("5. Anomaly Counts (by category)\n")
            w.write("-" * 80 + "\n")
            if not self.anomaly_counts:
                w.write("  (no anomalies detected)\n")
            else:
                sorted_keys = sorted(
                    self.anomaly_counts.items(), key=lambda x: -x[1])
                for name, cnt in sorted_keys:
                    w.write("  %-40s  %d\n" % (name, cnt))
            w.write("\n")

            # ------- 6. 总体结论 -------
            w.write("6. Overall Verdict\n")
            w.write("-" * 80 + "\n")
            # 判定:最大异常率 vs 阈值
            s1_fail_rate = _safe_div(
                self.s1_validated_fail,
                self.s1_validated_pass + self.s1_validated_fail)
            s2_fail_rate = _safe_div(
                self.s2_validated_fail,
                self.s2_validated_pass + self.s2_validated_fail)
            s3_fail_rate = _safe_div(
                self.s3_validated_fail,
                self.s3_validated_pass + self.s3_validated_fail)
            worst = max(s1_fail_rate, s2_fail_rate, s3_fail_rate)

            all_topics_ok = all(self.topic_msg_count.get(t, 0) > 0
                                for t in ALL_TOPICS)
            s2_drop = n_align - n_l181
            s3_drop = n_l181 - n_l543

            w.write("  topic reception: %s\n" %
                    ("all 13 topics received" if all_topics_ok
                     else "WARNING - some topics silent"))
            w.write("  stage 2 frame drop (scan_aligned->l181) : %d\n" % s2_drop)
            w.write("  stage 3 frame drop (l181->l543)         : %d\n" % s3_drop)
            w.write("  worst-stage failure rate                : %.2f%%\n"
                    % (100.0 * worst))
            if (worst <= self.pass_rate and all_topics_ok
                    and abs(s2_drop) <= 1 and abs(s3_drop) <= 1):
                verdict = "PASS"
            elif worst <= self.acceptable_rate and all_topics_ok:
                verdict = "ACCEPTABLE"
            else:
                verdict = "FAIL"
            w.write("\n  VERDICT: %s\n" % verdict)
            if verdict == "PASS":
                w.write("  整条链路数据格式、变换、时序、一致性均通过核验。\n")
            elif verdict == "ACCEPTABLE":
                w.write("  链路基本可用,存在若干非致命异常,建议检查上方\n")
                w.write("  异常明细中计数最高的几类并定位原因。\n")
            else:
                w.write("  链路存在严重异常,建议先解决 anomaly 分类里计数最高\n")
                w.write("  的 3 项后再进入下游 RL 推理测试。\n")
            w.write("\n")

            # ------- 7. 启动同步屏障(v1.1)-------
            w.write("7. Startup Sync Barriers (v1.1)\n")
            w.write("-" * 80 + "\n")
            w.write("  rospy 订阅的各路 TCP 握手不是同时完成的,先就绪的伴随 topic\n"
                    "  会抢到锚之前已发布的旧消息。首条锚到达时必须丢弃这些残留,\n"
                    "  否则 FIFO 配对会永久错位。本次丢弃计数如下:\n\n")
            def _fmt_sync(label, pairs):
                if pairs is None:
                    return "  %-18s [not triggered yet]" % label
                parts = ", ".join("%s=%d" % p for p in pairs)
                total = sum(n for _, n in pairs)
                tag = "OK" if total == 0 else "dropped %d pre-anchor msgs" % total
                return "  %-18s %-52s [%s]" % (label, parts, tag)
            s1d = self._s1_sync_dropped
            s2d = self._s2_sync_dropped
            s3d = self._s3_sync_dropped
            w.write(_fmt_sync("Stage 1", [
                ("invmask",  s1d[0] if s1d else 0),
                ("invstats", s1d[1] if s1d else 0),
            ]) + "\n")
            w.write(_fmt_sync("Stage 2", [
                ("srcmap", s2d[0] if s2d else 0),
                ("rmask",  s2d[1] if s2d else 0),
                ("l181",   s2d[2] if s2d else 0),
                ("fstats", s2d[3] if s2d else 0),
            ]) + "\n")
            w.write(_fmt_sync("Stage 3", [
                ("ids",     s3d[0] if s3d else 0),
                ("ids_vis", s3d[1] if s3d else 0),
                ("stamps",  s3d[2] if s3d else 0),
            ]) + "\n")
            w.write("\n")

            # ------- Part 2: 逐帧追溯 -------
            w.write("=" * 80 + "\n")
            w.write(" " * 22 + "Part 2: Per-lidar_543-frame Traceability\n")
            w.write(" " * 22 + "(共 %d 帧,Part 2 实际记录 %d 帧,上限 %d)\n"
                    % (self.s3_anchored_count, self.s3_frames_written,
                       self.max_frames_written))
            w.write("=" * 80 + "\n")

            # 把 frames 文件的内容接进来
            if os.path.exists(self.frames_tmp_path):
                try:
                    with open(self.frames_tmp_path, "r") as fin:
                        while True:
                            chunk = fin.read(65536)
                            if not chunk: break
                            w.write(chunk)
                except Exception as e:
                    w.write("\n[ERROR] failed to include frames detail: %s\n" % e)

            w.write("\n\n" + "=" * 80 + "\n")
            w.write(" " * 26 + "End of Report\n")
            w.write("=" * 80 + "\n")

        # 清理临时文件
        try:
            os.remove(self.frames_tmp_path)
        except OSError:
            pass


# ============================================================
# main
# ============================================================
def main():
    rospy.init_node("pipeline_verifier", anonymous=False)
    try:
        PipelineVerifierNode()
    except Exception as e:
        rospy.logerr("[pipeline_verifier] init failed: %s", e)
        sys.exit(1)
    rospy.spin()


if __name__ == "__main__":
    main()
