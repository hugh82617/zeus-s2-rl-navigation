#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import datetime
import json
import math
import os
import threading

import numpy as np
import onnxruntime as ort
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import String as StringMsg


LIDAR_FRAME_DIM = 181
LIDAR_FRAMES = 3
LIDAR_TOTAL = LIDAR_FRAME_DIM * LIDAR_FRAMES
OBS_DIM = 3 + 3 + 3 + 4 + LIDAR_TOTAL
ACTION_DIM = 2

S_LIN3 = 0
E_LIN3 = 3
S_ANG3 = 3
E_ANG3 = 6
S_GRAV = 6
E_GRAV = 9
S_GOAL = 9
E_GOAL = 13
S_LIDAR = 13
E_LIDAR = 556

GRAVITY_VEC_DEFAULT = np.array([0.0, 0.0, -1.0], dtype=np.float32)

_LIN_MAX = 5.0
_ANG_MAX = 20.0
_GOAL_DIST_MAX = 100.0


class RLOnnxInferenceNode(object):
    def __init__(self):
        rospy.init_node("rl_onnx_inference_node", anonymous=False)

        # ===== 模型参数 =====
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx",
        )
        self.input_name = rospy.get_param("~input_name", "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        # ===== 话题：严格采用 16 版本 =====
        self.speed_topic = rospy.get_param("~speed_topic", "/zeus_speed")
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_local_goal_body")
        self.goal_status_topic = rospy.get_param("~goal_status_topic", "/rl_local_goal_status")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/rl/lidar_543")
        self.imu_topic = "/imu"  # 保留字段但不订阅
        self.out_cmd_topic = rospy.get_param("~out_cmd_topic", "/rl_cmd_vel")
        self.obs_topic = rospy.get_param("~obs_topic", "/rl/obs_556")

        self.debug_lin_topic = rospy.get_param("~debug_lin_topic", "/rl/debug/lin_vel3")
        self.debug_ang_topic = rospy.get_param("~debug_ang_topic", "/rl/debug/ang_vel3")
        self.debug_goal_recv_topic = rospy.get_param("~debug_goal_recv_topic", "/rl/debug/goal4_received")
        self.debug_goal_used_topic = rospy.get_param("~debug_goal_used_topic", "/rl/debug/goal4_used")
        self.debug_goal_status_echo_topic = rospy.get_param("~debug_goal_status_echo_topic", "/rl/debug/goal_status_echo")
        self.debug_obs0_topic = rospy.get_param("~debug_obs0_topic", "/rl/debug/obs0_181")
        self.debug_obs1_topic = rospy.get_param("~debug_obs1_topic", "/rl/debug/obs1_181")
        self.debug_obs2_topic = rospy.get_param("~debug_obs2_topic", "/rl/debug/obs2_181")
        self.debug_speed_age_topic = rospy.get_param("~debug_speed_age_topic", "/rl/debug/speed_age_ms")
        self.debug_goal_age_topic = rospy.get_param("~debug_goal_age_topic", "/rl/debug/goal_age_ms")
        self.debug_age_pair_topic = rospy.get_param("~debug_age_pair_topic", "/rl/debug/sync_age_pair_ms")
        self.debug_obs_meta_topic = rospy.get_param("~debug_obs_meta_topic", "/rl/debug/obs_meta")

        # ===== 运行参数 =====
        # 重力向量固定为 [0, 0, -1]，不使用 IMU
        self.goal_nonzero_eps = float(rospy.get_param("~goal_nonzero_eps", 1e-4))
        self.goal_pos_eps_enter = float(rospy.get_param("~goal_pos_eps_enter", 0.05))
        self.goal_pos_eps_exit = float(rospy.get_param("~goal_pos_eps_exit", 0.02))
        self.goal_yaw_eps_enter = float(rospy.get_param("~goal_yaw_eps_enter", 0.05))
        self.goal_yaw_eps_exit = float(rospy.get_param("~goal_yaw_eps_exit", 0.02))
        self.goal_small_value_eps = float(rospy.get_param("~goal_small_value_eps", 1e-6))
        self.use_split_goal_gate = bool(rospy.get_param("~use_split_goal_gate", True))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.20))
        self._goal_active = False

        # ===== 稳定到达判定参数（距离到达 + 双零停稳 + 连续保持 N 帧）=====
        # goal_reached 由本节点在 _inference_loop 中按下列稳定准则唯一产生，
        # 不再使用单纯 dist_now<=goal_tolerance 的瞬时判定，
        # 也不再允许外部独立的 goal_reached 逻辑并存。
        self.stable_stop_min_count = int(rospy.get_param("~stable_stop_min_count", 3))
        self.stable_stop_lin_eps = float(rospy.get_param("~stable_stop_lin_eps", 1e-3))
        self.stable_stop_ang_eps = float(rospy.get_param("~stable_stop_ang_eps", 1e-3))

        self.zero_goal_on_status_clear = bool(rospy.get_param("~zero_goal_on_status_clear", True))
        self.zero_cmd_on_goal_cleared = bool(rospy.get_param("~zero_cmd_on_goal_cleared", True))
        self.zero_cmd_on_goal_reached = bool(rospy.get_param("~zero_cmd_on_goal_reached", True))
        self.publish_zero_when_goal_reached = bool(rospy.get_param("~publish_zero_when_goal_reached", True))
        self.publish_obs_when_goal_invalid = bool(rospy.get_param("~publish_obs_when_goal_invalid", False))
        self.zero_cmd_when_goal_invalid = bool(rospy.get_param("~zero_cmd_when_goal_invalid", True))

        # 动作映射链：直接将 raw action 映射到物理范围，不再进行 [-1, 1] clip
        self.use_clip = False
        self.action_clip_low = -1.0
        self.action_clip_hi = 1.0
        self.linear_min = float(rospy.get_param("~linear_min", -0.2))
        self.linear_max = float(rospy.get_param("~linear_max", 0.5))
        self.angular_min = float(rospy.get_param("~angular_min", -1.0))
        self.angular_max = float(rospy.get_param("~angular_max", 1.0))

        self.publish_obs = bool(rospy.get_param("~publish_obs", True))
        self.debug_publish_inputs = bool(rospy.get_param("~debug_publish_inputs", True))
        self.debug_log = bool(rospy.get_param("~debug_log", True))
        self.debug_obs_log = bool(rospy.get_param("~debug_obs_log", False))
        self.debug_age_log = bool(rospy.get_param("~debug_age_log", True))

        self.warn_speed_age_ms = float(rospy.get_param("~warn_speed_age_ms", 60.0))
        self.warn_goal_age_ms = float(rospy.get_param("~warn_goal_age_ms", 80.0))
        self.drop_if_speed_too_old = bool(rospy.get_param("~drop_if_speed_too_old", False))
        self.drop_if_goal_too_old = bool(rospy.get_param("~drop_if_goal_too_old", False))

        # 采用 20 的输入自校验阈值
        self.lidar_max_range = float(rospy.get_param("~lidar_max_range", 30.0))
        self.val_lin_max = float(rospy.get_param("~val_lin_max", _LIN_MAX))
        self.val_ang_max = float(rospy.get_param("~val_ang_max", _ANG_MAX))
        self.val_goal_dist_max = float(rospy.get_param("~val_goal_dist_max", _GOAL_DIST_MAX))

        self.action_log_path = rospy.get_param("~action_log_path", "/home/ros/4.txt")
        self.meta_log_path = rospy.get_param("~meta_log_path", "/home/ros/obs_meta.jsonl")

        # ===== 内部状态 =====
        self.initialized = False
        self.has_speed = False
        self.has_goal = False
        self.has_lidar = False
        self.goal_done_logged = False
        self.goal_reached = False
        # 稳定到达判定相关：双零停稳（|v|≈0 且 |w|≈0）连续保持的采样点计数
        self._stable_stop_count = 0
        # /rl_local_goal_status 状态机发布标志（每个新 goal 只发一次，防止回环重置 _stable_stop_count）
        # goal_received_published：cb_goal 发布 goal_received 之后置 True
        # goal_tracking_published：_inference_loop 首次成功发布 /rl_cmd_vel 后置 True
        self.goal_received_published = False
        self.goal_tracking_published = False
        self._last_wait_state = None

        self._src_lin3 = np.zeros(3, dtype=np.float32)
        self._src_ang3 = np.zeros(3, dtype=np.float32)
        self._src_goal4 = np.zeros(4, dtype=np.float32)
        self._src_lidar = np.full(LIDAR_TOTAL, self.lidar_max_range, dtype=np.float32)
        self._src_gravity3 = GRAVITY_VEC_DEFAULT.copy()
        self._obs_buf = np.zeros(OBS_DIM, dtype=np.float32)

        self.t_speed = None
        self.t_goal = None
        self.t_lidar = None
        self.latest_goal_status = "none"

        self._data_lock = threading.Lock()
        self._infer_event = threading.Event()
        self._infer_thread = threading.Thread(target=self._inference_loop, name="rl_infer", daemon=True)

        self._ensure_action_log_file()
        self._meta_fp = None
        try:
            self._meta_fp = open(self.meta_log_path, "a", 1)
        except Exception as e:
            rospy.logwarn("[RL] 无法打开 meta 日志文件 %s: %s", self.meta_log_path, e)

        # ===== ONNX =====
        rospy.loginfo("[RL] loading ONNX model: %s", self.model_path)
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])

        in_names = [x.name for x in self.session.get_inputs()]
        out_names = [x.name for x in self.session.get_outputs()]
        if self.input_name not in in_names:
            self.input_name = in_names[0]
            rospy.logwarn("[RL] input_name 回退到 '%s'", self.input_name)
        if self.output_name not in out_names:
            self.output_name = out_names[0]
            rospy.logwarn("[RL] output_name 回退到 '%s'", self.output_name)

        _dummy = np.zeros((1, OBS_DIM), dtype=np.float32)
        self.session.run([self.output_name], {self.input_name: _dummy})
        rospy.loginfo("[RL] model warmup done")

        # ===== 发布者 =====
        self.pub_cmd = rospy.Publisher(self.out_cmd_topic, Twist, queue_size=10)
        self.pub_obs = rospy.Publisher(self.obs_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_lin = rospy.Publisher(self.debug_lin_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_ang = rospy.Publisher(self.debug_ang_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_goal_recv = rospy.Publisher(self.debug_goal_recv_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_goal_used = rospy.Publisher(self.debug_goal_used_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_goal_status_echo = rospy.Publisher(self.debug_goal_status_echo_topic, StringMsg, queue_size=10)
        self.pub_debug_obs0 = rospy.Publisher(self.debug_obs0_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_obs1 = rospy.Publisher(self.debug_obs1_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_obs2 = rospy.Publisher(self.debug_obs2_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_speed_age = rospy.Publisher(self.debug_speed_age_topic, Float32, queue_size=10)
        self.pub_debug_goal_age = rospy.Publisher(self.debug_goal_age_topic, Float32, queue_size=10)
        self.pub_debug_age_pair = rospy.Publisher(self.debug_age_pair_topic, Float32MultiArray, queue_size=10)
        self.pub_debug_obs_meta = rospy.Publisher(self.debug_obs_meta_topic, StringMsg, queue_size=10)

        # /rl_local_goal_status 为 RL 任务“正式到达”的唯一输出话题，
        # 由本节点在稳定到达判定通过后统一发布 goal_reached。
        self.pub_goal_status = rospy.Publisher(self.goal_status_topic, StringMsg, queue_size=10)

        # ===== 订阅者：严格采用 16 版本 =====
        rospy.Subscriber(self.speed_topic, Twist, self.cb_speed, queue_size=50)
        rospy.Subscriber(self.goal_topic, Float32MultiArray, self.cb_goal, queue_size=50)
        rospy.Subscriber(self.goal_status_topic, StringMsg, self.cb_goal_status, queue_size=50)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=50)
        rospy.loginfo("[RL] gravity fixed [0, 0, -1], IMU disabled")

        rospy.on_shutdown(self._on_shutdown)
        self._infer_thread.start()
        self.initialized = True
        self.obs_seq = 0

        # ── 启动快照：首帧推理成功后写一次，用于验证各路数据时刻对齐 ──
        self._snapshot_done = False
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._snapshot_path = "/home/ros/rl_input_snapshot_{}.txt".format(_ts)

        rospy.loginfo("[RL] subscribed: speed=%s goal=%s goal_status=%s lidar=%s",
                      self.speed_topic, self.goal_topic, self.goal_status_topic, self.lidar_topic)
        rospy.loginfo("[RL] publish: cmd=%s obs=%s", self.out_cmd_topic, self.obs_topic)
        rospy.loginfo("[RL] mode = fused(20-core + 16-topics/debug)")

    # ===== 工具函数 =====
    @staticmethod
    def _to_msg(arr):
        msg = Float32MultiArray()
        msg.data = np.asarray(arr, dtype=np.float32).tolist()
        return msg

    def safe_publish(self, pub, msg, tag=""):
        if rospy.is_shutdown() or pub is None:
            return
        try:
            pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[RL] publish error [%s]: %s", tag, e)

    def _ensure_action_log_file(self):
        log_dir = os.path.dirname(self.action_log_path)
        if log_dir and not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        with open(self.action_log_path, "a") as f:
            if os.path.getsize(self.action_log_path) == 0:
                f.write("time,raw_v,raw_w,clip_v,clip_w,cmd_v,cmd_w\n")

    def _append_action_log(self, raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w):
        try:
            now = rospy.Time.now().to_sec()
            with open(self.action_log_path, "a") as f:
                f.write("{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(
                    now, raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w
                ))
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[RL] failed to write action log %s: %s", self.action_log_path, str(e))

    def _write_meta(self, obj):
        try:
            line = json.dumps(obj, ensure_ascii=False)
        except Exception:
            return
        try:
            self.pub_debug_obs_meta.publish(StringMsg(data=line))
        except Exception:
            pass
        if self._meta_fp is not None:
            try:
                self._meta_fp.write(line + "\n")
            except Exception:
                pass

    @staticmethod
    def _age_ms(newer, older):
        if newer is None or older is None:
            return None
        return (newer - older).to_sec() * 1000.0

    def _publish_age_topics(self, speed_age_ms, goal_age_ms):
        if speed_age_ms is not None:
            self.safe_publish(self.pub_debug_speed_age, Float32(data=float(speed_age_ms)), "speed_age")
        if goal_age_ms is not None:
            self.safe_publish(self.pub_debug_goal_age, Float32(data=float(goal_age_ms)), "goal_age")
        pair = Float32MultiArray()
        pair.data = [
            -1.0 if speed_age_ms is None else float(speed_age_ms),
            -1.0 if goal_age_ms is None else float(goal_age_ms),
        ]
        self.safe_publish(self.pub_debug_age_pair, pair, "age_pair")

    def _maybe_log_waiting(self):
        missing = []
        if not self.has_speed:
            missing.append("speed")
        if not self.has_goal:
            missing.append("goal")
        if not self.has_lidar:
            missing.append("lidar")
        state = tuple(missing)
        if state != self._last_wait_state:
            self._last_wait_state = state
            if missing:
                rospy.loginfo("[RL] 等待话题就绪: %s", ", ".join(missing))

    def _goal_valid_locked(self):
        goal4 = self._src_goal4.astype(np.float32)
        if self.use_split_goal_gate:
            pos_norm = float(np.linalg.norm(goal4[:3]))
            yaw_abs = float(abs(goal4[3]))
            if self._goal_active:
                self._goal_active = ((pos_norm > self.goal_pos_eps_exit) or (yaw_abs > self.goal_yaw_eps_exit))
            else:
                self._goal_active = ((pos_norm > self.goal_pos_eps_enter) or (yaw_abs > self.goal_yaw_eps_enter))
        else:
            goal_norm = float(np.linalg.norm(goal4))
            if self._goal_active:
                self._goal_active = goal_norm > self.goal_nonzero_eps * 0.5
            else:
                self._goal_active = goal_norm > self.goal_nonzero_eps
        return self._goal_active

    def _build_obs_556_locked(self):
        self._obs_buf[S_LIN3:E_LIN3] = self._src_lin3
        self._obs_buf[S_ANG3:E_ANG3] = self._src_ang3
        self._obs_buf[S_GRAV:E_GRAV] = self._src_gravity3
        self._obs_buf[S_GOAL:E_GOAL] = self._src_goal4
        self._obs_buf[S_LIDAR:E_LIDAR] = self._src_lidar
        return np.ascontiguousarray(self._obs_buf.reshape(1, OBS_DIM))

    def _validate_and_correct(self, obs_1d):
        fatal = []
        warn = []

        seg = obs_1d[S_LIN3:E_LIN3]
        if not np.all(np.isfinite(seg)):
            obs_1d[S_LIN3:E_LIN3] = 0.0
            warn.append("lin3[0:3] NaN/Inf -> 已置零")
        elif np.any(np.abs(seg) > self.val_lin_max):
            warn.append("lin3[0:3] 超阈值: %s" % seg.tolist())

        seg = obs_1d[S_ANG3:E_ANG3]
        if not np.all(np.isfinite(seg)):
            obs_1d[S_ANG3:E_ANG3] = 0.0
            warn.append("ang3[3:6] NaN/Inf -> 已置零")
        elif np.any(np.abs(seg) > self.val_ang_max):
            warn.append("ang3[3:6] 超阈值: %s" % seg.tolist())

        seg = obs_1d[S_GRAV:E_GRAV]
        if not np.all(np.isfinite(seg)):
            obs_1d[S_GRAV:E_GRAV] = GRAVITY_VEC_DEFAULT
            warn.append("gravity[6:9] NaN/Inf -> 已恢复默认重力")

        seg = obs_1d[S_GOAL:E_GOAL]
        if not np.all(np.isfinite(seg)):
            fatal.append("goal4[9:13] NaN/Inf=%s，丢弃本帧" % seg.tolist())
        else:
            dist = math.sqrt(float(seg[0]) ** 2 + float(seg[1]) ** 2)
            if dist > self.val_goal_dist_max:
                warn.append("goal4[9:13] dist=%.3f 过大，可能陈旧目标" % dist)

        seg = obs_1d[S_LIDAR:E_LIDAR]
        bad = ~np.isfinite(seg)
        n_bad = int(bad.sum())
        if n_bad > 0:
            seg[bad] = self.lidar_max_range
            warn.append("lidar[13:556] %d 点 NaN/Inf -> 已填充 %.1f" % (n_bad, self.lidar_max_range))
        if float(seg.min()) < 0.0:
            warn.append("lidar[13:556] 出现负值")
        if float(seg.max()) > self.lidar_max_range * 1.5:
            warn.append("lidar[13:556] 超量程异常")

        if obs_1d.shape[0] != OBS_DIM:
            fatal.append("obs 总维度=%d != %d" % (obs_1d.shape[0], OBS_DIM))

        return len(fatal) == 0, fatal, warn

    def _map_action(self, raw_action):
        a = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if a.shape[0] < 2:
            raise ValueError("action 维度=%d < 2" % a.shape[0])
        raw_v, raw_w = float(a[0]), float(a[1])

        # 不再执行 [-1, 1] 裁剪，raw action 直接参与动作映射
        cmd_v = (raw_v + 1.0) * 0.5 * (self.linear_max - self.linear_min) + self.linear_min
        cmd_w = (raw_w + 1.0) * 0.5 * (self.angular_max - self.angular_min) + self.angular_min

        # 为了兼容原有日志字段，clip_v / clip_w 直接记录 raw_v / raw_w
        clip_v, clip_w = raw_v, raw_w
        return raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w

    # ===== 启动快照 =====
    def _write_startup_snapshot(self, t_speed, t_goal, t_lidar,
                                 speed_age_ms, goal_age_ms,
                                 raw_lin3, raw_ang3, raw_goal4, raw_lidar543,
                                 obs_1d):
        """
        节点启动后首帧推理成功时调用一次。
        分两大块：
          A. 原始话题接收值（附采集时刻）
          B. 实际输入模型的 obs_556 各段值（附来源时刻）
        两块数值应完全一致，时刻差越小越好。
        """
        def fmt_t(t):
            if t is None:
                return "None"
            return "{:.6f} s".format(t.to_sec())

        def fmt_age(ms):
            if ms is None:
                return "None"
            return "{:.3f} ms".format(ms)

        def lidar_block(arr, base_idx, label_prefix):
            """把 arr 的前20个值逐行打印，base_idx 是全局索引偏移"""
            rows = []
            for i, v in enumerate(arr[:20]):
                rows.append("    [{:>3d}]  {:.6f}".format(base_idx + i, float(v)))
            rows.append("    ... ({} 维共 {} 个值，仅展示前20)".format(label_prefix, len(arr)))
            return rows

        L = []  # 输出行列表

        # ════════════════════════════════════════════════════════
        L.append("=" * 68)
        L.append("  RL 节点启动输入快照  —  用于验证各路数据时刻对齐")
        L.append("  生成时间 : {}".format(
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        L.append("  快照文件 : {}".format(self._snapshot_path))
        L.append("=" * 68)
        L.append("")

        # ── 时刻总览 ─────────────────────────────────────────
        L.append("┌─ 时刻总览 " + "─" * 56 + "┐")
        L.append("│  t_speed  (线速度/角速度 callback 到达时刻)")
        L.append("│      = {}".format(fmt_t(t_speed)))
        L.append("│  t_goal   (目标向量 callback 到达时刻)")
        L.append("│      = {}".format(fmt_t(t_goal)))
        L.append("│  t_lidar  (激光雷达 callback 到达时刻，推理触发基准)")
        L.append("│      = {}".format(fmt_t(t_lidar)))
        L.append("│")
        L.append("│  speed_age_ms = t_lidar - t_speed = {}".format(fmt_age(speed_age_ms)))
        L.append("│  goal_age_ms  = t_lidar - t_goal  = {}".format(fmt_age(goal_age_ms)))
        L.append("│")
        L.append("│  ★ age 越接近 0ms 越好，>100ms 需排查同步问题")
        L.append("└" + "─" * 67 + "┘")
        L.append("")

        # ════════════════════════════════════════════════════════
        L.append("A. 原始话题接收值  （来自各 callback，未经处理）")
        L.append("=" * 68)
        L.append("")

        # A1 线速度
        L.append("  [A1] 线速度 lin3   话题: {}".format(self.speed_topic))
        L.append("       采集时刻: {}".format(fmt_t(t_speed)))
        L.append("       vx  [0] = {:.6f}".format(float(raw_lin3[0])))
        L.append("       vy  [1] = {:.6f}".format(float(raw_lin3[1])))
        L.append("       vz  [2] = {:.6f}".format(float(raw_lin3[2])))
        L.append("")

        # A2 角速度
        L.append("  [A2] 角速度 ang3   话题: {}".format(self.speed_topic))
        L.append("       采集时刻: {}".format(fmt_t(t_speed)))
        L.append("       wx  [3] = {:.6f}".format(float(raw_ang3[0])))
        L.append("       wy  [4] = {:.6f}".format(float(raw_ang3[1])))
        L.append("       wz  [5] = {:.6f}".format(float(raw_ang3[2])))
        L.append("")

        # A3 目标向量
        L.append("  [A3] 目标向量 goal4   话题: {}".format(self.goal_topic))
        L.append("       采集时刻: {}".format(fmt_t(t_goal)))
        L.append("       dx   [0] = {:.6f}".format(float(raw_goal4[0])))
        L.append("       dy   [1] = {:.6f}".format(float(raw_goal4[1])))
        L.append("       dz   [2] = {:.6f}".format(float(raw_goal4[2])))
        L.append("       dyaw [3] = {:.6f}".format(float(raw_goal4[3])))
        L.append("")

        # A4 激光雷达前20
        L.append("  [A4] 激光雷达 lidar543   话题: {}".format(self.lidar_topic))
        L.append("       采集时刻: {}".format(fmt_t(t_lidar)))
        L.append("       共 {} 维（3帧 × {} 点/帧），以下展示前20位".format(
            len(raw_lidar543), LIDAR_FRAME_DIM))
        for i in range(20):
            L.append("       [{:>3d}] = {:.6f}".format(i, float(raw_lidar543[i])))
        L.append("       ...")
        L.append("")

        # ════════════════════════════════════════════════════════
        L.append("B. 实际输入模型的 obs_556  （t_lidar 时刻组装，共 {} 维）".format(OBS_DIM))
        L.append("=" * 68)
        L.append("")

        # B1 线速度段
        L.append("  [B1] obs[0:3]  线速度 lin3   来源: {}".format(self.speed_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_speed)))
        L.append("       速度差(t_lidar - t_speed): {}".format(fmt_age(speed_age_ms)))
        L.append("       obs[0]  vx = {:.6f}".format(float(obs_1d[0])))
        L.append("       obs[1]  vy = {:.6f}".format(float(obs_1d[1])))
        L.append("       obs[2]  vz = {:.6f}".format(float(obs_1d[2])))
        L.append("")

        # B2 角速度段
        L.append("  [B2] obs[3:6]  角速度 ang3   来源: {}".format(self.speed_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_speed)))
        L.append("       obs[3]  wx = {:.6f}".format(float(obs_1d[3])))
        L.append("       obs[4]  wy = {:.6f}".format(float(obs_1d[4])))
        L.append("       obs[5]  wz = {:.6f}".format(float(obs_1d[5])))
        L.append("")

        # B3 重力段
        L.append("  [B3] obs[6:9]  重力向量 grav   固定常量，无时刻概念")
        L.append("       obs[6]  gx = {:.6f}".format(float(obs_1d[6])))
        L.append("       obs[7]  gy = {:.6f}".format(float(obs_1d[7])))
        L.append("       obs[8]  gz = {:.6f}".format(float(obs_1d[8])))
        L.append("")

        # B4 目标向量段
        L.append("  [B4] obs[9:13]  目标向量 goal4   来源: {}".format(self.goal_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_goal)))
        L.append("       目标差(t_lidar - t_goal): {}".format(fmt_age(goal_age_ms)))
        L.append("       obs[9]   dx   = {:.6f}".format(float(obs_1d[9])))
        L.append("       obs[10]  dy   = {:.6f}".format(float(obs_1d[10])))
        L.append("       obs[11]  dz   = {:.6f}".format(float(obs_1d[11])))
        L.append("       obs[12]  dyaw = {:.6f}".format(float(obs_1d[12])))
        L.append("")

        # B5 激光雷达段前20
        L.append("  [B5] obs[13:556]  激光雷达 lidar543   来源: {}".format(self.lidar_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_lidar)))
        L.append("       共 {} 维（3帧 × {} 点/帧），以下展示前20位".format(
            E_LIDAR - S_LIDAR, LIDAR_FRAME_DIM))
        for i in range(20):
            L.append("       obs[{:>3d}] = {:.6f}".format(S_LIDAR + i, float(obs_1d[S_LIDAR + i])))
        L.append("       ...")
        L.append("")

        # ── A vs B 对比确认 ──────────────────────────────────
        L.append("─" * 68)
        L.append("C. A vs B 数值一致性确认")
        L.append("   （A 和 B 来自同一次锁内快照，理论上完全相同）")
        L.append("")
        diffs = []
        pairs = [
            ("vx",   raw_lin3[0],  obs_1d[0]),
            ("vy",   raw_lin3[1],  obs_1d[1]),
            ("vz",   raw_lin3[2],  obs_1d[2]),
            ("wx",   raw_ang3[0],  obs_1d[3]),
            ("wy",   raw_ang3[1],  obs_1d[4]),
            ("wz",   raw_ang3[2],  obs_1d[5]),
            ("dx",   raw_goal4[0], obs_1d[9]),
            ("dy",   raw_goal4[1], obs_1d[10]),
            ("dz",   raw_goal4[2], obs_1d[11]),
            ("dyaw", raw_goal4[3], obs_1d[12]),
        ]
        for name, a_val, b_val in pairs:
            diff = abs(float(a_val) - float(b_val))
            flag = "  ✓" if diff < 1e-7 else "  ✗ diff={:.2e}".format(diff)
            L.append("   {:5s}: A={:.6f}  B={:.6f}{}".format(
                name, float(a_val), float(b_val), flag))
            if diff >= 1e-7:
                diffs.append(name)
        L.append("")
        if diffs:
            L.append("   ⚠ 以下字段 A/B 不一致（不应出现）: {}".format(", ".join(diffs)))
        else:
            L.append("   ✓ 所有字段 A/B 完全一致")
        L.append("")

        # ── 时序结论 ─────────────────────────────────────────
        L.append("─" * 68)
        L.append("D. 时序结论")
        ok = True
        if speed_age_ms is not None and speed_age_ms > 100.0:
            L.append("   ⚠ speed_age_ms = {:.1f} ms  超过 100ms，速度数据偏旧！".format(speed_age_ms))
            ok = False
        elif speed_age_ms is not None:
            L.append("   ✓ speed_age_ms = {:.1f} ms  正常".format(speed_age_ms))
        if goal_age_ms is not None and goal_age_ms > 100.0:
            L.append("   ⚠ goal_age_ms  = {:.1f} ms  超过 100ms，目标数据偏旧！".format(goal_age_ms))
            ok = False
        elif goal_age_ms is not None:
            L.append("   ✓ goal_age_ms  = {:.1f} ms  正常".format(goal_age_ms))
        if ok:
            L.append("   ✓ 各路数据时刻差均在 100ms 以内，时序对齐正常")
        L.append("=" * 68)

        try:
            with open(self._snapshot_path, "w") as f:
                f.write("\n".join(L) + "\n")
            rospy.loginfo("[RL][SNAPSHOT] 启动快照已写入: %s", self._snapshot_path)
        except Exception as e:
            rospy.logwarn("[RL][SNAPSHOT] 写入失败: %s", e)

    # ===== 回调 =====
    def cb_speed(self, msg):
        if not self.initialized:
            return
        with self._data_lock:
            self._src_lin3[:] = [msg.linear.x, msg.linear.y, msg.linear.z]
            self._src_ang3[:] = [msg.angular.x, msg.angular.y, msg.angular.z]
            self.t_speed = rospy.Time.now()
            self.has_speed = True
            lin_copy = self._src_lin3.copy()
            ang_copy = self._src_ang3.copy()
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_lin, self._to_msg(lin_copy), "debug_lin")
            self.safe_publish(self.pub_debug_ang, self._to_msg(ang_copy), "debug_ang")

    def cb_goal(self, msg):
        if not self.initialized:
            return
        data = np.asarray(msg.data, dtype=np.float32)
        if data.shape != (4,):
            rospy.logwarn_throttle(1.0, "[RL] %s shape != (4,), got %s", self.goal_topic, data.shape)
            return
        data[np.abs(data) < self.goal_small_value_eps] = 0.0
        with self._data_lock:
            self._src_goal4[:] = data
            self.t_goal = rospy.Time.now()
            self.has_goal = True
            # 新目标到达 → 清零所有与“稳定到达判定”相关的内部状态，
            # 保证每个新目标都从零开始重新积累“双零停稳”连续帧计数。
            self.goal_done_logged = False
            self.goal_reached = False
            self._stable_stop_count = 0
            # 同时清零本轮 /rl_local_goal_status 的状态发布标志：
            # 即将由本回调发布 goal_received，_inference_loop 稍后在首次成功推理+控制时发布 goal_tracking。
            self.goal_received_published = False
            self.goal_tracking_published = False
            self._last_wait_state = None
            goal_copy = self._src_goal4.copy()
            t_event = rospy.Time.now()
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_goal_recv, self._to_msg(goal_copy), "debug_goal_recv")

        # ── 统一发布 goal_received（每个新 goal 只发一次）──────────
        # cb_goal 本身每条消息只触发一次，这里用 goal_received_published 做一次性保护；
        # 发布放在锁外，避免与推理线程争用。
        emit_received = False
        with self._data_lock:
            if not self.goal_received_published:
                self.goal_received_published = True
                emit_received = True

        if emit_received:
            self.safe_publish(
                self.pub_goal_status,
                StringMsg(data="goal_received"),
                "goal_status_received",
            )
            rospy.loginfo(
                "[RL] *** goal_received | goal4=[%+.4f,%+.4f,%+.4f,%+.4f] ***",
                float(goal_copy[0]), float(goal_copy[1]),
                float(goal_copy[2]), float(goal_copy[3]),
            )
            self._write_meta({
                "type": "goal_status_event",
                "event": "goal_received",
                "t_event": round(t_event.to_sec(), 6),
                "goal4": [round(float(x), 6) for x in goal_copy],
            })

    def cb_goal_status(self, msg):
        if not self.initialized:
            return
        status = msg.data.strip()
        self.safe_publish(self.pub_debug_goal_status_echo, StringMsg(data=status), "goal_status_echo")

        zero_goal_copy = None
        publish_zero_cmd = False
        with self._data_lock:
            # 边沿触发：只有当收到的状态与 latest_goal_status 不同，才执行“新状态切换”对应的重置逻辑。
            # 这是为了应对“RL 节点既订阅又发布 /rl_local_goal_status”导致的自发自收回环：
            # 同值重复不得再次清零 _stable_stop_count、goal_reached 等内部状态，否则稳定到达判定永远达不成。
            prev_status = self.latest_goal_status
            is_edge = (status != prev_status)
            self.latest_goal_status = status

            if is_edge and status in ("goal_received", "goal_tracking"):
                # 真正的 none → goal_received / goal_received → goal_tracking 等新跟踪周期
                # 才清零“稳定到达判定”相关内部状态；同值重复不触发。
                self.goal_done_logged = False
                self.goal_reached = False
                self._stable_stop_count = 0

            # 注意：goal_reached 已改由本节点在 _inference_loop 中按
            # “距离到达 + 双零停稳 + 连续保持 N 帧”的稳定准则唯一产生，
            # cb_goal_status 不再对外部传入的 "goal_reached" 做独立置位，
            # 仅在真正的清理类状态（goal_cleared / shutdown）下做通用清理。
            if is_edge and self.zero_goal_on_status_clear and status in ("goal_cleared", "shutdown"):
                self._src_goal4[:] = 0.0
                self.t_goal = rospy.Time.now()
                self._goal_active = False
                self._stable_stop_count = 0
                # 清理态进入，意味着本轮 goal 彻底结束，状态发布标志也重置，
                # 以便下一个新目标到来时 cb_goal 能干净地重新开始本轮状态机。
                self.goal_received_published = False
                self.goal_tracking_published = False
                zero_goal_copy = self._src_goal4.copy()

            if is_edge and (status == "goal_cleared") and self.zero_cmd_on_goal_cleared:
                publish_zero_cmd = True

        if zero_goal_copy is not None:
            self.safe_publish(self.pub_debug_goal_used, self._to_msg(zero_goal_copy), "debug_goal_used_zero")
        if publish_zero_cmd:
            self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_goal_status")

    def cb_lidar(self, msg):
        if not self.initialized:
            return
        data = np.asarray(msg.data, dtype=np.float32)
        if data.shape != (LIDAR_TOTAL,):
            rospy.logwarn_throttle(1.0, "[RL] %s shape != (%d,), got %s", self.lidar_topic, LIDAR_TOTAL, data.shape)
            return

        with self._data_lock:
            if self.goal_reached:
                return
            self._src_lidar[:] = data
            self.t_lidar = rospy.Time.now()
            self.has_lidar = True
            lidar_copy = self._src_lidar.copy() if self.debug_publish_inputs else None
            speed_age_ms = self._age_ms(self.t_lidar, self.t_speed)
            goal_age_ms = self._age_ms(self.t_lidar, self.t_goal)

        if lidar_copy is not None:
            s = LIDAR_FRAME_DIM
            self.safe_publish(self.pub_debug_obs0, self._to_msg(lidar_copy[0:s]), "debug_obs0")
            self.safe_publish(self.pub_debug_obs1, self._to_msg(lidar_copy[s:2 * s]), "debug_obs1")
            self.safe_publish(self.pub_debug_obs2, self._to_msg(lidar_copy[2 * s:3 * s]), "debug_obs2")

        self._publish_age_topics(speed_age_ms, goal_age_ms)
        self._infer_event.set()

    # ===== 推理线程 =====
    def _inference_loop(self):
        while not rospy.is_shutdown():
            triggered = self._infer_event.wait(timeout=0.5)
            self._infer_event.clear()
            if not triggered:
                continue

            # 本帧稳定到达判定是否触发，None 表示未触发；触发时捕获日志所需的现场快照，
            # 在释放 _data_lock 后统一发布零速度 / goal_reached / full log。
            arrival_info = None

            with self._data_lock:
                if self.goal_reached:
                    continue
                if not (self.has_speed and self.has_goal and self.has_lidar):
                    self._maybe_log_waiting()
                    continue

                goal_valid = self._goal_valid_locked()
                t_ref = self.t_lidar
                t_speed_snap = self.t_speed
                t_goal_snap = self.t_goal
                speed_age_ms = self._age_ms(t_ref, t_speed_snap)
                goal_age_ms = self._age_ms(t_ref, t_goal_snap)

                if self.debug_age_log and goal_valid:
                    speed_str = "None" if speed_age_ms is None else "{:.1f}".format(speed_age_ms)
                    goal_str = "None" if goal_age_ms is None else "{:.1f}".format(goal_age_ms)
                    rospy.loginfo_throttle(0.5,
                                           "[RL][SYNC] lidar-trigger speed_age_ms=%s goal_age_ms=%s goal_status=%s",
                                           speed_str, goal_str, self.latest_goal_status)

                if speed_age_ms is not None and speed_age_ms > self.warn_speed_age_ms:
                    rospy.logwarn_throttle(0.5, "[RL][SYNC] speed_age_ms=%.1f > warn_speed_age_ms=%.1f",
                                           speed_age_ms, self.warn_speed_age_ms)
                    if self.drop_if_speed_too_old:
                        self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_speed_old")
                        continue

                if goal_age_ms is not None and goal_age_ms > self.warn_goal_age_ms:
                    rospy.logwarn_throttle(0.5, "[RL][SYNC] goal_age_ms=%.1f > warn_goal_age_ms=%.1f",
                                           goal_age_ms, self.warn_goal_age_ms)
                    if self.drop_if_goal_too_old:
                        self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_goal_old")
                        continue

                dx = float(self._src_goal4[0])
                dy = float(self._src_goal4[1])
                dz = float(self._src_goal4[2])
                dyaw = float(self._src_goal4[3])
                dist_now = math.sqrt(dx * dx + dy * dy)

                # ── 稳定到达判定 ──────────────────────────────────────
                # 条件：
                #   (1) 当前目标处于有效执行态（goal_valid）
                #   (2) 机器人进入目标距离范围：dist_now <= goal_tolerance
                #   (3) 实际线速度≈0 且 实际角速度≈0（双零停稳）
                #   (4) 上述“双零停稳”状态连续保持不少于 stable_stop_min_count 个采样点
                # 同一目标只触发一次（依赖 self.goal_reached 标志 + cb_goal() 的重置）。
                lin_speed_now = float(np.linalg.norm(self._src_lin3))
                ang_speed_now = float(np.linalg.norm(self._src_ang3))
                in_range = (dist_now <= self.goal_tolerance)
                is_stopped = (
                    lin_speed_now < self.stable_stop_lin_eps
                    and ang_speed_now < self.stable_stop_ang_eps
                )

                if goal_valid and in_range and is_stopped:
                    self._stable_stop_count += 1
                else:
                    self._stable_stop_count = 0

                stable_count_now = self._stable_stop_count

                if (
                    goal_valid
                    and in_range
                    and is_stopped
                    and stable_count_now >= self.stable_stop_min_count
                ):
                    # 正式认定到达：
                    #   - 置 self.goal_reached = True（锁内，避免与 cb_lidar 竞态）
                    #   - 同时置 goal_done_logged = True 抑制可能的重复日志
                    #   - 捕获现场快照（距离 / 速度 / 连续保持采样点数）
                    #   - 零速度发布、/rl_local_goal_status=goal_reached、full log 统一在锁外完成
                    self.goal_reached = True
                    self.goal_done_logged = True
                    arrival_info = {
                        "dist_now": dist_now,
                        "lin_speed": lin_speed_now,
                        "ang_speed": ang_speed_now,
                        "stable_count": stable_count_now,
                        "lin3": self._src_lin3.tolist(),
                        "ang3": self._src_ang3.tolist(),
                        "goal4": [dx, dy, dz, dyaw],
                        "goal_tolerance": self.goal_tolerance,
                        "stable_stop_min_count": self.stable_stop_min_count,
                        "stable_stop_lin_eps": self.stable_stop_lin_eps,
                        "stable_stop_ang_eps": self.stable_stop_ang_eps,
                        "goal_valid": goal_valid,
                        "latest_goal_status": self.latest_goal_status,
                    }
                else:
                    obs_2d = self._build_obs_556_locked()
                    obs_1d = obs_2d.reshape(-1)
                    speed6_snap = self._src_lin3.tolist() + self._src_ang3.tolist()
                    goal4_snap = self._src_goal4.tolist()
                    goal_copy = self._src_goal4.copy()

            # ── 锁外：稳定到达认定后的统一处理（同一目标仅触发一次）──
            if arrival_info is not None:
                # 1) 发布零速度
                self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_stable_reached")
                # 2) 统一输出 /rl_local_goal_status = goal_reached
                self.safe_publish(
                    self.pub_goal_status,
                    StringMsg(data="goal_reached"),
                    "goal_status_stable_reached",
                )
                # 3) full log：打印 + 写 meta，补充距离、速度、连续保持采样点数
                rospy.loginfo(
                    "[RL] *** 稳定到达目标 | dist=%.3fm |v|=%.4fm/s |w|=%.4frad/s "
                    "stable=%d/%d  tol=%.3fm  v_eps=%.4f w_eps=%.4f "
                    "lin3=%s ang3=%s goal4=[%+.4f,%+.4f,%+.4f,%+.4f] status=%s ***",
                    arrival_info["dist_now"],
                    arrival_info["lin_speed"],
                    arrival_info["ang_speed"],
                    arrival_info["stable_count"],
                    arrival_info["stable_stop_min_count"],
                    arrival_info["goal_tolerance"],
                    arrival_info["stable_stop_lin_eps"],
                    arrival_info["stable_stop_ang_eps"],
                    np.array2string(
                        np.asarray(arrival_info["lin3"], dtype=np.float32),
                        precision=4, separator=','
                    ),
                    np.array2string(
                        np.asarray(arrival_info["ang3"], dtype=np.float32),
                        precision=4, separator=','
                    ),
                    arrival_info["goal4"][0],
                    arrival_info["goal4"][1],
                    arrival_info["goal4"][2],
                    arrival_info["goal4"][3],
                    arrival_info["latest_goal_status"],
                )
                self._write_meta({
                    "type": "goal_status_event",
                    "event": "goal_reached",
                    "t_event": round(rospy.Time.now().to_sec(), 6),
                    "obs_seq": self.obs_seq,
                    "dist_now": round(arrival_info["dist_now"], 6),
                    "lin_speed": round(arrival_info["lin_speed"], 6),
                    "ang_speed": round(arrival_info["ang_speed"], 6),
                    "stable_count": arrival_info["stable_count"],
                    "stable_stop_min_count": arrival_info["stable_stop_min_count"],
                    "stable_stop_lin_eps": arrival_info["stable_stop_lin_eps"],
                    "stable_stop_ang_eps": arrival_info["stable_stop_ang_eps"],
                    "goal_tolerance": arrival_info["goal_tolerance"],
                    "goal_valid": bool(arrival_info["goal_valid"]),
                    "latest_goal_status": arrival_info["latest_goal_status"],
                    "lin3": [round(float(x), 6) for x in arrival_info["lin3"]],
                    "ang3": [round(float(x), 6) for x in arrival_info["ang3"]],
                    "goal4": [round(float(x), 6) for x in arrival_info["goal4"]],
                })
                continue

            self.obs_seq += 1
            seq = self.obs_seq
            self._last_wait_state = ()

            # ── 首帧快照（只写一次）──────────────────────────
            if not self._snapshot_done:
                self._snapshot_done = True
                self._write_startup_snapshot(
                    t_speed      = t_speed_snap,
                    t_goal       = t_goal_snap,
                    t_lidar      = t_ref,
                    speed_age_ms = speed_age_ms,
                    goal_age_ms  = goal_age_ms,
                    raw_lin3     = np.array(speed6_snap[:3], dtype=np.float32),
                    raw_ang3     = np.array(speed6_snap[3:], dtype=np.float32),
                    raw_goal4    = np.array(goal4_snap,      dtype=np.float32),
                    raw_lidar543 = obs_1d[S_LIDAR:E_LIDAR].copy(),
                    obs_1d       = obs_1d.copy(),
                )

            def _sec(t):
                return round(t.to_sec(), 6) if t is not None else None

            self._write_meta({
                "type": "obs_meta",
                "obs_seq": seq,
                "t_ref": _sec(t_ref),
                "t_speed": _sec(t_speed_snap),
                "t_goal": _sec(t_goal_snap),
                "t_lidar": _sec(t_ref),
                "age_speed_ms": -1.0 if speed_age_ms is None else round(speed_age_ms, 3),
                "age_goal_ms": -1.0 if goal_age_ms is None else round(goal_age_ms, 3),
                "speed6": speed6_snap,
                "goal4": goal4_snap,
                "goal_status": self.latest_goal_status,
            })

            ok, fatal_msgs, warn_msgs = self._validate_and_correct(obs_1d)
            for m in warn_msgs:
                rospy.logwarn_throttle(1.0, "[RL][VAL] %s", m)
            if not ok:
                for m in fatal_msgs:
                    rospy.logwarn_throttle(1.0, "[RL][VAL][FATAL] %s", m)
                continue

            if not goal_valid:
                if self.publish_obs and self.publish_obs_when_goal_invalid:
                    self.safe_publish(self.pub_obs, self._to_msg(obs_1d), "obs_invalid_goal")
                if self.zero_cmd_when_goal_invalid:
                    self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_goal_invalid")
                continue

            if self.publish_obs:
                self.safe_publish(self.pub_obs, self._to_msg(obs_1d), "obs_556")
            if self.debug_publish_inputs:
                self.safe_publish(self.pub_debug_goal_used, self._to_msg(goal_copy), "debug_goal_used")

            if self.debug_obs_log:
                s = LIDAR_FRAME_DIM
                rospy.loginfo(
                    "[RL][OBS] lin=%s ang=%s gravity=%s goal=%s lidar0[:5]=%s lidar1[:5]=%s lidar2[:5]=%s",
                    np.array2string(obs_1d[0:3], precision=4, separator=','),
                    np.array2string(obs_1d[3:6], precision=4, separator=','),
                    np.array2string(obs_1d[6:9], precision=4, separator=','),
                    np.array2string(obs_1d[9:13], precision=4, separator=','),
                    np.array2string(obs_1d[13:18], precision=4, separator=','),
                    np.array2string(obs_1d[13 + s:18 + s], precision=4, separator=','),
                    np.array2string(obs_1d[13 + 2 * s:18 + 2 * s], precision=4, separator=','),
                )

            try:
                outputs = self.session.run([self.output_name], {self.input_name: obs_2d})
                raw_action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] 推理失败: %s", e)
                continue

            try:
                raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w = self._map_action(raw_action)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] 动作映射失败: %s", e)
                continue

            cmd = Twist()
            cmd.linear.x = cmd_v
            cmd.angular.z = cmd_w
            self.safe_publish(self.pub_cmd, cmd, "cmd_vel")
            self._append_action_log(raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w)

            self._write_meta({
                "type": "action_meta",
                "obs_seq": seq,
                "t_action": round(rospy.Time.now().to_sec(), 6),
                "raw_v": round(raw_v, 6),
                "raw_w": round(raw_w, 6),
                "clip_v": round(clip_v, 6),
                "clip_w": round(clip_w, 6),
                "cmd_v": round(cmd_v, 6),
                "cmd_w": round(cmd_w, 6),
            })

            # ── 首次成功推理+控制 → 发一次 goal_tracking ────────────────
            # 本处是 _inference_loop 中“数据齐全 + goal_valid + 成功构造 obs + 成功跑出动作
            # + 已向 /rl_cmd_vel 正常发布”的第一次成功落点。每个 goal 只发一次，否则
            # 自己的 goal_tracking 回环会在 cb_goal_status 里触发 _stable_stop_count 的
            # 重复清零（尽管我们已改成边沿触发，仍用本标志提供双重保险）。
            emit_tracking = False
            with self._data_lock:
                if not self.goal_tracking_published:
                    self.goal_tracking_published = True
                    emit_tracking = True

            if emit_tracking:
                self.safe_publish(
                    self.pub_goal_status,
                    StringMsg(data="goal_tracking"),
                    "goal_status_tracking",
                )
                rospy.loginfo(
                    "[RL] *** goal_tracking | 首帧控制 obs_seq=%d cmd_v=%+.4f cmd_w=%+.4f "
                    "dist=%.3fm goal4=[%+.4f,%+.4f,%+.4f,%+.4f] ***",
                    seq, cmd_v, cmd_w, dist_now, dx, dy, dz, dyaw,
                )
                self._write_meta({
                    "type": "goal_status_event",
                    "event": "goal_tracking",
                    "t_event": round(rospy.Time.now().to_sec(), 6),
                    "obs_seq": seq,
                    "cmd_v": round(cmd_v, 6),
                    "cmd_w": round(cmd_w, 6),
                    "dist_now": round(dist_now, 6),
                    "goal4": [round(dx, 6), round(dy, 6), round(dz, 6), round(dyaw, 6)],
                })

            if self.debug_log:
                rospy.loginfo_throttle(
                    0.5,
                    "[RL] goal dx=%+.3f dy=%+.3f dz=%+.3f dyaw=%+.4f dist=%.3fm | status=%s | raw[%+.4f,%+.4f] clip[%+.4f,%+.4f] | cmd v=%+.4f w=%+.4f",
                    dx, dy, dz, dyaw, dist_now, self.latest_goal_status,
                    raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w
                )

    def _on_shutdown(self):
        rospy.loginfo("[RL] shutting down — publishing zero cmd_vel...")
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass
        try:
            if self._meta_fp is not None:
                self._meta_fp.close()
        except Exception:
            pass

    def spin(self):
        rospy.loginfo("[RL] fused node running. waiting callbacks...")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RLOnnxInferenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
