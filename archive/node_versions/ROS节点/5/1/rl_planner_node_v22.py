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
        
        # 修改1：统一到达阈值为 0.50m（与目标节点一致）
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.50))
        self._goal_active = False

        self.zero_goal_on_status_clear = bool(rospy.get_param("~zero_goal_on_status_clear", True))
        self.zero_cmd_on_goal_cleared = bool(rospy.get_param("~zero_cmd_on_goal_cleared", True))
        self.zero_cmd_on_goal_reached = bool(rospy.get_param("~zero_cmd_on_goal_reached", True))
        self.publish_zero_when_goal_reached = bool(rospy.get_param("~publish_zero_when_goal_reached", True))
        self.publish_obs_when_goal_invalid = bool(rospy.get_param("~publish_obs_when_goal_invalid", False))
        self.zero_cmd_when_goal_invalid = bool(rospy.get_param("~zero_cmd_when_goal_invalid", True))

        # ============================================================
        # 动作映射链 (2026-04-21 patched, 增加目标角门控):
        #   raw -> 减 bias -> clip[-1,1] -> 线性映射 -> 二次 clip -> 低通滤波 -> 目标角门控
        # ------------------------------------------------------------
        # 工序 1 (减 bias):
        #   修改2：默认关闭静态偏置补偿，只有在有明确实验依据时才启用
        # 工序 2 (clip):
        #   use_clip=True 必开。模型在饱和 OOD 下 raw 经常 ±5,
        #   不 clip 会让 cmd 越过物理限位。
        # 工序 3 (映射): 原线性映射不变。
        # 工序 4 (二次 clip 保险): 防数值意外。
        # 工序 5 (低通滤波):
        #   只对 cmd_w 滤,cmd_v 不滤(线速度本来就稳定)
        #   alpha=1.0 关闭, 0.3 中等(推荐起点), 0.5 弱
        # 工序 6 (目标角门控):
        #   大角度限速，超大角度原地转向
        # ============================================================
        self.use_clip          = bool (rospy.get_param("~use_action_clip",   True))
        self.action_clip_low   = float(rospy.get_param("~action_clip_low",  -1.0))
        self.action_clip_hi    = float(rospy.get_param("~action_clip_hi",    1.0))
        self.linear_min        = float(rospy.get_param("~linear_min",       -0.2))
        self.linear_max        = float(rospy.get_param("~linear_max",        0.5))
        self.angular_min       = float(rospy.get_param("~angular_min",      -0.5))
        self.angular_max       = float(rospy.get_param("~angular_max",       0.5))
        
        # 修改2：取消默认角速度静态偏置
        self.action_bias_w     = float(rospy.get_param("~action_bias_w",    0.0))
        
        # 工序 5: cmd_w 一阶低通滤波系数
        self.cmd_w_filter_alpha = float(rospy.get_param("~cmd_w_filter_alpha", 0.3))
        self._cmd_w_prev       = 0.0  # 滤波器历史
        
        # 修改4：增加目标角门控参数
        self.use_goal_angle_gate = bool(rospy.get_param("~use_goal_angle_gate", True))
        self.goal_angle_slowdown_thresh = float(rospy.get_param("~goal_angle_slowdown_thresh", 0.5))
        self.goal_angle_turninplace_thresh = float(rospy.get_param("~goal_angle_turninplace_thresh", 0.8))
        self.goal_angle_slowdown_v_max = float(rospy.get_param("~goal_angle_slowdown_v_max", 0.10))

        # ===== 版本22新增：近目标收束控制参数 =====
        self.enable_near_goal_controller = bool(rospy.get_param("~enable_near_goal_controller", True))

        self.near_goal_dist_1 = float(rospy.get_param("~near_goal_dist_1", 1.0))
        self.near_goal_dist_2 = float(rospy.get_param("~near_goal_dist_2", 0.7))
        self.near_goal_dist_3 = float(rospy.get_param("~near_goal_dist_3", 0.6))

        self.near_goal_v_cap_1 = float(rospy.get_param("~near_goal_v_cap_1", 0.20))
        self.near_goal_v_cap_2 = float(rospy.get_param("~near_goal_v_cap_2", 0.10))

        self.near_goal_w_scale_1 = float(rospy.get_param("~near_goal_w_scale_1", 0.5))
        self.near_goal_w_scale_2 = float(rospy.get_param("~near_goal_w_scale_2", 0.3))

        self.near_goal_align_yaw_thresh = float(rospy.get_param("~near_goal_align_yaw_thresh", 0.25))
        self.near_goal_align_k = float(rospy.get_param("~near_goal_align_k", 1.2))
        self.near_goal_align_w_cap = float(rospy.get_param("~near_goal_align_w_cap", 0.20))

        # 禁止近目标区出现"大线速度 + 大角速度"组合
        self.near_goal_no_arc_dist = float(rospy.get_param("~near_goal_no_arc_dist", 1.0))
        self.near_goal_no_arc_w_thresh = float(rospy.get_param("~near_goal_no_arc_w_thresh", 0.25))
        self.near_goal_no_arc_v_cap = float(rospy.get_param("~near_goal_no_arc_v_cap", 0.12))

        # 绕圈保护器
        self.enable_orbit_guard = bool(rospy.get_param("~enable_orbit_guard", True))
        self.orbit_guard_dist = float(rospy.get_param("~orbit_guard_dist", 1.2))
        self.orbit_guard_window = int(rospy.get_param("~orbit_guard_window", 12))
        self.orbit_guard_same_sign_ratio = float(rospy.get_param("~orbit_guard_same_sign_ratio", 0.9))
        self.orbit_guard_abs_w_thresh = float(rospy.get_param("~orbit_guard_abs_w_thresh", 0.35))
        self.orbit_guard_progress_thresh = float(rospy.get_param("~orbit_guard_progress_thresh", 0.05))

        self._recent_cmd_w = []
        self._recent_dist = []

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

        self._lock_speed = threading.Lock()
        self._lock_goal = threading.Lock()
        self._lock_lidar = threading.Lock()

        self.latest_goal_status = "unknown"
        self._goal_status_lock = threading.Lock()

        self.obs_seq = 0
        self._action_log_fp = None
        self._meta_fp = None

        self._snapshot_done = False
        self._snapshot_path = None

        rospy.loginfo("[RL] ════════════════════════════════════════════════════════════")
        rospy.loginfo("[RL] RL ONNX 推理节点 v22 (近目标收束 + 绕圈保护)")
        rospy.loginfo("[RL] ════════════════════════════════════════════════════════════")
        rospy.loginfo("[RL] model_path             = %s", self.model_path)
        rospy.loginfo("[RL] input_name             = %s", self.input_name)
        rospy.loginfo("[RL] output_name            = %s", self.output_name)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] speed_topic            = %s", self.speed_topic)
        rospy.loginfo("[RL] goal_topic             = %s", self.goal_topic)
        rospy.loginfo("[RL] goal_status_topic      = %s", self.goal_status_topic)
        rospy.loginfo("[RL] lidar_topic            = %s", self.lidar_topic)
        rospy.loginfo("[RL] out_cmd_topic          = %s", self.out_cmd_topic)
        rospy.loginfo("[RL] obs_topic              = %s", self.obs_topic)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] goal_tolerance         = %.3f m", self.goal_tolerance)
        rospy.loginfo("[RL] use_split_goal_gate    = %s", self.use_split_goal_gate)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] 动作映射链参数:")
        rospy.loginfo("[RL]   use_action_clip      = %s", self.use_clip)
        rospy.loginfo("[RL]   action_clip_low      = %.2f", self.action_clip_low)
        rospy.loginfo("[RL]   action_clip_hi       = %.2f", self.action_clip_hi)
        rospy.loginfo("[RL]   linear_min           = %.2f", self.linear_min)
        rospy.loginfo("[RL]   linear_max           = %.2f", self.linear_max)
        rospy.loginfo("[RL]   angular_min          = %.2f", self.angular_min)
        rospy.loginfo("[RL]   angular_max          = %.2f", self.angular_max)
        rospy.loginfo("[RL]   action_bias_w        = %.4f", self.action_bias_w)
        rospy.loginfo("[RL]   cmd_w_filter_alpha   = %.2f", self.cmd_w_filter_alpha)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] 目标角门控参数:")
        rospy.loginfo("[RL]   use_goal_angle_gate       = %s", self.use_goal_angle_gate)
        rospy.loginfo("[RL]   goal_angle_slowdown_thresh  = %.2f rad", self.goal_angle_slowdown_thresh)
        rospy.loginfo("[RL]   goal_angle_turninplace_thresh = %.2f rad", self.goal_angle_turninplace_thresh)
        rospy.loginfo("[RL]   goal_angle_slowdown_v_max   = %.2f m/s", self.goal_angle_slowdown_v_max)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] v22 近目标收束控制参数:")
        rospy.loginfo("[RL]   enable_near_goal_controller = %s", self.enable_near_goal_controller)
        rospy.loginfo("[RL]   near_goal_dist_1            = %.2f m", self.near_goal_dist_1)
        rospy.loginfo("[RL]   near_goal_dist_2            = %.2f m", self.near_goal_dist_2)
        rospy.loginfo("[RL]   near_goal_dist_3            = %.2f m", self.near_goal_dist_3)
        rospy.loginfo("[RL]   near_goal_v_cap_1           = %.2f m/s", self.near_goal_v_cap_1)
        rospy.loginfo("[RL]   near_goal_v_cap_2           = %.2f m/s", self.near_goal_v_cap_2)
        rospy.loginfo("[RL]   near_goal_w_scale_1         = %.2f", self.near_goal_w_scale_1)
        rospy.loginfo("[RL]   near_goal_w_scale_2         = %.2f", self.near_goal_w_scale_2)
        rospy.loginfo("[RL]   near_goal_align_yaw_thresh  = %.2f rad", self.near_goal_align_yaw_thresh)
        rospy.loginfo("[RL]   near_goal_align_k           = %.2f", self.near_goal_align_k)
        rospy.loginfo("[RL]   near_goal_align_w_cap       = %.2f rad/s", self.near_goal_align_w_cap)
        rospy.loginfo("[RL]   near_goal_no_arc_dist       = %.2f m", self.near_goal_no_arc_dist)
        rospy.loginfo("[RL]   near_goal_no_arc_w_thresh   = %.2f rad/s", self.near_goal_no_arc_w_thresh)
        rospy.loginfo("[RL]   near_goal_no_arc_v_cap      = %.2f m/s", self.near_goal_no_arc_v_cap)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] v22 绕圈保护器参数:")
        rospy.loginfo("[RL]   enable_orbit_guard          = %s", self.enable_orbit_guard)
        rospy.loginfo("[RL]   orbit_guard_dist            = %.2f m", self.orbit_guard_dist)
        rospy.loginfo("[RL]   orbit_guard_window          = %d 帧", self.orbit_guard_window)
        rospy.loginfo("[RL]   orbit_guard_same_sign_ratio = %.2f", self.orbit_guard_same_sign_ratio)
        rospy.loginfo("[RL]   orbit_guard_abs_w_thresh    = %.2f rad/s", self.orbit_guard_abs_w_thresh)
        rospy.loginfo("[RL]   orbit_guard_progress_thresh = %.2f m", self.orbit_guard_progress_thresh)
        rospy.loginfo("[RL] ──────────────────────────────────────────────────────────────")
        rospy.loginfo("[RL] action_log_path        = %s", self.action_log_path)
        rospy.loginfo("[RL] meta_log_path          = %s", self.meta_log_path)
        rospy.loginfo("[RL] ════════════════════════════════════════════════════════════")

        try:
            self.session = ort.InferenceSession(
                self.model_path, providers=["CPUExecutionProvider"]
            )
            rospy.loginfo("[RL] ONNX 模型加载成功")
        except Exception as e:
            rospy.logerr("[RL] ONNX 模型加载失败: %s", e)
            raise

        self._open_action_log()
        self._open_meta_log()
        self._prepare_snapshot_path()

        self.pub_cmd = rospy.Publisher(self.out_cmd_topic, Twist, queue_size=1)
        self.pub_obs = rospy.Publisher(self.obs_topic, Float32MultiArray, queue_size=1)

        self.pub_debug_lin_vel = rospy.Publisher(self.debug_lin_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_ang_vel = rospy.Publisher(self.debug_ang_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_goal_recv = rospy.Publisher(self.debug_goal_recv_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_goal_used = rospy.Publisher(self.debug_goal_used_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_goal_status = rospy.Publisher(self.debug_goal_status_echo_topic, StringMsg, queue_size=1)
        self.pub_debug_obs0 = rospy.Publisher(self.debug_obs0_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs1 = rospy.Publisher(self.debug_obs1_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs2 = rospy.Publisher(self.debug_obs2_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_speed_age = rospy.Publisher(self.debug_speed_age_topic, Float32, queue_size=1)
        self.pub_debug_goal_age = rospy.Publisher(self.debug_goal_age_topic, Float32, queue_size=1)
        self.pub_debug_age_pair = rospy.Publisher(self.debug_age_pair_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs_meta = rospy.Publisher(self.debug_obs_meta_topic, StringMsg, queue_size=1)

        rospy.Subscriber(self.speed_topic, Float32MultiArray, self._speed_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, Float32MultiArray, self._goal_cb, queue_size=1)
        rospy.Subscriber(self.goal_status_topic, StringMsg, self._goal_status_cb, queue_size=1)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self._lidar_cb, queue_size=1)

        self.inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.inference_thread.start()

        rospy.on_shutdown(self._on_shutdown)
        self.initialized = True
        rospy.loginfo("[RL] 节点初始化完成")

    # ===== 工具函数 =====
    def _open_action_log(self):
        try:
            self._action_log_fp = open(self.action_log_path, "w")
            self._action_log_fp.write("raw_v,raw_w,clip_v,clip_w,cmd_v,cmd_w\n")
            rospy.loginfo("[RL] action log 已打开: %s", self.action_log_path)
        except Exception as e:
            rospy.logwarn("[RL] 打开 action log 失败: %s", e)

    def _open_meta_log(self):
        try:
            self._meta_fp = open(self.meta_log_path, "w")
            rospy.loginfo("[RL] meta log 已打开: %s", self.meta_log_path)
        except Exception as e:
            rospy.logwarn("[RL] 打开 meta log 失败: %s", e)

    def _prepare_snapshot_path(self):
        """
        生成一个带时间戳的启动快照路径，每次启动节点时均写入新文件。
        格式：rl_planner_node4_snapshot_YYMMDD_HHMMSS.txt
        存储于 /home/ros
        """
        now = datetime.datetime.now()
        fname = "rl_planner_node4_snapshot_{}.txt".format(now.strftime("%y%m%d_%H%M%S"))
        self._snapshot_path = os.path.join("/home/ros", fname)
        rospy.loginfo("[RL] 启动快照路径: %s", self._snapshot_path)

    def _append_action_log(self, raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w):
        if self._action_log_fp is not None:
            try:
                self._action_log_fp.write(
                    "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(
                        raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w
                    )
                )
                self._action_log_fp.flush()
            except Exception:
                pass

    def _write_meta(self, meta_dict):
        if self._meta_fp is not None:
            try:
                self._meta_fp.write(json.dumps(meta_dict, ensure_ascii=False) + "\n")
                self._meta_fp.flush()
            except Exception:
                pass

    @staticmethod
    def safe_publish(pub, msg, label=""):
        try:
            pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[RL][PUB] 发布失败 %s: %s", label, e)

    @staticmethod
    def _to_msg(arr_1d):
        arr_1d = np.asarray(arr_1d, dtype=np.float32).reshape(-1)
        msg = Float32MultiArray()
        msg.data = arr_1d.tolist()
        return msg

    # ===== 回调函数 =====
    def _speed_cb(self, msg):
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.shape[0] < 6:
            rospy.logwarn_throttle(1.0, "[RL] speed_cb: 维度不足 %d", arr.shape[0])
            return
        with self._lock_speed:
            self._src_lin3[:] = arr[:3]
            self._src_ang3[:] = arr[3:6]
            self.t_speed = rospy.Time.now()
            self.has_speed = True
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_lin_vel, self._to_msg(self._src_lin3), "debug_lin")
            self.safe_publish(self.pub_debug_ang_vel, self._to_msg(self._src_ang3), "debug_ang")

    def _goal_cb(self, msg):
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.shape[0] < 4:
            rospy.logwarn_throttle(1.0, "[RL] goal_cb: 维度不足 %d", arr.shape[0])
            return
        with self._lock_goal:
            self._src_goal4[:] = arr[:4]
            self.t_goal = rospy.Time.now()
            self.has_goal = True
            self._goal_active = True
            self.goal_done_logged = False
            self.goal_reached = False
            # 修改5：新目标到来时复位滤波器状态
            self._reset_action_filter_state("new_goal_received")
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_goal_recv, msg, "debug_goal_recv")

    def _goal_status_cb(self, msg):
        status_str = str(msg.data).strip().lower()
        with self._goal_status_lock:
            old_status = self.latest_goal_status
            self.latest_goal_status = status_str
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_goal_status, msg, "debug_goal_status")
        if status_str in ("goal_reached", "goal_cleared", "shutdown"):
            with self._lock_goal:
                if self.zero_goal_on_status_clear:
                    self._src_goal4[:] = 0.0
                self._goal_active = False
                self.goal_reached = (status_str == "goal_reached")
                self.goal_done_logged = False
            # 修改5：状态清零时复位滤波器状态
            self._reset_action_filter_state("goal_status_" + status_str)
            if self.zero_cmd_on_goal_cleared or self.zero_cmd_on_goal_reached:
                self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_status_" + status_str)
            if old_status != status_str:
                rospy.loginfo("[RL] 目标状态 -> %s", status_str)

    def _lidar_cb(self, msg):
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.shape[0] < LIDAR_TOTAL:
            rospy.logwarn_throttle(1.0, "[RL] lidar_cb: 维度不足 %d", arr.shape[0])
            return
        with self._lock_lidar:
            self._src_lidar[:] = arr[:LIDAR_TOTAL]
            self.t_lidar = rospy.Time.now()
            self.has_lidar = True
        if self.debug_publish_inputs:
            s = LIDAR_FRAME_DIM
            self.safe_publish(self.pub_debug_obs0, self._to_msg(self._src_lidar[0:s]), "debug_obs0")
            self.safe_publish(self.pub_debug_obs1, self._to_msg(self._src_lidar[s : 2 * s]), "debug_obs1")
            self.safe_publish(self.pub_debug_obs2, self._to_msg(self._src_lidar[2 * s : 3 * s]), "debug_obs2")

    # ===== 观测值构建 =====
    def _build_obs_556_locked(self):
        self._obs_buf[S_LIN3:E_LIN3] = self._src_lin3
        self._obs_buf[S_ANG3:E_ANG3] = self._src_ang3
        self._obs_buf[S_GRAV:E_GRAV] = self._src_gravity3
        self._obs_buf[S_GOAL:E_GOAL] = self._src_goal4
        self._obs_buf[S_LIDAR:E_LIDAR] = self._src_lidar
        return self._obs_buf.reshape(1, -1)

    def _validate_and_correct(self, obs_1d):
        fatal = []
        warn = []

        seg = obs_1d[0:3]
        if float(seg.max()) > self.val_lin_max:
            warn.append("lin[0:3] 超范围")

        seg = obs_1d[3:6]
        if float(seg.max()) > self.val_ang_max:
            warn.append("ang[3:6] 超范围")

        seg = obs_1d[6:9]
        norm_g = float(np.linalg.norm(seg))
        if abs(norm_g - 1.0) > 0.2:
            warn.append("gravity norm=%.3f != 1" % norm_g)

        seg = obs_1d[9:13]
        dx_dy_dz = seg[:3]
        dist_2d = float(np.linalg.norm(dx_dy_dz[:2]))
        if dist_2d > self.val_goal_dist_max:
            warn.append("goal dist_2d=%.2fm 超限" % dist_2d)

        seg = obs_1d[13:556]
        if float(seg.min()) < 0.0:
            warn.append("lidar[13:556] 出现负值")
        if float(seg.max()) > self.lidar_max_range * 1.5:
            warn.append("lidar[13:556] 超量程异常")

        if obs_1d.shape[0] != OBS_DIM:
            fatal.append("obs 总维度=%d != %d" % (obs_1d.shape[0], OBS_DIM))

        return len(fatal) == 0, fatal, warn

    # 修改5：增加统一的滤波器状态复位函数
    def _reset_action_filter_state(self, reason="unknown"):
        """
        复位动作滤波器状态，防止历史角速度滤波值影响新目标。
        在以下时刻调用：
        1. 收到新目标
        2. 收到 goal_reached / goal_cleared / shutdown 状态
        3. 本地距离判定触发到达
        4. goal_valid == False 且准备发布零速度
        5. 绕圈保护器触发时
        """
        self._cmd_w_prev = 0.0
        rospy.loginfo_throttle(2.0, "[RL] reset action filter state: %s", reason)

    # ===== 版本22新增：近目标收束辅助函数 =====
    def _push_control_history(self, dist_now, cmd_w):
        """记录最近的距离和角速度，用于绕圈检测"""
        self._recent_dist.append(float(dist_now))
        self._recent_cmd_w.append(float(cmd_w))

        max_len = max(3, int(self.orbit_guard_window))
        if len(self._recent_dist) > max_len:
            self._recent_dist = self._recent_dist[-max_len:]
        if len(self._recent_cmd_w) > max_len:
            self._recent_cmd_w = self._recent_cmd_w[-max_len:]

    def _orbit_guard_triggered(self):
        """
        检测是否触发绕圈保护条件：
        - 长期单侧转向（same_sign_ratio > 阈值）
        - 角速度较大（abs_w_mean > 阈值）
        - 距离几乎不下降（progress < 阈值）
        """
        if not self.enable_orbit_guard:
            return False
        if len(self._recent_cmd_w) < self.orbit_guard_window:
            return False
        if len(self._recent_dist) < self.orbit_guard_window:
            return False

        w = np.asarray(self._recent_cmd_w[-self.orbit_guard_window:], dtype=np.float32)
        d = np.asarray(self._recent_dist[-self.orbit_guard_window:], dtype=np.float32)

        abs_w_mean = float(np.mean(np.abs(w)))
        if abs_w_mean < self.orbit_guard_abs_w_thresh:
            return False

        same_sign_ratio = max(
            float(np.mean(w > 0.0)),
            float(np.mean(w < 0.0))
        )
        if same_sign_ratio < self.orbit_guard_same_sign_ratio:
            return False

        # 距离下降太少，说明在绕
        progress = float(d[0] - d[-1])
        if progress > self.orbit_guard_progress_thresh:
            return False

        return True

    def _map_action(self, raw_action):
        """
        2026-04-21 patched: S 形抑制组合方案,5 道工序。
            raw -> [1]减bias -> [2]clip[-1,1] -> [3]线性映射 -> [4]二次clip -> [5]低通滤波

        每道工序都可通过 ROS param 关闭(分别对应 action_bias_w=0,
        use_clip=False, angular_max=1.0, cmd_w_filter_alpha=1.0)。

        返回 (raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w):
          raw_v, raw_w : 模型原始输出(未做任何处理)
          clip_v, clip_w : 减 bias + clip 后的值(下游日志使用)
          cmd_v, cmd_w : 最终下发到 /rl_cmd_vel 的物理量
        """
        a = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if a.shape[0] < 2:
            raise ValueError("action 维度=%d < 2" % a.shape[0])
        raw_v, raw_w = float(a[0]), float(a[1])

        # ----- 工序 1: 减网络 bias (只对 raw_w) -----
        # 默认关闭静态偏置补偿（action_bias_w=0.0）
        # 只有在有明确实验依据时才允许重新启用
        raw_w_debiased = raw_w - self.action_bias_w

        # ----- 工序 2: clip 到 [action_clip_low, action_clip_hi] (默认 [-1, 1]) -----
        # 训练时 PPO wrapper 通常会 clip,部署侧必须一致,否则 raw 飙到 ±5
        # 会让 cmd 越过物理限位,陷入 actuator saturation 反馈环。
        if self.use_clip:
            clip_v = float(np.clip(raw_v,          self.action_clip_low, self.action_clip_hi))
            clip_w = float(np.clip(raw_w_debiased, self.action_clip_low, self.action_clip_hi))
        else:
            clip_v, clip_w = raw_v, raw_w_debiased

        # ----- 工序 3: 线性映射到物理范围 -----
        # [action_clip_low, action_clip_hi] -> [linear/angular_min, max]
        span = self.action_clip_hi - self.action_clip_low
        cmd_v = (clip_v - self.action_clip_low) / span * (self.linear_max  - self.linear_min)  + self.linear_min
        cmd_w = (clip_w - self.action_clip_low) / span * (self.angular_max - self.angular_min) + self.angular_min

        # ----- 工序 4: 底盘侧二次 clip 保险 -----
        # 防止上述映射因数值意外越界,同时显式表明"绝不超过 linear/angular_min/max"。
        cmd_v = float(np.clip(cmd_v, self.linear_min,  self.linear_max))
        cmd_w = float(np.clip(cmd_w, self.angular_min, self.angular_max))

        # ----- 工序 5: cmd_w 一阶低通滤波 (压 S 形振荡) -----
        # 仅对角速度滤,不对线速度滤(线速度本来变化就缓慢)
        # cmd_w_filter_alpha = 1.0 时此步骤等价于不滤波
        cmd_w = (self.cmd_w_filter_alpha * cmd_w
                 + (1.0 - self.cmd_w_filter_alpha) * self._cmd_w_prev)
        self._cmd_w_prev = cmd_w

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
        L.append("  [B4] obs[9:13]  目标向量 goal   来源: {}".format(self.goal_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_goal)))
        L.append("       目标差(t_lidar - t_goal): {}".format(fmt_age(goal_age_ms)))
        L.append("       obs[ 9]  dx   = {:.6f}".format(float(obs_1d[9])))
        L.append("       obs[10]  dy   = {:.6f}".format(float(obs_1d[10])))
        L.append("       obs[11]  dz   = {:.6f}".format(float(obs_1d[11])))
        L.append("       obs[12]  dyaw = {:.6f}".format(float(obs_1d[12])))
        L.append("")

        # B5 激光雷达段
        L.append("  [B5] obs[13:556]  激光雷达 lidar543   来源: {}".format(self.lidar_topic))
        L.append("       来源时刻: {}".format(fmt_t(t_lidar)))
        L.append("       共 {} 维 (3帧 × {} 点/帧)，以下展示前20:".format(
            LIDAR_TOTAL, LIDAR_FRAME_DIM))
        for row in lidar_block(obs_1d[S_LIDAR:E_LIDAR], S_LIDAR, "lidar543"):
            L.append(row)
        L.append("")

        # ════════════════════════════════════════════════════════
        L.append("=" * 68)
        L.append("  A 和 B 的对应段数值应完全一致")
        L.append("  age 越小越好，理想情况下 < 10ms")
        L.append("=" * 68)

        # 写入文件
        try:
            with open(self._snapshot_path, "w") as fp:
                fp.write("\n".join(L))
            rospy.loginfo("[RL] 启动快照已写入: %s", self._snapshot_path)
        except Exception as e:
            rospy.logwarn("[RL] 写入启动快照失败: %s", e)

    def _check_goal_valid(self, goal4_copy):
        """
        分段式目标有效性门控。
        如果 use_split_goal_gate=False, 则退化为简单的"全零则无效"检测。
        """
        if not self.use_split_goal_gate:
            abs_sum = float(np.sum(np.abs(goal4_copy)))
            return abs_sum > self.goal_nonzero_eps

        dx, dy, dz, dyaw = goal4_copy
        dist_2d = math.sqrt(dx * dx + dy * dy)

        if not self._goal_active:
            if dist_2d >= self.goal_pos_eps_enter or abs(dyaw) >= self.goal_yaw_eps_enter:
                self._goal_active = True
        else:
            if dist_2d < self.goal_pos_eps_exit and abs(dyaw) < self.goal_yaw_eps_exit:
                self._goal_active = False

        return self._goal_active

    # ===== 推理循环 =====
    def _inference_loop(self):
        rospy.loginfo("[RL] 推理线程启动, 等待传感器数据...")
        while not rospy.is_shutdown():
            state = (self.has_speed, self.has_goal, self.has_lidar)
            if state != self._last_wait_state:
                rospy.loginfo(
                    "[RL] 传感器就绪状态 speed=%s goal=%s lidar=%s",
                    self.has_speed, self.has_goal, self.has_lidar
                )
                self._last_wait_state = state
            if not (self.has_speed and self.has_goal and self.has_lidar):
                rospy.sleep(0.1)
                continue

            with self._lock_lidar:
                t_ref = self.t_lidar
            if t_ref is None:
                continue

            with self._lock_speed:
                t_speed_snap = self.t_speed
            with self._lock_goal:
                t_goal_snap = self.t_goal

            speed_age_ms = None
            goal_age_ms = None
            if t_speed_snap is not None and t_ref is not None:
                speed_age_ms = (t_ref - t_speed_snap).to_sec() * 1000.0
            if t_goal_snap is not None and t_ref is not None:
                goal_age_ms = (t_ref - t_goal_snap).to_sec() * 1000.0

            if speed_age_ms is not None and speed_age_ms > self.warn_speed_age_ms:
                rospy.logwarn_throttle(2.0, "[RL] speed_age=%.1fms 较大", speed_age_ms)
            if goal_age_ms is not None and goal_age_ms > self.warn_goal_age_ms:
                rospy.logwarn_throttle(2.0, "[RL] goal_age=%.1fms 较大", goal_age_ms)

            if self.drop_if_speed_too_old and speed_age_ms is not None:
                if speed_age_ms > self.warn_speed_age_ms:
                    rospy.logwarn_throttle(1.0, "[RL] speed_age 超标，跳过推理")
                    continue
            if self.drop_if_goal_too_old and goal_age_ms is not None:
                if goal_age_ms > self.warn_goal_age_ms:
                    rospy.logwarn_throttle(1.0, "[RL] goal_age 超标，跳过推理")
                    continue

            if self.debug_publish_inputs:
                if speed_age_ms is not None:
                    self.safe_publish(
                        self.pub_debug_speed_age, Float32(data=speed_age_ms), "speed_age_ms"
                    )
                if goal_age_ms is not None:
                    self.safe_publish(
                        self.pub_debug_goal_age, Float32(data=goal_age_ms), "goal_age_ms"
                    )
                if speed_age_ms is not None and goal_age_ms is not None:
                    self.safe_publish(
                        self.pub_debug_age_pair,
                        self._to_msg(np.array([speed_age_ms, goal_age_ms], dtype=np.float32)),
                        "age_pair",
                    )

            with (
                self._lock_speed,
                self._lock_goal,
                self._lock_lidar,
            ):
                goal_copy = self._src_goal4.copy()
                goal_valid = self._check_goal_valid(goal_copy)

                dx = float(self._src_goal4[0])
                dy = float(self._src_goal4[1])
                dz = float(self._src_goal4[2])
                dyaw = float(self._src_goal4[3])
                dist_now = math.sqrt(dx * dx + dy * dy)

                # 修改1：统一本地到达判定，不再受 goal_tracking 状态阻塞
                # 只要 goal_valid 且距离在阈值内，就认为到达
                if goal_valid and (dist_now <= self.goal_tolerance):
                    self.goal_reached = True
                    # 修改5：到达时复位滤波器状态
                    self._reset_action_filter_state("local_reached")
                    if not self.goal_done_logged:
                        if self.publish_zero_when_goal_reached:
                            self.safe_publish(self.pub_cmd, Twist(), "cmd_zero_goal_tol")
                        rospy.loginfo("[RL] *** 已到达目标点 dist=%.3fm，等待下一目标 ***", dist_now)
                        self.goal_done_logged = True
                    continue

                obs_2d = self._build_obs_556_locked()
                obs_1d = obs_2d.reshape(-1)
                speed6_snap = self._src_lin3.tolist() + self._src_ang3.tolist()
                goal4_snap = self._src_goal4.tolist()
                goal_copy = self._src_goal4.copy()

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
                # 修改5：goal_valid == False 时复位滤波器
                self._reset_action_filter_state("goal_invalid")
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

            # ============================================================
            # 版本22核心修改：三层近目标收束控制 + 绕圈保护
            # ============================================================
            angle_gate_label = "none"
            near_goal_label = "none"
            orbit_guard_label = "none"

            abs_dyaw = abs(dyaw)

            # ------------------------------------------------------------
            # A. 原有目标角门控：保留，但只作为"粗门控"
            # ------------------------------------------------------------
            if self.use_goal_angle_gate:
                if abs_dyaw > self.goal_angle_turninplace_thresh:
                    cmd_v = 0.0
                    angle_gate_label = "turn_in_place"
                elif abs_dyaw > self.goal_angle_slowdown_thresh:
                    cmd_v = min(cmd_v, self.goal_angle_slowdown_v_max)
                    if cmd_v < 0.0:
                        cmd_v = 0.0
                    angle_gate_label = "slowdown"

            # ------------------------------------------------------------
            # B. 近目标收束控制：解决"快到目标点时绕圈"
            # ------------------------------------------------------------
            if self.enable_near_goal_controller:
                # 一级：进入近目标区，先压线速度和角速度
                if dist_now < self.near_goal_dist_1:
                    cmd_v = min(cmd_v, self.near_goal_v_cap_1)
                    cmd_w = self.near_goal_w_scale_1 * cmd_w
                    near_goal_label = "near_goal_stage_1"

                # 二级：更近时继续压角速度
                if dist_now < self.near_goal_dist_2:
                    cmd_v = min(cmd_v, self.near_goal_v_cap_2)
                    cmd_w = self.near_goal_w_scale_2 * cmd_w
                    near_goal_label = "near_goal_stage_2"

                # 三级：非常近但还没对准时，进入精靠模式
                if (dist_now < self.near_goal_dist_3) and (abs_dyaw > self.near_goal_align_yaw_thresh):
                    cmd_v = 0.0
                    cmd_w = float(np.clip(
                        self.near_goal_align_k * dyaw,
                        -self.near_goal_align_w_cap,
                        self.near_goal_align_w_cap
                    ))
                    near_goal_label = "near_goal_align"

                # 禁止近目标区出现"大线速度 + 大角速度"组合
                if (dist_now < self.near_goal_no_arc_dist) and (abs(cmd_w) > self.near_goal_no_arc_w_thresh):
                    cmd_v = min(cmd_v, self.near_goal_no_arc_v_cap)
                    if near_goal_label == "none":
                        near_goal_label = "near_goal_no_arc"

            # ------------------------------------------------------------
            # C. 绕圈保护器：长期单侧转 + 距离几乎不下降 -> 强制切回对准模式
            # ------------------------------------------------------------
            self._push_control_history(dist_now, cmd_w)

            if (dist_now < self.orbit_guard_dist) and self._orbit_guard_triggered():
                cmd_v = 0.0
                cmd_w = float(np.clip(
                    self.near_goal_align_k * dyaw,
                    -self.near_goal_align_w_cap,
                    self.near_goal_align_w_cap
                ))
                orbit_guard_label = "orbit_guard_align"
                self._reset_action_filter_state("orbit_guard_triggered")

            # ------------------------------------------------------------
            # D. 最终保险 clip
            # ------------------------------------------------------------
            cmd_v = float(np.clip(cmd_v, self.linear_min, self.linear_max))
            cmd_w = float(np.clip(cmd_w, self.angular_min, self.angular_max))

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
                "angle_gate": angle_gate_label,
                "near_goal_mode": near_goal_label,
                "orbit_guard": orbit_guard_label,
                "dist_now": round(dist_now, 6),
                "dyaw": round(dyaw, 6),
            })

            if self.debug_log:
                rospy.loginfo_throttle(
                    0.5,
                    "[RL] goal dx=%+.3f dy=%+.3f dz=%+.3f dyaw=%+.4f dist=%.3fm | status=%s | gate=%s | near=%s | orbit=%s | raw[%+.4f,%+.4f] clip[%+.4f,%+.4f] | cmd v=%+.4f w=%+.4f",
                    dx, dy, dz, dyaw, dist_now, self.latest_goal_status,
                    angle_gate_label, near_goal_label, orbit_guard_label,
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
