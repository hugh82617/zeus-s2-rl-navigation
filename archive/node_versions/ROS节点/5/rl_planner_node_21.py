#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import os
import threading

import numpy as np
import onnxruntime as ort
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
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
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
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
        self.use_static_gravity = bool(rospy.get_param("~use_static_gravity", False))
        self.gravity_x = float(rospy.get_param("~gravity_x", 0.0))
        self.gravity_y = float(rospy.get_param("~gravity_y", 0.0))
        self.gravity_z = float(rospy.get_param("~gravity_z", -1.0))

        self.goal_nonzero_eps = float(rospy.get_param("~goal_nonzero_eps", 1e-4))
        self.goal_pos_eps_enter = float(rospy.get_param("~goal_pos_eps_enter", 0.05))
        self.goal_pos_eps_exit = float(rospy.get_param("~goal_pos_eps_exit", 0.02))
        self.goal_yaw_eps_enter = float(rospy.get_param("~goal_yaw_eps_enter", 0.05))
        self.goal_yaw_eps_exit = float(rospy.get_param("~goal_yaw_eps_exit", 0.02))
        self.goal_small_value_eps = float(rospy.get_param("~goal_small_value_eps", 1e-6))
        self.use_split_goal_gate = bool(rospy.get_param("~use_split_goal_gate", True))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.20))
        self._goal_active = False

        self.zero_goal_on_status_clear = bool(rospy.get_param("~zero_goal_on_status_clear", True))
        self.zero_cmd_on_goal_cleared = bool(rospy.get_param("~zero_cmd_on_goal_cleared", True))
        self.zero_cmd_on_goal_reached = bool(rospy.get_param("~zero_cmd_on_goal_reached", True))
        self.publish_zero_when_goal_reached = bool(rospy.get_param("~publish_zero_when_goal_reached", True))
        self.publish_obs_when_goal_invalid = bool(rospy.get_param("~publish_obs_when_goal_invalid", False))
        self.zero_cmd_when_goal_invalid = bool(rospy.get_param("~zero_cmd_when_goal_invalid", True))

        # 采用 20 的动作映射链：先裁 raw action 到 [-1,1]，再映射到物理范围
        self.use_clip = bool(rospy.get_param("~use_clip", True))
        self.action_clip_low = float(rospy.get_param("~action_clip_low", -1.0))
        self.action_clip_hi = float(rospy.get_param("~action_clip_high", 1.0))
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
        self._last_wait_state = None

        self._src_lin3 = np.zeros(3, dtype=np.float32)
        self._src_ang3 = np.zeros(3, dtype=np.float32)
        self._src_goal4 = np.zeros(4, dtype=np.float32)
        self._src_lidar = np.full(LIDAR_TOTAL, self.lidar_max_range, dtype=np.float32)
        self._src_gravity3 = np.array([self.gravity_x, self.gravity_y, self.gravity_z], dtype=np.float32)
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

        # ===== 订阅者：严格采用 16 版本 =====
        rospy.Subscriber(self.speed_topic, Twist, self.cb_speed, queue_size=50)
        rospy.Subscriber(self.goal_topic, Float32MultiArray, self.cb_goal, queue_size=50)
        rospy.Subscriber(self.goal_status_topic, StringMsg, self.cb_goal_status, queue_size=50)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=50)
        if not self.use_static_gravity:
            rospy.Subscriber(self.imu_topic, Imu, self.cb_imu, queue_size=50)
            rospy.loginfo("[RL] IMU mode: subscribing %s for projected_gravity", self.imu_topic)
        else:
            rospy.loginfo("[RL] Static gravity mode: gravity=[%.3f, %.3f, %.3f]",
                          self.gravity_x, self.gravity_y, self.gravity_z)

        rospy.on_shutdown(self._on_shutdown)
        self._infer_thread.start()
        self.initialized = True
        self.obs_seq = 0

        rospy.loginfo("[RL] subscribed: speed=%s goal=%s goal_status=%s lidar=%s imu=%s",
                      self.speed_topic, self.goal_topic, self.goal_status_topic,
                      self.lidar_topic, self.imu_topic if not self.use_static_gravity else "DISABLED")
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
        if self.use_clip:
            clip_v = float(np.clip(raw_v, self.action_clip_low, self.action_clip_hi))
            clip_w = float(np.clip(raw_w, self.action_clip_low, self.action_clip_hi))
        else:
            clip_v, clip_w = raw_v, raw_w
        cmd_v = (clip_v + 1.0) * 0.5 * (self.linear_max - self.linear_min) + self.linear_min
        cmd_w = (clip_w + 1.0) * 0.5 * (self.angular_max - self.angular_min) + self.angular_min
        return raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w

    # ===== 回调 =====
    def cb_imu(self, msg):
        if not self.initialized:
            return
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        grav = np.array([-ax / norm, -ay / norm, -az / norm], dtype=np.float32)
        with self._data_lock:
            self._src_gravity3 = grav

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
            self.goal_done_logged = False
            self.goal_reached = False
            self._last_wait_state = None
            goal_copy = self._src_goal4.copy()
        if self.debug_publish_inputs:
            self.safe_publish(self.pub_debug_goal_recv, self._to_msg(goal_copy), "debug_goal_recv")

    def cb_goal_status(self, msg):
        if not self.initialized:
            return
        status = msg.data.strip()
        self.safe_publish(self.pub_debug_goal_status_echo, StringMsg(data=status), "goal_status_echo")

        zero_goal_copy = None
        publish_zero_cmd = False
        with self._data_lock:
            self.latest_goal_status = status
            if status in ("goal_received", "goal_tracking"):
                self.goal_done_logged = False
                self.goal_reached = False

            if self.zero_goal_on_status_clear and status in ("goal_cleared", "goal_reached", "shutdown"):
                self._src_goal4[:] = 0.0
                self.t_goal = rospy.Time.now()
                self._goal_active = False
                zero_goal_copy = self._src_goal4.copy()
                if status == "goal_reached":
                    self.goal_reached = True

            if status == "goal_reached" and not self.goal_done_logged:
                rospy.loginfo("[RL] 已到达目标点")
                self.goal_done_logged = True

            if ((status == "goal_cleared") and self.zero_cmd_on_goal_cleared) or \
               ((status == "goal_reached") and self.zero_cmd_on_goal_reached):
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

                if (self.latest_goal_status != "goal_tracking") and (dist_now <= self.goal_tolerance) and goal_valid:
                    self.goal_reached = True
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
