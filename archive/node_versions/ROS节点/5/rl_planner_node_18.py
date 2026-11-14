#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import threading

import numpy as np
import rospy
import onnxruntime as ort

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ══════════════════════════════════════════════════════════
#  观测向量布局常量（与训练完全一致，禁止随意修改）
#
#   索引   内容             来源话题              维度
#  [0:3]   线速度 vx/vy/vz  /odom                 3
#  [3:6]   角速度 wx/wy/wz  /odom                 3
#  [6:9]   重力向量          固定=[0,0,-1]         3
#  [9:13]  目标 dx/dy/dz/dyaw /rl_local_goal_body  4
#  [13:556] 激光扫描         /rl/lidar_543        543
#  总计:                                           556
# ══════════════════════════════════════════════════════════
OBS_DIM = 556

S_LIN3  = 0;   E_LIN3  = 3
S_ANG3  = 3;   E_ANG3  = 6
S_GRAV  = 6;   E_GRAV  = 9
S_GOAL  = 9;   E_GOAL  = 13
S_LIDAR = 13;  E_LIDAR = 556

DIM_LIN3  = E_LIN3  - S_LIN3    # 3
DIM_ANG3  = E_ANG3  - S_ANG3    # 3
DIM_GRAV  = E_GRAV  - S_GRAV    # 3
DIM_GOAL  = E_GOAL  - S_GOAL    # 4
DIM_LIDAR = E_LIDAR - S_LIDAR   # 543

GRAVITY_VEC = np.array([0.0, 0.0, -1.0], dtype=np.float32)

_LIN_MAX       = 5.0
_ANG_MAX       = 20.0
_GOAL_DIST_MAX = 100.0


class RLOnnxInferenceNode(object):

    def __init__(self):
        rospy.init_node("rl_onnx_node", anonymous=False)

        # ── 话题参数 ──────────────────────────────────────────
        self.model_path  = rospy.get_param("~model_path",
                           "/home/ros/catkin_ws/src/rl_planner/models/model.onnx")
        self.input_name  = rospy.get_param("~input_name",  "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        self.topic_odom     = rospy.get_param("~topic_odom",     "/odom")
        self.topic_goal4    = rospy.get_param("~topic_goal4",    "/rl_local_goal_body")
        self.topic_lidar543 = rospy.get_param("~topic_lidar543", "/rl/lidar_543")
        self.topic_cmd_vel  = rospy.get_param("~topic_cmd_vel",  "/cmd_vel")

        # ── 运行参数 ──────────────────────────────────────────
        self.warn_goal_age_ms  = float(rospy.get_param("~warn_goal_age_ms",  80.0))
        self.warn_speed_age_ms = float(rospy.get_param("~warn_speed_age_ms", 80.0))
        self.goal_tolerance    = float(rospy.get_param("~goal_tolerance",     0.20))

        self.use_clip        = bool( rospy.get_param("~use_clip",         True))
        self.action_clip_low = float(rospy.get_param("~action_clip_low", -1.0))
        self.action_clip_hi  = float(rospy.get_param("~action_clip_high", 1.0))

        self.linear_min  = float(rospy.get_param("~linear_min",  -0.2))
        self.linear_max  = float(rospy.get_param("~linear_max",   0.5))
        self.angular_min = float(rospy.get_param("~angular_min", -1.0))
        self.angular_max = float(rospy.get_param("~angular_max",  1.0))

        self.pub_zero_on_goal = bool(rospy.get_param("~publish_zero_when_goal_reached", True))

        # ── 自校验阈值（可外部覆盖）──────────────────────────
        self.lidar_max_range   = float(rospy.get_param("~lidar_max_range",   30.0))
        self.val_lin_max       = float(rospy.get_param("~val_lin_max",       _LIN_MAX))
        self.val_ang_max       = float(rospy.get_param("~val_ang_max",       _ANG_MAX))
        self.val_goal_dist_max = float(rospy.get_param("~val_goal_dist_max", _GOAL_DIST_MAX))

        # ── 线程锁 ────────────────────────────────────────────
        self.lock = threading.Lock()

        # ── 各分段源数据（独立数组，与 obs buffer 完全分离）──
        self._src_lin3  = np.zeros(DIM_LIN3,  dtype=np.float32)
        self._src_ang3  = np.zeros(DIM_ANG3,  dtype=np.float32)
        self._src_goal4 = np.zeros(DIM_GOAL,  dtype=np.float32)
        self._src_lidar = np.full(DIM_LIDAR, self.lidar_max_range, dtype=np.float32)

        # ── 预分配 obs buffer（用于每帧组装，不对外暴露引用）─
        self._obs_buf = np.zeros(OBS_DIM, dtype=np.float32)

        # ── 话题就绪标志 ─────────────────────────────────────
        self.has_odom  = False
        self.has_goal  = False
        self.has_lidar = False

        self.t_speed = None
        self.t_goal  = None
        self.t_lidar = None

        self.goal_reached     = False
        self.goal_done_logged = False
        self._last_wait_state = None

        # ── ONNX 会话 ─────────────────────────────────────────
        self.session = ort.InferenceSession(
            self.model_path, providers=["CPUExecutionProvider"])

        self._in_name  = self.input_name
        self._out_name = self.output_name
        try:
            in_names  = [x.name for x in self.session.get_inputs()]
            out_names = [x.name for x in self.session.get_outputs()]
            if self._in_name  not in in_names:
                self._in_name  = in_names[0]
                rospy.logwarn("[RL] input_name 回退到 '%s'", self._in_name)
            if self._out_name not in out_names:
                self._out_name = out_names[0]
                rospy.logwarn("[RL] output_name 回退到 '%s'", self._out_name)
        except Exception as e:
            rospy.logwarn("[RL] ONNX 名称检查失败: %s", e)

        # 模型热身
        _dummy = np.zeros((1, OBS_DIM), dtype=np.float32)
        self.session.run([self._out_name], {self._in_name: _dummy})

        # ── 发布 / 订阅 ───────────────────────────────────────
        self.pub_cmd     = rospy.Publisher(self.topic_cmd_vel, Twist,             queue_size=10)
        self.pub_obs_556 = rospy.Publisher("/rl/obs_556",      Float32MultiArray, queue_size=10)

        rospy.Subscriber(self.topic_odom,     Odometry,          self.cb_odom,  queue_size=50)
        rospy.Subscriber(self.topic_goal4,    Float32MultiArray, self.cb_goal,  queue_size=50)
        rospy.Subscriber(self.topic_lidar543, Float32MultiArray, self.cb_lidar, queue_size=50)

        rospy.loginfo("[RL] ── 启动完成 ──────────────────────────────────")
        rospy.loginfo("[RL] model  : %s", self.model_path)
        rospy.loginfo("[RL] topics : odom=%s  goal=%s  lidar=%s",
                      self.topic_odom, self.topic_goal4, self.topic_lidar543)
        rospy.loginfo("[RL] obs_556: /rl/obs_556  布局 lin3[0:3] ang3[3:6]"
                      " grav[6:9] goal4[9:13] lidar[13:556]")

    # ══════════════════════════════════════════════════════════
    #  工具
    # ══════════════════════════════════════════════════════════

    def _to_msg(self, arr):
        msg = Float32MultiArray()
        msg.data = arr.tolist() if isinstance(arr, np.ndarray) else list(arr)
        return msg

    def safe_publish(self, pub, msg, tag=""):
        if rospy.is_shutdown() or pub is None:
            return
        try:
            pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[RL] publish error [%s]: %s", tag, e)

    def _goal_dist_xy(self):
        dx, dy = float(self._src_goal4[0]), float(self._src_goal4[1])
        return math.sqrt(dx * dx + dy * dy)

    # ══════════════════════════════════════════════════════════
    #  组装 obs_556
    #
    #  设计原则：
    #  1. 每帧全量重写所有 5 段，不依赖任何历史残留值
    #  2. 重力向量每帧强制写入，不信任 buffer 历史状态
    #  3. 写入前对每段源数组做维度硬断言
    #  4. 返回 buf.copy()（独立副本），与内部 buffer 内存隔离，
    #     保证传给 ONNX 的数组不会被其他回调并发修改
    # ══════════════════════════════════════════════════════════

    def _build_obs_556(self):
        buf = self._obs_buf

        # 维度硬断言（布局不匹配时立即抛出，不静默通过）
        assert self._src_lin3.shape  == (DIM_LIN3,),  \
            "lin3 shape %s != (%d,)"  % (self._src_lin3.shape,  DIM_LIN3)
        assert self._src_ang3.shape  == (DIM_ANG3,),  \
            "ang3 shape %s != (%d,)"  % (self._src_ang3.shape,  DIM_ANG3)
        assert self._src_goal4.shape == (DIM_GOAL,),  \
            "goal4 shape %s != (%d,)" % (self._src_goal4.shape, DIM_GOAL)
        assert self._src_lidar.shape == (DIM_LIDAR,), \
            "lidar shape %s != (%d,)" % (self._src_lidar.shape, DIM_LIDAR)

        # 全量写入（顺序与布局定义一一对应）
        buf[S_LIN3 :E_LIN3 ] = self._src_lin3    # [0:3]    线速度
        buf[S_ANG3 :E_ANG3 ] = self._src_ang3    # [3:6]    角速度
        buf[S_GRAV :E_GRAV ] = GRAVITY_VEC       # [6:9]    重力（每帧强写）
        buf[S_GOAL :E_GOAL ] = self._src_goal4   # [9:13]   目标
        buf[S_LIDAR:E_LIDAR] = self._src_lidar   # [13:556] 激光

        # 返回独立副本，与 buf 内存彻底隔离
        return buf.copy().reshape(1, OBS_DIM)

    # ══════════════════════════════════════════════════════════
    #  自校验 + 自修正
    #
    #  FATAL  — 无法安全修正 → 丢弃本帧
    #  WARN   — 已就地修正   → 打印警告后继续推理
    # ══════════════════════════════════════════════════════════

    def _validate_and_correct(self, obs_1d):
        """
        obs_1d: shape (556,) 独立副本，可安全就地修改
        返回 (ok, fatal_list, warn_list)
        """
        fatal = []
        warn  = []

        # [0:3] 线速度
        seg = obs_1d[S_LIN3:E_LIN3]
        if not np.all(np.isfinite(seg)):
            obs_1d[S_LIN3:E_LIN3] = 0.0
            warn.append("lin3[0:3] NaN/Inf → 已置零")
        elif np.any(np.abs(seg) > self.val_lin_max):
            warn.append("lin3[0:3] |v|>%.1fm/s: [%+.3f %+.3f %+.3f]"
                        % (self.val_lin_max, seg[0], seg[1], seg[2]))

        # [3:6] 角速度
        seg = obs_1d[S_ANG3:E_ANG3]
        if not np.all(np.isfinite(seg)):
            obs_1d[S_ANG3:E_ANG3] = 0.0
            warn.append("ang3[3:6] NaN/Inf → 已置零")
        elif np.any(np.abs(seg) > self.val_ang_max):
            warn.append("ang3[3:6] |w|>%.1frad/s: [%+.3f %+.3f %+.3f]"
                        % (self.val_ang_max, seg[0], seg[1], seg[2]))

        # [6:9] 重力常量（防御性核查）
        seg = obs_1d[S_GRAV:E_GRAV]
        if not np.allclose(seg, GRAVITY_VEC, atol=1e-6):
            obs_1d[S_GRAV:E_GRAV] = GRAVITY_VEC
            warn.append("gravity[6:9] 异常 %s → 已强制恢复 [0,0,-1]" % seg.tolist())

        # [9:13] 目标向量（NaN/Inf 无法修正 → FATAL）
        seg = obs_1d[S_GOAL:E_GOAL]
        if not np.all(np.isfinite(seg)):
            fatal.append("goal4[9:13] NaN/Inf=%s，丢弃本帧" % seg.tolist())
        else:
            dist = math.sqrt(float(seg[0])**2 + float(seg[1])**2)
            if dist > self.val_goal_dist_max:
                warn.append("goal4[9:13] dist=%.1fm > %.1fm，可能为陈旧目标"
                            % (dist, self.val_goal_dist_max))

        # [13:556] 激光 543 维
        seg = obs_1d[S_LIDAR:E_LIDAR]
        bad = ~np.isfinite(seg)
        n_bad = int(bad.sum())
        if n_bad > 0:
            seg[bad] = self.lidar_max_range
            obs_1d[S_LIDAR:E_LIDAR] = seg
            warn.append("lidar[13:556] %d 点 NaN/Inf → 已填充 %.1fm"
                        % (n_bad, self.lidar_max_range))
        if float(seg.min()) < 0.0:
            warn.append("lidar[13:556] min=%.4f < 0，数据异常" % float(seg.min()))
        if float(seg.max()) > self.lidar_max_range * 1.5:
            warn.append("lidar[13:556] max=%.1fm 超量程×1.5=%.1fm，可能单位错误"
                        % (float(seg.max()), self.lidar_max_range * 1.5))

        # 总维度最终核查
        if obs_1d.shape[0] != OBS_DIM:
            fatal.append("obs 总维度=%d ≠ %d" % (obs_1d.shape[0], OBS_DIM))

        return len(fatal) == 0, fatal, warn

    # ══════════════════════════════════════════════════════════
    #  动作映射
    # ══════════════════════════════════════════════════════════

    def _map_action(self, raw_action):
        """返回 (raw_v, raw_w, clipped_v, clipped_w, cmd_v, cmd_w)"""
        a = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if a.shape[0] < 2:
            raise ValueError("action 维度=%d < 2" % a.shape[0])

        raw_v, raw_w = float(a[0]), float(a[1])

        if self.use_clip:
            clipped_v = float(np.clip(raw_v, self.action_clip_low, self.action_clip_hi))
            clipped_w = float(np.clip(raw_w, self.action_clip_low, self.action_clip_hi))
        else:
            clipped_v, clipped_w = raw_v, raw_w

        cmd_v = (clipped_v + 1.0) * 0.5 * (self.linear_max  - self.linear_min)  + self.linear_min
        cmd_w = (clipped_w + 1.0) * 0.5 * (self.angular_max - self.angular_min) + self.angular_min

        return raw_v, raw_w, clipped_v, clipped_w, cmd_v, cmd_w

    # ══════════════════════════════════════════════════════════
    #  等待日志（状态变化才打印）
    # ══════════════════════════════════════════════════════════

    def _maybe_log_waiting(self):
        missing = []
        if not self.has_odom:  missing.append("odom")
        if not self.has_goal:  missing.append("goal")
        if not self.has_lidar: missing.append("lidar")
        state = tuple(missing)
        if state != self._last_wait_state:
            self._last_wait_state = state
            if missing:
                rospy.loginfo("[RL] 等待话题就绪: %s", ", ".join(missing))

    # ══════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════

    def cb_odom(self, msg):
        """只更新源数据，不接触 obs buffer。"""
        with self.lock:
            self._src_lin3[0] = msg.twist.twist.linear.x
            self._src_lin3[1] = msg.twist.twist.linear.y
            self._src_lin3[2] = msg.twist.twist.linear.z
            self._src_ang3[0] = msg.twist.twist.angular.x
            self._src_ang3[1] = msg.twist.twist.angular.y
            self._src_ang3[2] = msg.twist.twist.angular.z
            self.t_speed  = rospy.Time.now()
            self.has_odom = True

    def cb_goal(self, msg):
        """只更新源数据，不接触 obs buffer。"""
        data = msg.data
        if len(data) < 4:
            rospy.logwarn_throttle(2.0, "[RL] goal 长度=%d < 4，忽略", len(data))
            return
        with self.lock:
            self._src_goal4[0] = float(data[0])   # dx
            self._src_goal4[1] = float(data[1])   # dy
            self._src_goal4[2] = float(data[2])   # dz
            self._src_goal4[3] = float(data[3])   # dyaw
            self.t_goal   = rospy.Time.now()
            self.has_goal = True
            if self.goal_reached:
                dist = math.sqrt(float(data[0])**2 + float(data[1])**2)
                rospy.loginfo("[RL] 新目标到来 dist=%.2fm，恢复推理", dist)
            self.goal_reached     = False
            self.goal_done_logged = False
            self._last_wait_state = None

    def cb_lidar(self, msg):
        """推理主入口：组装 → 校验 → 推理 → 发布。"""
        data = msg.data
        if len(data) < DIM_LIDAR:
            rospy.logwarn_throttle(2.0, "[RL] lidar 长度=%d < %d，忽略",
                                   len(data), DIM_LIDAR)
            return

        with self.lock:
            if self.goal_reached:
                return

            # 精确取前 543 个值写入源数组
            self._src_lidar[:] = np.asarray(data[:DIM_LIDAR], dtype=np.float32)
            self.t_lidar  = rospy.Time.now()
            self.has_lidar = True

            # 等待所有话题首次就绪
            if not (self.has_odom and self.has_goal and self.has_lidar):
                self._maybe_log_waiting()
                return
            self._last_wait_state = ()

            # 到达目标检测
            if self._goal_dist_xy() <= self.goal_tolerance:
                self.goal_reached = True
                if not self.goal_done_logged:
                    if self.pub_zero_on_goal:
                        self.safe_publish(self.pub_cmd, Twist(), "cmd_zero")
                    rospy.loginfo("[RL] *** 已到达目标点 dist=%.3fm，等待下一目标 ***",
                                  self._goal_dist_xy())
                    self.goal_done_logged = True
                return

            # 时序检查
            if self.t_goal is not None:
                age = (self.t_lidar - self.t_goal).to_sec() * 1000.0
                if age > self.warn_goal_age_ms:
                    rospy.logwarn_throttle(1.0, "[RL][SYNC] goal_age=%.1fms", age)
            if self.t_speed is not None:
                age = (self.t_lidar - self.t_speed).to_sec() * 1000.0
                if age > self.warn_speed_age_ms:
                    rospy.logwarn_throttle(1.0, "[RL][SYNC] odom_age=%.1fms", age)

            # 组装观测（返回独立副本，shape=(1,556)）
            try:
                obs_2d = self._build_obs_556()
            except AssertionError as e:
                rospy.logwarn_throttle(1.0, "[RL][OBS] 维度断言失败: %s", e)
                return

            obs_1d = obs_2d.reshape(-1)   # (556,)，与 obs_2d 共享内存

            # 自校验 + 自修正（就地修改 obs_1d，obs_2d 同步更新）
            ok, fatal_msgs, warn_msgs = self._validate_and_correct(obs_1d)
            for m in warn_msgs:
                rospy.logwarn_throttle(1.0, "[RL][VAL] %s", m)
            if not ok:
                for m in fatal_msgs:
                    rospy.logwarn_throttle(1.0, "[RL][VAL][FATAL] %s", m)
                return

            # 发布 obs_556（推理前的最终输入，供实时核查）
            self.safe_publish(self.pub_obs_556, self._to_msg(obs_1d), "obs_556")

            # ONNX 推理
            try:
                outputs    = self.session.run([self._out_name],
                                             {self._in_name: obs_2d})
                raw_action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] 推理失败: %s", e)
                return

            # 动作映射
            try:
                raw_v, raw_w, clip_v, clip_w, cmd_v, cmd_w = self._map_action(raw_action)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] 动作映射失败: %s", e)
                return

            # 发布速度指令
            cmd = Twist()
            cmd.linear.x  = cmd_v
            cmd.angular.z = cmd_w
            self.safe_publish(self.pub_cmd, cmd, "cmd_vel")

            # 推理日志（0.5s 一次）
            dx   = float(self._src_goal4[0])
            dy   = float(self._src_goal4[1])
            dz   = float(self._src_goal4[2])
            dyaw = float(self._src_goal4[3])
            dist = math.sqrt(dx*dx + dy*dy)

            rospy.loginfo_throttle(
                0.5,
                "[RL] goal dx=%+.3f dy=%+.3f dz=%+.3f dyaw=%+.4f dist=%.3fm"
                " | model raw[%+.4f,%+.4f] clip[%+.4f,%+.4f]"
                " | cmd v=%+.4f w=%+.4f",
                dx, dy, dz, dyaw, dist,
                raw_v, raw_w,
                clip_v, clip_w,
                cmd_v, cmd_w
            )


def main():
    try:
        RLOnnxInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
