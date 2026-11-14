#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

"""
rl_planner_node_10.py
=====================
改进点（相对 v9）：
  Fix-1  锁内不再直接调用 publish，改为锁内收集待发布数据，锁外统一发布，消除潜在死锁。
  Fix-2  新增 /imu 订阅，动态更新 projected_gravity（[6:9]），替代原来的静态常量。
         若无 IMU 话题，可通过参数 use_static_gravity=true 退化为静态重力向量。
  Fix-3  _clip_if_valid 增加 None 防御，避免参数异常时 TypeError。
  Fix-4  goal 有效性检查增加滞回（hysteresis）逻辑，消除 cmd_vel 忽零忽非零抖动。
  Fix-5  lidar 维度检查改为 shape 校验；魔法数字提取为模块级常量。
  Fix-6  _publish_zero_cmd 简化为直接发布 Twist()（默认全零）。
  Fix-7  节点注册 rospy.on_shutdown 钩子，退出前发布一次零速指令。
  Fix-8  _infer 中 obs_batch 构建改用 [None, :] 切片，语义更清晰。
"""

import math
import threading
import rospy
import numpy as np
import onnxruntime as ort

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# 模块级常量（Fix-5）
# ---------------------------------------------------------------------------
LIDAR_FRAME_DIM: int = 181                              # 单帧 LiDAR 维度
LIDAR_FRAMES:    int = 3                                # 帧数
LIDAR_TOTAL:     int = LIDAR_FRAME_DIM * LIDAR_FRAMES  # 543
OBS_DIM:         int = 3 + 3 + 3 + 4 + LIDAR_TOTAL    # 556
ACTION_DIM:      int = 2


class RLOnnxInferenceNode(object):
    """
    RL ONNX 推理节点（556 维观测，事件触发版，v10）

    观测顺序（556 维）：
      [0:3]    lin_vel           -> [vx, vy, vz]
      [3:6]    ang_vel           -> [wx, wy, wz]
      [6:9]    projected_gravity -> [gx, gy, gz]（动态 IMU 或静态参数）
      [9:13]   pose_commands     -> [dx_b, dy_b, dz_b, dyaw_b]
      [13:194] lidar_obs0        -> 第 0 帧 LiDAR（181 维，最新帧）
      [194:375] lidar_obs1       -> 第 1 帧 LiDAR（181 维）
      [375:556] lidar_obs2       -> 第 2 帧 LiDAR（181 维，最旧帧）
    """

    def __init__(self):
        rospy.init_node("rl_onnx_inference_node", anonymous=False)

        # ------------------------------------------------------------------ #
        # 参数读取
        # ------------------------------------------------------------------ #
        default_model = "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx"
        self.model_path    = rospy.get_param("~model_path",    default_model)
        self.input_name    = rospy.get_param("~input_name",    "obs")
        self.output_name   = rospy.get_param("~output_name",   "action")

        self.speed_topic   = rospy.get_param("~speed_topic",   "/zeus_speed")
        self.goal_topic    = rospy.get_param("~goal_topic",    "/rl_local_goal_body")
        self.lidar_topic   = rospy.get_param("~lidar_topic",   "/rl/lidar_543")
        self.imu_topic     = rospy.get_param("~imu_topic",     "/imu")
        self.out_cmd_topic = rospy.get_param("~out_cmd_topic", "/rl_cmd_vel")
        self.obs_topic     = rospy.get_param("~obs_topic",     "/rl/obs_556")

        self.debug_lin_topic  = rospy.get_param("~debug_lin_topic",  "/rl/debug/lin_vel3")
        self.debug_ang_topic  = rospy.get_param("~debug_ang_topic",  "/rl/debug/ang_vel3")
        self.debug_goal_topic = rospy.get_param("~debug_goal_topic", "/rl/debug/goal4")
        self.debug_obs0_topic = rospy.get_param("~debug_obs0_topic", "/rl/debug/obs0_181")
        self.debug_obs1_topic = rospy.get_param("~debug_obs1_topic", "/rl/debug/obs1_181")
        self.debug_obs2_topic = rospy.get_param("~debug_obs2_topic", "/rl/debug/obs2_181")

        # Fix-2: 重力模式参数
        self.use_static_gravity = bool(rospy.get_param("~use_static_gravity", False))
        self.gravity_x = float(rospy.get_param("~gravity_x",  0.0))
        self.gravity_y = float(rospy.get_param("~gravity_y",  0.0))
        self.gravity_z = float(rospy.get_param("~gravity_z", -1.0))

        # Fix-4: goal 滞回阈值（enter > exit，形成死区）
        self.goal_nonzero_eps   = float(rospy.get_param("~goal_nonzero_eps",   1e-4))
        self.goal_pos_eps_enter = float(rospy.get_param("~goal_pos_eps_enter", 0.05))
        self.goal_pos_eps_exit  = float(rospy.get_param("~goal_pos_eps_exit",  0.02))
        self.goal_yaw_eps_enter = float(rospy.get_param("~goal_yaw_eps_enter", 0.05))
        self.goal_yaw_eps_exit  = float(rospy.get_param("~goal_yaw_eps_exit",  0.02))
        self.use_split_goal_gate = bool(rospy.get_param("~use_split_goal_gate", True))
        self._goal_active = False   # 滞回状态机

        self.publish_obs_when_goal_invalid = bool(
            rospy.get_param("~publish_obs_when_goal_invalid", False)
        )

        self.clip_v_min = float(rospy.get_param("~clip_v_min", -0.2))
        self.clip_v_max = float(rospy.get_param("~clip_v_max",  1.5))
        self.clip_w_min = float(rospy.get_param("~clip_w_min", -3.0))
        self.clip_w_max = float(rospy.get_param("~clip_w_max",  3.0))

        self.publish_obs                = bool(rospy.get_param("~publish_obs",                True))
        self.debug_publish_inputs       = bool(rospy.get_param("~debug_publish_inputs",       True))
        self.debug_log                  = bool(rospy.get_param("~debug_log",                  True))
        self.debug_obs_log              = bool(rospy.get_param("~debug_obs_log",              False))
        self.zero_cmd_when_goal_invalid = bool(rospy.get_param("~zero_cmd_when_goal_invalid", True))

        # ------------------------------------------------------------------ #
        # 最新数据缓存
        # ------------------------------------------------------------------ #
        self.latest_lin3     = None
        self.latest_ang3     = None
        self.latest_goal4    = None
        self.latest_lidar543 = None
        # Fix-2: 重力向量缓存（IMU 模式下动态更新；静态模式下保持参数值）
        self.latest_gravity3 = np.array(
            [self.gravity_x, self.gravity_y, self.gravity_z], dtype=np.float32
        )

        self.lock = threading.Lock()

        # ------------------------------------------------------------------ #
        # ONNX 模型加载
        # ------------------------------------------------------------------ #
        rospy.loginfo("[RL] loading ONNX model: %s", self.model_path)
        self.session = ort.InferenceSession(
            self.model_path,
            providers=["CPUExecutionProvider"]
        )

        actual_input_name  = self.session.get_inputs()[0].name
        actual_output_name = self.session.get_outputs()[0].name

        if self.input_name != actual_input_name:
            rospy.logwarn(
                "[RL] input_name from param is '%s', but model expects '%s'. Use model value.",
                self.input_name, actual_input_name
            )
            self.input_name = actual_input_name
        if self.output_name != actual_output_name:
            rospy.logwarn(
                "[RL] output_name from param is '%s', but model outputs '%s'. Use model value.",
                self.output_name, actual_output_name
            )
            self.output_name = actual_output_name

        rospy.loginfo("[RL] ONNX ready. input=%s output=%s", self.input_name, self.output_name)

        # ------------------------------------------------------------------ #
        # 订阅
        # ------------------------------------------------------------------ #
        rospy.Subscriber(self.speed_topic, Twist,             self.cb_speed, queue_size=1)
        rospy.Subscriber(self.goal_topic,  Float32MultiArray, self.cb_goal,  queue_size=1)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=1)

        # Fix-2: 仅在 IMU 模式下订阅
        if not self.use_static_gravity:
            rospy.Subscriber(self.imu_topic, Imu, self.cb_imu, queue_size=1)
            rospy.loginfo("[RL] IMU mode: subscribing %s for projected_gravity", self.imu_topic)
        else:
            rospy.loginfo(
                "[RL] Static gravity mode: gravity=[%.3f, %.3f, %.3f]",
                self.gravity_x, self.gravity_y, self.gravity_z
            )

        # ------------------------------------------------------------------ #
        # 发布
        # ------------------------------------------------------------------ #
        self.pub_cmd = rospy.Publisher(self.out_cmd_topic, Twist,             queue_size=1)
        self.pub_obs = rospy.Publisher(self.obs_topic,     Float32MultiArray, queue_size=1)

        self.pub_debug_lin  = rospy.Publisher(self.debug_lin_topic,  Float32MultiArray, queue_size=1)
        self.pub_debug_ang  = rospy.Publisher(self.debug_ang_topic,  Float32MultiArray, queue_size=1)
        self.pub_debug_goal = rospy.Publisher(self.debug_goal_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs0 = rospy.Publisher(self.debug_obs0_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs1 = rospy.Publisher(self.debug_obs1_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs2 = rospy.Publisher(self.debug_obs2_topic, Float32MultiArray, queue_size=1)

        # Fix-7: 退出钩子
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[RL] subscribed: speed=%s  goal=%s  lidar=%s",
            self.speed_topic, self.goal_topic, self.lidar_topic
        )
        rospy.loginfo("[RL] publish: cmd=%s  obs=%s", self.out_cmd_topic, self.obs_topic)
        rospy.loginfo("[RL] obs_556 publish mode = callback-driven (event-based)")

    # ====================================================================== #
    # 回调：IMU（Fix-2）
    # ====================================================================== #
    def cb_imu(self, msg):
        """
        从 IMU linear_acceleration 提取归一化重力方向作为 projected_gravity。
        适合低动态场景；高动态场景建议改用 orientation 四元数 + 旋转矩阵方式。
        """
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        grav = np.array([-ax / norm, -ay / norm, -az / norm], dtype=np.float32)
        with self.lock:
            self.latest_gravity3 = grav

    # ====================================================================== #
    # 回调：速度
    # ====================================================================== #
    def cb_speed(self, msg):
        debug_msgs   = {}
        publish_tasks = []

        with self.lock:
            self.latest_lin3 = np.array(
                [msg.linear.x,  msg.linear.y,  msg.linear.z],  dtype=np.float32
            )
            self.latest_ang3 = np.array(
                [msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32
            )
            if self.debug_publish_inputs:
                debug_msgs["lin"] = self._to_msg(self.latest_lin3)
                debug_msgs["ang"] = self._to_msg(self.latest_ang3)
            publish_tasks = self._process_event_locked("speed")

        # Fix-1: 锁外统一 publish
        if debug_msgs:
            self.pub_debug_lin.publish(debug_msgs["lin"])
            self.pub_debug_ang.publish(debug_msgs["ang"])
        self._execute_publish_tasks(publish_tasks)

    # ====================================================================== #
    # 回调：目标
    # ====================================================================== #
    def cb_goal(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        # Fix-5: shape 校验
        if arr.shape != (4,):
            rospy.logwarn_throttle(
                1.0, "[RL] %s shape != (4,), got %s", self.goal_topic, arr.shape
            )
            return

        debug_msgs    = {}
        publish_tasks = []

        with self.lock:
            self.latest_goal4 = arr
            if self.debug_publish_inputs:
                debug_msgs["goal"] = self._to_msg(self.latest_goal4)
            publish_tasks = self._process_event_locked("goal")

        # Fix-1: 锁外统一 publish
        if debug_msgs:
            self.pub_debug_goal.publish(debug_msgs["goal"])
        self._execute_publish_tasks(publish_tasks)

    # ====================================================================== #
    # 回调：LiDAR
    # ====================================================================== #
    def cb_lidar(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        # Fix-5: shape 校验，使用具名常量
        if arr.shape != (LIDAR_TOTAL,):
            rospy.logwarn_throttle(
                1.0, "[RL] %s shape != (%d,), got %s",
                self.lidar_topic, LIDAR_TOTAL, arr.shape
            )
            return

        debug_msgs    = {}
        publish_tasks = []

        with self.lock:
            self.latest_lidar543 = arr
            if self.debug_publish_inputs:
                s = LIDAR_FRAME_DIM
                debug_msgs["obs0"] = self._to_msg(arr[0:s])
                debug_msgs["obs1"] = self._to_msg(arr[s:2 * s])
                debug_msgs["obs2"] = self._to_msg(arr[2 * s:3 * s])
            publish_tasks = self._process_event_locked("lidar")

        # Fix-1: 锁外统一 publish
        if debug_msgs:
            self.pub_debug_obs0.publish(debug_msgs["obs0"])
            self.pub_debug_obs1.publish(debug_msgs["obs1"])
            self.pub_debug_obs2.publish(debug_msgs["obs2"])
        self._execute_publish_tasks(publish_tasks)

    # ====================================================================== #
    # 工具方法
    # ====================================================================== #
    @staticmethod
    def _clip_if_valid(value, min_v, max_v):
        """Fix-3: 增加 None 防御，避免 math.isnan(None) 抛 TypeError"""
        out = value
        if min_v is not None and not math.isnan(min_v):
            out = max(out, min_v)
        if max_v is not None and not math.isnan(max_v):
            out = min(out, max_v)
        return out

    @staticmethod
    def _to_msg(arr):
        msg = Float32MultiArray()
        msg.data = np.asarray(arr, dtype=np.float32).tolist()
        return msg

    def _ready(self):
        return (
            self.latest_lin3     is not None and
            self.latest_ang3     is not None and
            self.latest_goal4    is not None and
            self.latest_lidar543 is not None
        )

    def _goal_valid(self):
        """
        Fix-4: 滞回（Hysteresis）状态机。
          无效态 → 有效态：pos_norm 或 yaw_abs > enter 阈值
          有效态 → 无效态：pos_norm 和 yaw_abs 均 < exit 阈值
        enter > exit 形成死区，消除边界抖动。
        """
        if self.latest_goal4 is None:
            self._goal_active = False
            return False

        goal4 = self.latest_goal4.astype(np.float32)

        if self.use_split_goal_gate:
            pos_norm = float(np.linalg.norm(goal4[:3]))
            yaw_abs  = float(abs(goal4[3]))

            if self._goal_active:
                # 有效态：需要 pos 和 yaw 同时低于 exit 阈值才切换为无效
                self._goal_active = (
                    (pos_norm > self.goal_pos_eps_exit) or
                    (yaw_abs  > self.goal_yaw_eps_exit)
                )
            else:
                # 无效态：pos 或 yaw 超过 enter 阈值才切换为有效
                self._goal_active = (
                    (pos_norm > self.goal_pos_eps_enter) or
                    (yaw_abs  > self.goal_yaw_eps_enter)
                )
        else:
            goal_norm = float(np.linalg.norm(goal4))
            if self._goal_active:
                self._goal_active = goal_norm > self.goal_nonzero_eps * 0.5
            else:
                self._goal_active = goal_norm > self.goal_nonzero_eps

        return self._goal_active

    def _build_obs(self):
        """拼接 556 维观测向量（必须在持锁状态下调用）"""
        lin3     = self.latest_lin3.astype(np.float32)
        ang3     = self.latest_ang3.astype(np.float32)
        gravity3 = self.latest_gravity3.astype(np.float32)  # Fix-2: 动态重力
        goal4    = self.latest_goal4.astype(np.float32)
        lidar543 = self.latest_lidar543.astype(np.float32)

        obs = np.concatenate([lin3, ang3, gravity3, goal4, lidar543]).astype(np.float32)

        if obs.shape != (OBS_DIM,):
            raise RuntimeError(
                "obs dim error: expect {}, got {}".format(OBS_DIM, obs.shape)
            )
        return obs

    def _infer(self, obs_556):
        """Fix-8: 用 [None, :] 替代 reshape(1, OBS_DIM)"""
        obs_batch = obs_556[None, :].astype(np.float32)
        outputs   = self.session.run([self.output_name], {self.input_name: obs_batch})
        action    = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
        if action.shape != (ACTION_DIM,):
            raise RuntimeError(
                "action dim error: expect {}, got {}".format(ACTION_DIM, action.shape)
            )
        return float(action[0]), float(action[1])

    # ====================================================================== #
    # 核心事件处理（必须在持有 self.lock 时调用）
    # 返回 (publisher, message) 列表，由调用方在锁外逐一发布（Fix-1）
    # ====================================================================== #
    def _process_event_locked(self, source_tag):
        tasks = []  # list of (Publisher, message)

        if not self._ready():
            rospy.logwarn_throttle(
                2.0,
                "[RL] waiting topics... source=%s  lin3=%s  ang3=%s  goal4=%s  lidar543=%s",
                source_tag,
                self.latest_lin3     is not None,
                self.latest_ang3     is not None,
                self.latest_goal4    is not None,
                self.latest_lidar543 is not None,
            )
            return tasks

        goal_valid = self._goal_valid()

        if not goal_valid:
            if self.publish_obs and self.publish_obs_when_goal_invalid:
                obs = self._build_obs()
                tasks.append((self.pub_obs, self._to_msg(obs)))
            if self.zero_cmd_when_goal_invalid:
                tasks.append((self.pub_cmd, Twist()))  # Fix-6: Twist() 默认全零
            return tasks

        obs = self._build_obs()

        if self.publish_obs:
            tasks.append((self.pub_obs, self._to_msg(obs)))

        if self.debug_obs_log:
            s = LIDAR_FRAME_DIM
            rospy.loginfo(
                "[RL][OBS][%s] lin=%s ang=%s gravity=%s goal=%s "
                "lidar0[:5]=%s lidar1[:5]=%s lidar2[:5]=%s",
                source_tag,
                np.array2string(obs[0:3],  precision=4, separator=','),
                np.array2string(obs[3:6],  precision=4, separator=','),
                np.array2string(obs[6:9],  precision=4, separator=','),
                np.array2string(obs[9:13], precision=4, separator=','),
                np.array2string(obs[13:18],              precision=4, separator=','),
                np.array2string(obs[13 + s:18 + s],      precision=4, separator=','),
                np.array2string(obs[13 + 2*s:18 + 2*s],  precision=4, separator=','),
            )

        v, w         = self._infer(obs)
        v_raw, w_raw = v, w
        v = self._clip_if_valid(v, self.clip_v_min, self.clip_v_max)
        w = self._clip_if_valid(w, self.clip_w_min, self.clip_w_max)

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        tasks.append((self.pub_cmd, cmd))

        if self.debug_log:
            rospy.loginfo_throttle(
                0.5,
                "[RL][%s] raw=[%.6f, %.6f] -> clipped=[%.6f, %.6f] "
                "| lin=%s  ang=%s",
                source_tag, v_raw, w_raw, v, w,
                np.array2string(self.latest_lin3, precision=4, separator=','),
                np.array2string(self.latest_ang3, precision=4, separator=','),
            )

        return tasks

    @staticmethod
    def _execute_publish_tasks(tasks):
        """Fix-1: 在锁外统一执行所有发布任务"""
        for pub, msg in tasks:
            pub.publish(msg)

    # ====================================================================== #
    # 关闭钩子（Fix-7）
    # ====================================================================== #
    def _on_shutdown(self):
        rospy.loginfo("[RL] shutting down — publishing zero cmd_vel...")
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass

    # ====================================================================== #
    # 主循环
    # ====================================================================== #
    def spin(self):
        rospy.loginfo("[RL] event-driven mode running. waiting callbacks...")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RLOnnxInferenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
