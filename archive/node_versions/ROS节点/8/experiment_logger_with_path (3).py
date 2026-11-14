#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
实验记录 + 轨迹发布节点 (ExperimentLoggerWithPath)

本版本相对前版的重大重构 (对应 11 条修改意见):
  1. 位姿基准统一到 base_footprint:
     /laserpose 只作为 TF 查询触发器, 真正写入 latest_pose 的是
     TF 查询得到的 map -> base_footprint 变换. 轨迹 / 起点 / 终点 /
     路径长度 / 效率 / 终点误差 全部以 base_footprint 为基准.
  2. 增加 goal_active / goal_finished 作为"有目标点"的总开关,
     无有效目标时: 不输出, 不记录, 不启动实验.
  3. 单次实验会话锁: 一个 goal 只能产出一个 TXT. finish_recording
     结束后立即 goal_finished=True / goal_active=False, 锁死,
     直到下一个 cb_goal 才重新打开.
  4. try_start_recording 改为多重 AND 门槛 (目标有效 + 未完成 +
     未在录制 + 模式合法 + 规划器进入执行态 + 必要话题齐全).
  5. finish_recording 后彻底禁止继续追样本 / 重启实验 / 重复输出.
  6. DWA /move_base/status 按 goal_id.stamp 过滤, 旧 goal 的
     SUCCEEDED 一律忽略.
  7. RL 要求 rl_goal_started; DWA 要求 dwa_goal_started; 二者
     均作为 try_start_recording 的硬门槛.
  8. 增加 shutting_down 标志. on_shutdown 设为 True 后, cb_timer
     不再启动新实验, 各 publish 不再外发.
  9. 所有 publish 通过 _safe_publish 封装: 已 shutdown / ROS 已退 /
     抛异常 一律不影响主流程.
  10. success 只认 finish_reason 以 goal_reached 结尾;
      goal_reach_distance 只作为辅助分析指标, 不参与 success 判定.
  11. IMU 降级为可选项: 无 IMU 时 ax/ay/az/gx/gy/gz 写 0.0,
      不阻塞启动 / 不阻塞写样本.
"""

from __future__ import print_function

import os
import math
import threading

import rospy
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray


# ---- actionlib GoalStatus 常量 -----------------------------------------------
GS_PENDING    = 0
GS_ACTIVE     = 1
GS_PREEMPTED  = 2
GS_SUCCEEDED  = 3
GS_ABORTED    = 4
GS_REJECTED   = 5
GS_PREEMPTING = 6
GS_RECALLING  = 7
GS_RECALLED   = 8
GS_LOST       = 9

TERMINAL_DWA_STATUSES = (GS_PREEMPTED, GS_SUCCEEDED, GS_ABORTED,
                         GS_REJECTED, GS_RECALLED, GS_LOST)


def quat_to_yaw(x, y, z, w):
    """单位四元数 -> yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def ensure_dir(path):
    """创建目录 (幂等, 防 race)."""
    try:
        os.makedirs(path)
    except OSError:
        if not os.path.isdir(path):
            raise


class ExperimentLoggerWithPath(object):
    def __init__(self):
        rospy.init_node("experiment_logger_with_path", anonymous=False)

        # ===== 话题参数 =====
        self.active_mode_topic = rospy.get_param("~active_mode_topic", "/cmd_vel_switch/active_mode")
        self.goal_topic        = rospy.get_param("~goal_topic",        "/move_base/current_goal")
        self.pose_topic        = rospy.get_param("~pose_topic",        "/laserpose")   # 仅作为 TF 查询触发
        self.speed_topic       = rospy.get_param("~speed_topic",       "/zeus_speed")
        self.imu_topic         = rospy.get_param("~imu_topic",         "/imu/data")
        self.dwa_status_topic  = rospy.get_param("~dwa_status_topic",  "/move_base/status")
        self.rl_status_topic   = rospy.get_param("~rl_status_topic",   "/rl_local_goal_status")
        self.path_topic        = rospy.get_param("~path_topic",        "/trajectory")

        # ===== 坐标系参数 =====
        self.map_frame         = rospy.get_param("~map_frame",         "map")
        self.robot_frame       = rospy.get_param("~robot_frame",       "base_footprint")
        self.tf_lookup_timeout = float(rospy.get_param("~tf_lookup_timeout", 0.1))

        # ===== 保存与采样参数 =====
        self.save_dir             = os.path.expanduser(rospy.get_param("~save_dir", "/home/ros/save_dir"))
        self.record_rate_hz       = float(rospy.get_param("~record_rate_hz", 10.0))
        self.min_traj_dist        = float(rospy.get_param("~min_traj_dist", 0.02))
        self.max_path_points      = int(rospy.get_param("~max_path_points", 5000))
        self.path_clear_delay_sec = float(rospy.get_param("~path_clear_delay_sec", 5.0))
        self.goal_reach_tol       = float(rospy.get_param("~goal_reach_tol", 0.25))
        # 过滤旧 goal 的 /move_base/status 时, 允许的 stamp 容差
        self.goal_stamp_tol_sec   = float(rospy.get_param("~goal_stamp_tol_sec", 0.5))

        # ===== 合法模式 =====
        valid_modes_param = rospy.get_param("~valid_modes", ["rl", "dwa"])
        self.valid_modes = set([str(m).strip().lower() for m in valid_modes_param])

        # ===== 创建目录 =====
        self.rl_save_dir  = os.path.join(self.save_dir, "rl")
        self.dwa_save_dir = os.path.join(self.save_dir, "dwa")
        for d in [self.save_dir, self.rl_save_dir, self.dwa_save_dir]:
            ensure_dir(d)

        # ===== 共享状态锁 =====
        # 保护 latest_goal / latest_pose / latest_speed / latest_imu /
        # 以及目标生命周期量的原子更新.
        self._state_lock = threading.Lock()

        # ===== 关闭总闸 =====
        # on_shutdown 置 True 之后, 所有 cb / timer / publish 都快速退出.
        self.shutting_down = False

        # ===== 当前系统状态 =====
        self.active_mode  = ""
        self.latest_goal  = None     # /move_base/current_goal 得到, map 坐标系
        self.latest_pose  = None     # base_footprint 在 map 下的位姿 (TF)
        self.latest_speed = None     # /zeus_speed (Twist)
        self.latest_imu   = None     # /imu/data, 可选, 可能为 None

        # ===== 目标生命周期 =====
        self.goal_active            = False   # 是否存在正在跟踪的目标
        self.goal_finished          = False   # 当前目标是否已结束 (记录已落盘 or 被放弃)
        self.goal_arrival_wall_time = 0.0     # cb_goal 的本机时刻 (wall clock)
        self.goal_arrival_stamp_sec = 0.0     # cb_goal 消息的 header.stamp, 用于 DWA 过滤

        # RL / DWA 子状态
        self.rl_goal_status      = "none"
        self.rl_goal_started     = False
        self.dwa_goal_started    = False
        self.dwa_terminal_status = None
        self.dwa_terminal_text   = ""

        # ===== 实验记录状态 =====
        self.recording             = False
        self.current_record        = None
        self.current_mode          = ""
        self.experiment_id         = self.get_next_experiment_id()
        self.pending_finish_reason = None

        # ===== RViz 轨迹 =====
        self.path_pub            = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.path_msg            = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.last_path_x         = None
        self.last_path_y         = None
        self.path_active         = False
        self.path_clear_deadline = None

        # ===== TF =====
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ===== 订阅 =====
        rospy.Subscriber(self.active_mode_topic, String,                      self.cb_active_mode, queue_size=10)
        rospy.Subscriber(self.goal_topic,        PoseStamped,                 self.cb_goal,        queue_size=10)
        rospy.Subscriber(self.pose_topic,        PoseWithCovarianceStamped,   self.cb_pose,        queue_size=50)
        rospy.Subscriber(self.speed_topic,       Twist,                       self.cb_speed,       queue_size=50)
        rospy.Subscriber(self.imu_topic,         Imu,                         self.cb_imu,         queue_size=50)
        rospy.Subscriber(self.dwa_status_topic,  GoalStatusArray,             self.cb_dwa_status,  queue_size=20)
        rospy.Subscriber(self.rl_status_topic,   String,                      self.cb_rl_status,   queue_size=20)

        # ===== 定时器 =====
        period = 1.0 / self.record_rate_hz if self.record_rate_hz > 0.0 else 0.1
        rospy.Timer(rospy.Duration(period), self.cb_timer)

        # ===== 关闭钩子 =====
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("实验记录+轨迹发布节点已启动")
        rospy.loginfo("map_frame         : %s", self.map_frame)
        rospy.loginfo("robot_frame       : %s  (所有位姿与指标以此为基准)", self.robot_frame)
        rospy.loginfo("tf_lookup_timeout : %.3f s", self.tf_lookup_timeout)
        rospy.loginfo("active_mode_topic : %s", self.active_mode_topic)
        rospy.loginfo("goal_topic        : %s", self.goal_topic)
        rospy.loginfo("pose_topic (trig) : %s", self.pose_topic)
        rospy.loginfo("speed_topic       : %s", self.speed_topic)
        rospy.loginfo("imu_topic         : %s  (可选, 无则相关字段写 0)", self.imu_topic)
        rospy.loginfo("dwa_status_topic  : %s", self.dwa_status_topic)
        rospy.loginfo("rl_status_topic   : %s", self.rl_status_topic)
        rospy.loginfo("path_topic        : %s", self.path_topic)
        rospy.loginfo("save_dir          : %s", self.save_dir)
        rospy.loginfo("path_clear_delay  : %.2f s", self.path_clear_delay_sec)
        rospy.loginfo("goal_reach_tol    : %.3f m  (辅助分析, 不作为 success 判据)", self.goal_reach_tol)
        rospy.loginfo("goal_stamp_tol    : %.3f s  (DWA status 过滤容差)", self.goal_stamp_tol_sec)
        rospy.loginfo("valid_modes       : %s", sorted(self.valid_modes))
        rospy.loginfo("下一个实验编号    : %04d", self.experiment_id)

    # ==================================================================
    # 工具方法
    # ==================================================================
    def get_next_experiment_id(self):
        existing = []
        for root_dir in [self.rl_save_dir, self.dwa_save_dir]:
            if not os.path.exists(root_dir):
                continue
            for name in os.listdir(root_dir):
                if name.startswith("exp_") and name.endswith(".txt"):
                    try:
                        idx = int(name.split("_")[1])
                        existing.append(idx)
                    except (ValueError, IndexError):
                        continue
        return max(existing) + 1 if existing else 1

    @staticmethod
    def distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_mode_save_dir(self, mode):
        if mode == "rl":
            return self.rl_save_dir
        if mode == "dwa":
            return self.dwa_save_dir
        return self.save_dir

    def _safe_publish(self, pub, msg):
        """安全发布: shutting_down 或 rospy 已退时不发; publish 异常打日志不抛."""
        if self.shutting_down or rospy.is_shutdown():
            return
        try:
            pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "publish 异常: %s", str(e))

    # ==================================================================
    # TF: map -> base_footprint
    # ==================================================================
    def lookup_robot_pose(self, stamp):
        """查询 base_footprint 在 map 下的位姿; 成功返回 dict, 失败返回 None."""
        if self.shutting_down:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                stamp, rospy.Duration(self.tf_lookup_timeout))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # 回退: 用最新可用变换 (Time(0))
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.map_frame, self.robot_frame, rospy.Time(0))
            except Exception as e:
                rospy.logwarn_throttle(
                    2.0, "TF 查询失败 %s -> %s: %s",
                    self.map_frame, self.robot_frame, str(e))
                return None

        t = tr.transform.translation
        q = tr.transform.rotation
        return {
            "x":  t.x,
            "y":  t.y,
            "z":  t.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "yaw": quat_to_yaw(q.x, q.y, q.z, q.w),
            "frame_id": self.map_frame,
            "stamp": tr.header.stamp.to_sec(),
        }

    # ==================================================================
    # 轨迹发布 (所有 publish 都走 _safe_publish)
    # ==================================================================
    def clear_path(self, frame_id=None):
        if frame_id is None:
            frame_id = self.map_frame
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp    = rospy.Time.now()
        self.last_path_x = None
        self.last_path_y = None
        self._safe_publish(self.path_pub, self.path_msg)

    def schedule_clear_path(self):
        self.path_clear_deadline = rospy.Time.now().to_sec() + self.path_clear_delay_sec
        rospy.loginfo("轨迹将在 %.2f 秒后自动清空", self.path_clear_delay_sec)

    def append_path_point(self, pose_dict):
        if self.shutting_down:
            return
        x        = pose_dict["x"]
        y        = pose_dict["y"]
        frame_id = pose_dict["frame_id"]

        if not self.path_msg.header.frame_id:
            self.path_msg.header.frame_id = frame_id

        if self.last_path_x is None or self.last_path_y is None:
            need_add = True
        else:
            need_add = self.distance(x, y, self.last_path_x, self.last_path_y) >= self.min_traj_dist

        if not need_add:
            self.path_msg.header.stamp = rospy.Time.now()
            self._safe_publish(self.path_pub, self.path_msg)
            return

        ps = PoseStamped()
        ps.header.stamp        = rospy.Time.now()
        ps.header.frame_id     = frame_id
        ps.pose.position.x     = x
        ps.pose.position.y     = y
        ps.pose.position.z     = 0.0
        ps.pose.orientation.x  = pose_dict["qx"]
        ps.pose.orientation.y  = pose_dict["qy"]
        ps.pose.orientation.z  = pose_dict["qz"]
        ps.pose.orientation.w  = pose_dict["qw"]

        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_points:]

        self.last_path_x = x
        self.last_path_y = y
        self.path_msg.header.stamp = rospy.Time.now()
        self._safe_publish(self.path_pub, self.path_msg)

    # ==================================================================
    # 订阅回调
    # ==================================================================
    def cb_active_mode(self, msg):
        if self.shutting_down:
            return
        new_mode = msg.data.strip().lower()
        old_mode = self.active_mode
        self.active_mode = new_mode

        if new_mode != old_mode:
            rospy.loginfo("实际控制模式切换: %s -> %s",
                          old_mode if old_mode else "none", new_mode)

        if self.recording and new_mode != self.current_mode:
            self.pending_finish_reason = "mode_switched_to_%s" % new_mode

    def cb_goal(self, msg):
        """收到新 goal: 打开新的目标生命周期, 重置所有子状态."""
        if self.shutting_down:
            return

        goal = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw": quat_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            "frame_id": msg.header.frame_id if msg.header.frame_id else self.map_frame,
            "stamp":    msg.header.stamp.to_sec(),
        }

        with self._state_lock:
            self.latest_goal             = goal
            self.goal_active             = True
            self.goal_finished           = False
            self.goal_arrival_wall_time  = rospy.Time.now().to_sec()
            self.goal_arrival_stamp_sec  = goal["stamp"]

            # 重置所有子状态: 上一个 goal 的 RL/DWA 状态与此刻无关
            self.rl_goal_status      = "none"
            self.rl_goal_started     = False
            self.dwa_goal_started    = False
            self.dwa_terminal_status = None
            self.dwa_terminal_text   = ""

        # 若上一个实验未自然结束就来了新 goal, 打断它
        if self.recording:
            self.pending_finish_reason = "new_goal_received"

        # 新 goal = 新轨迹
        self.path_clear_deadline = None
        self.clear_path(frame_id=self.map_frame)
        self.path_active = True

        rospy.loginfo("收到新目标点: x=%.3f, y=%.3f, frame=%s, goal_stamp=%.6f",
                      goal["x"], goal["y"], goal["frame_id"], goal["stamp"])

    def cb_pose(self, msg):
        """/laserpose 只用作 TF 查询触发; latest_pose 存的是 base_footprint 位姿."""
        if self.shutting_down:
            return

        pose = self.lookup_robot_pose(msg.header.stamp)
        if pose is None:
            return

        with self._state_lock:
            self.latest_pose = pose

        # 必须 有效目标 且 未完成 且 在录制 才追加轨迹点与发布
        if (self.recording and self.goal_active and (not self.goal_finished)
                and self.path_active and self.current_mode in self.valid_modes):
            self.append_path_point(pose)
        # 不再无条件 republish: 无目标时节点不输出

    def cb_speed(self, msg):
        if self.shutting_down:
            return
        speed = {
            "v":     msg.linear.x,
            "w":     msg.angular.z,
            "stamp": rospy.Time.now().to_sec(),
        }
        with self._state_lock:
            self.latest_speed = speed

    def cb_imu(self, msg):
        if self.shutting_down:
            return
        imu = {
            "ax":    msg.linear_acceleration.x,
            "ay":    msg.linear_acceleration.y,
            "az":    msg.linear_acceleration.z,
            "gx":    msg.angular_velocity.x,
            "gy":    msg.angular_velocity.y,
            "gz":    msg.angular_velocity.z,
            "stamp": msg.header.stamp.to_sec(),
        }
        with self._state_lock:
            self.latest_imu = imu

    def cb_rl_status(self, msg):
        if self.shutting_down:
            return
        # 无有效目标 / 当前目标已锁死: 一律忽略, 避免旧状态触发
        if (not self.goal_active) or self.goal_finished:
            return

        status = msg.data.strip()
        self.rl_goal_status = status

        if status in ("goal_received", "goal_tracking"):
            self.rl_goal_started = True

        if self.recording and self.current_mode == "rl":
            if status == "goal_reached":
                self.pending_finish_reason = "rl_goal_reached"
            elif status in ("goal_cleared", "shutdown"):
                self.pending_finish_reason = "rl_goal_terminated_%s" % status

    def cb_dwa_status(self, msg):
        """按 goal_id.stamp 过滤, 旧 goal 的 SUCCEEDED 一律忽略."""
        if self.shutting_down:
            return
        if not msg.status_list:
            return
        if (not self.goal_active) or self.goal_finished:
            return
        if self.latest_goal is None:
            return

        # 只处理 status_list 中 goal_id.stamp 最新的一条
        latest = max(msg.status_list,
                     key=lambda st: st.goal_id.stamp.to_sec())
        latest_stamp = latest.goal_id.stamp.to_sec()

        # 该 status 还对应旧 goal -> 直接丢弃
        if latest_stamp + self.goal_stamp_tol_sec < self.goal_arrival_stamp_sec:
            return

        if latest.status == GS_ACTIVE:
            self.dwa_goal_started = True
            return

        if latest.status in TERMINAL_DWA_STATUSES and self.dwa_goal_started:
            self.dwa_terminal_status = latest.status
            self.dwa_terminal_text   = latest.text
            if self.recording and self.current_mode == "dwa":
                if latest.status == GS_SUCCEEDED:
                    self.pending_finish_reason = "dwa_goal_reached"
                else:
                    self.pending_finish_reason = \
                        "dwa_goal_terminated_status_%d" % latest.status

    # ==================================================================
    # 定时器 + 录制控制
    # ==================================================================
    def cb_timer(self, event):
        if self.shutting_down:
            return

        now_sec = rospy.Time.now().to_sec()

        # 非录制期: 到期清空轨迹
        if (not self.recording) and (self.path_clear_deadline is not None):
            if now_sec >= self.path_clear_deadline:
                self.clear_path(frame_id=self.map_frame)
                self.path_clear_deadline = None
                rospy.loginfo("已按延迟策略清空轨迹")

        if not self.recording:
            self.try_start_recording()

        if self.recording:
            if self.pending_finish_reason is not None:
                self.finish_recording(self.pending_finish_reason)
                return
            if self.active_mode not in self.valid_modes:
                self.finish_recording("left_valid_mode")
                return
            if self.active_mode != self.current_mode:
                self.finish_recording("mode_changed")
                return
            self.append_sample()

    def try_start_recording(self):
        """启动录制的多重 AND 门槛. 任一项不满足即拒绝."""
        if self.shutting_down:
            return
        if self.recording:
            return
        # ---- 目标生命周期总闸 ----
        if not self.goal_active:
            return
        if self.goal_finished:
            return
        # ---- 模式合法 ----
        if self.active_mode not in self.valid_modes:
            return
        # ---- 规划器执行态门槛 ----
        if self.active_mode == "rl" and not self.rl_goal_started:
            return
        if self.active_mode == "dwa" and not self.dwa_goal_started:
            return
        # ---- 必要话题齐全 (IMU 不在此列) ----
        with self._state_lock:
            if (self.latest_goal is None or self.latest_pose is None
                    or self.latest_speed is None):
                return

        self.start_recording()

    def start_recording(self):
        self.current_mode          = self.active_mode
        self.pending_finish_reason = None
        self.path_clear_deadline   = None

        with self._state_lock:
            start_pose = dict(self.latest_pose) if self.latest_pose else None
            goal_pose  = dict(self.latest_goal) if self.latest_goal else None

        if start_pose is None or goal_pose is None:
            return

        self.clear_path(frame_id=self.map_frame)
        self.path_active = True

        self.current_record = {
            "experiment_id":   self.experiment_id,
            "control_mode":    self.current_mode,
            "start_time":      rospy.Time.now().to_sec(),
            "start_pose":      start_pose,
            "goal_pose":       goal_pose,
            "end_time":        None,
            "end_pose":        None,
            "finish_reason":   "",
            "rl_goal_status":  "",
            "dwa_status_code": None,
            "dwa_status_text": "",
            "samples":         [],
        }

        self.recording = True
        self.append_sample()

        rospy.loginfo("开始记录实验 #%04d, mode=%s, goal=(%.3f, %.3f)",
                      self.experiment_id, self.current_mode,
                      goal_pose["x"], goal_pose["y"])

    def append_sample(self):
        """一次快照: pose + speed 必选; imu 可选, 缺失时写 0."""
        if self.current_record is None:
            return
        with self._state_lock:
            if self.latest_pose is None or self.latest_speed is None:
                return
            pt   = self.latest_pose["stamp"]
            px   = self.latest_pose["x"]
            py   = self.latest_pose["y"]
            pyaw = self.latest_pose["yaw"]
            sv   = self.latest_speed["v"]
            sw   = self.latest_speed["w"]
            # IMU 可选
            if self.latest_imu is not None:
                ax = self.latest_imu["ax"]
                ay = self.latest_imu["ay"]
                az = self.latest_imu["az"]
                gx = self.latest_imu["gx"]
                gy = self.latest_imu["gy"]
                gz = self.latest_imu["gz"]
            else:
                ax = ay = az = 0.0
                gx = gy = gz = 0.0

        self.current_record["samples"].append({
            "t":   pt,
            "x":   px,
            "y":   py,
            "yaw": pyaw,
            "v":   sv,
            "w":   sw,
            "ax":  ax,
            "ay":  ay,
            "az":  az,
            "gx":  gx,
            "gy":  gy,
            "gz":  gz,
        })

    def finish_recording(self, reason):
        if not self.recording or self.current_record is None:
            return

        with self._state_lock:
            end_pose = dict(self.latest_pose) if self.latest_pose else None

        self.current_record["end_time"]        = rospy.Time.now().to_sec()
        self.current_record["end_pose"]        = end_pose
        self.current_record["finish_reason"]   = reason
        self.current_record["rl_goal_status"]  = self.rl_goal_status
        self.current_record["dwa_status_code"] = self.dwa_terminal_status
        self.current_record["dwa_status_text"] = self.dwa_terminal_text

        try:
            self.save_record_to_txt(self.current_record)
        except Exception as e:
            rospy.logerr("实验记录保存失败 (#%04d): %s",
                         self.current_record["experiment_id"], str(e))

        rospy.loginfo("实验 #%04d 已结束, mode=%s, reason=%s, sample_count=%d",
                      self.current_record["experiment_id"],
                      self.current_record["control_mode"],
                      reason,
                      len(self.current_record["samples"]))

        # ---- 单次实验会话锁: 立即锁死, 直到下一个 cb_goal 才解锁 ----
        with self._state_lock:
            self.goal_finished = True
            self.goal_active   = False

        self.path_active = False
        self.schedule_clear_path()

        self.recording             = False
        self.current_record        = None
        self.current_mode          = ""
        self.pending_finish_reason = None
        self.experiment_id        += 1

    # ==================================================================
    # 关闭钩子
    # ==================================================================
    def on_shutdown(self):
        """置 shutting_down 总闸, 并把进行中的实验落盘."""
        self.shutting_down = True
        if self.recording and self.current_record is not None:
            rospy.logwarn("节点正在关闭, 保存进行中的实验 #%04d",
                          self.current_record["experiment_id"])
            try:
                self.finish_recording("node_shutdown")
            except Exception as e:
                rospy.logerr("shutdown 保存失败: %s", str(e))

    # ==================================================================
    # 指标计算
    # ==================================================================
    @staticmethod
    def compute_path_length(samples):
        """相邻样本 (x,y) 欧氏距离累加, 得到总路径长度 (m)."""
        if len(samples) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(samples)):
            dx = samples[i]["x"] - samples[i - 1]["x"]
            dy = samples[i]["y"] - samples[i - 1]["y"]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    def is_success(self, record):
        """success 只认 finish_reason 以 'goal_reached' 结尾.
        即 rl_goal_reached / dwa_goal_reached.
        goal_reach_distance 仅作辅助指标, 不参与 success 判定."""
        reason = record.get("finish_reason") or ""
        return reason.endswith("goal_reached")

    # ==================================================================
    # 文件输出
    # ==================================================================
    def save_record_to_txt(self, record):
        exp_id = record["experiment_id"]
        mode   = record["control_mode"]

        save_root = self.get_mode_save_dir(mode)
        ensure_dir(save_root)

        filename = "exp_%04d_%s.txt" % (exp_id, mode)
        filepath = os.path.join(save_root, filename)
        tmp_path = filepath + ".tmp"

        samples    = record["samples"]
        start_pose = record["start_pose"]
        goal_pose  = record["goal_pose"]
        end_pose   = record["end_pose"]

        duration = 0.0
        if record["end_time"] is not None and record["start_time"] is not None:
            duration = record["end_time"] - record["start_time"]

        # ---- 速度基础统计 ----
        v_list = [s["v"] for s in samples]
        w_list = [s["w"] for s in samples]
        avg_v = sum(v_list) / len(v_list) if v_list else 0.0
        avg_w = sum(w_list) / len(w_list) if w_list else 0.0
        max_v = max(v_list) if v_list else 0.0
        min_v = min(v_list) if v_list else 0.0
        max_w = max(w_list) if w_list else 0.0
        min_w = min(w_list) if w_list else 0.0

        # ---- 综合指标 ----
        total_path_length = self.compute_path_length(samples)

        straight_sg = 0.0
        if start_pose and goal_pose:
            straight_sg = self.distance(start_pose["x"], start_pose["y"],
                                        goal_pose["x"],  goal_pose["y"])
        straight_se = 0.0
        if start_pose and end_pose:
            straight_se = self.distance(start_pose["x"], start_pose["y"],
                                        end_pose["x"],   end_pose["y"])
        goal_reach_dist = float("inf")
        if end_pose and goal_pose:
            goal_reach_dist = self.distance(end_pose["x"], end_pose["y"],
                                            goal_pose["x"], goal_pose["y"])

        path_efficiency = 0.0
        if total_path_length > 1e-6:
            path_efficiency = straight_se / total_path_length
        path_efficiency = max(0.0, min(1.0, path_efficiency))

        avg_linear_speed = 0.0
        if duration > 1e-6:
            avg_linear_speed = total_path_length / duration

        success = self.is_success(record)

        # ---- 原子写入 ----
        with open(tmp_path, "w") as f:
            # ---- 实验头部 ----
            f.write("=" * 80 + "\n")
            f.write("实验编号: %04d\n" % exp_id)
            f.write("控制模式: %s\n" % mode)
            f.write("是否到达目标: %s\n" % ("YES" if success else "NO"))
            f.write("位姿基准: %s (in %s)\n" % (self.robot_frame, self.map_frame))
            f.write("记录起始时间戳: %.6f\n" % record["start_time"])
            f.write("记录终止时间戳: %.6f\n" % record["end_time"])
            f.write("实验持续时长(s): %.6f\n" % duration)
            f.write("结束原因: %s\n" % record["finish_reason"])
            f.write("RL状态话题最终值: %s\n" % record["rl_goal_status"])
            f.write("DWA状态码: %s\n" % str(record["dwa_status_code"]))
            f.write("DWA状态文本: %s\n" % record["dwa_status_text"])
            f.write("=" * 80 + "\n\n")

            # ---- 位姿信息 ----
            f.write("[起点信息]\n")
            f.write("start_x: %.6f\n"       % start_pose["x"])
            f.write("start_y: %.6f\n"       % start_pose["y"])
            f.write("start_yaw: %.6f\n"     % start_pose["yaw"])
            f.write("start_frame: %s\n\n"   % start_pose["frame_id"])

            f.write("[目标点信息]\n")
            f.write("goal_x: %.6f\n"        % goal_pose["x"])
            f.write("goal_y: %.6f\n"        % goal_pose["y"])
            f.write("goal_yaw: %.6f\n"      % goal_pose["yaw"])
            f.write("goal_frame: %s\n\n"    % goal_pose["frame_id"])

            if end_pose is not None:
                f.write("[结束位姿信息]\n")
                f.write("end_x: %.6f\n"     % end_pose["x"])
                f.write("end_y: %.6f\n"     % end_pose["y"])
                f.write("end_yaw: %.6f\n"   % end_pose["yaw"])
                f.write("end_frame: %s\n\n" % end_pose["frame_id"])

            # ---- 综合指标 ----
            # 注: goal_reach_distance 为辅助分析指标, 不参与 success 判定.
            f.write("[综合指标]\n")
            f.write("success:                 %s\n"     % ("1" if success else "0"))
            f.write("total_path_length(m):    %.6f\n"   % total_path_length)
            f.write("straight_start_to_goal:  %.6f\n"   % straight_sg)
            f.write("straight_start_to_end:   %.6f\n"   % straight_se)
            f.write("goal_reach_distance(m):  %s\n"     % (
                "inf" if math.isinf(goal_reach_dist) else "%.6f" % goal_reach_dist))
            f.write("path_efficiency:         %.6f\n"   % path_efficiency)
            f.write("avg_linear_speed(m/s):   %.6f\n\n" % avg_linear_speed)

            # ---- 速度统计 ----
            f.write("[速度统计信息]\n")
            f.write("sample_count:  %d\n"     % len(samples))
            f.write("avg_linear_x:  %.6f\n"   % avg_v)
            f.write("avg_angular_z: %.6f\n"   % avg_w)
            f.write("max_linear_x:  %.6f\n"   % max_v)
            f.write("min_linear_x:  %.6f\n"   % min_v)
            f.write("max_angular_z: %.6f\n"   % max_w)
            f.write("min_angular_z: %.6f\n\n" % min_w)

            # ---- 速度序列 ----
            f.write("[速度信息]\n")
            f.write("index,t,linear_x,angular_z\n")
            for i, s in enumerate(samples):
                f.write("%d,%.6f,%.6f,%.6f\n" % (i, s["t"], s["v"], s["w"]))
            f.write("\n")

            # ---- 轨迹序列 ----
            # alpha_z[i] = (gz[i]-gz[i-1]) / (t[i]-t[i-1]); 首样本为 0.
            f.write("[轨迹信息]\n")
            f.write("index,t,x,y,yaw,linear_x,angular_z,ax,ay,az,gx,gy,gz,alpha_z\n")
            for i, s in enumerate(samples):
                if i == 0:
                    alpha_z = 0.0
                else:
                    dt = s["t"] - samples[i - 1]["t"]
                    if dt > 1e-9:
                        alpha_z = (s["gz"] - samples[i - 1]["gz"]) / dt
                    else:
                        alpha_z = 0.0
                f.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" %
                        (i, s["t"], s["x"], s["y"], s["yaw"], s["v"], s["w"],
                         s["ax"], s["ay"], s["az"], s["gx"], s["gy"], s["gz"], alpha_z))

        os.rename(tmp_path, filepath)

        rospy.loginfo("实验记录已保存: %s | 成功=%s, 路径长=%.2fm, 效率=%.2f",
                      filepath,
                      "YES" if success else "NO",
                      total_path_length,
                      path_efficiency)


if __name__ == "__main__":
    try:
        node = ExperimentLoggerWithPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
