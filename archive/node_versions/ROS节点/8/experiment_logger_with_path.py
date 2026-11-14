#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import math
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ExperimentLoggerWithPath(object):
    def __init__(self):
        rospy.init_node("experiment_logger_with_path", anonymous=False)

        # ===== 话题参数 =====
        self.active_mode_topic = rospy.get_param("~active_mode_topic", "/cmd_vel_switch/active_mode")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.pose_topic = rospy.get_param("~pose_topic", "/laserpose")
        self.speed_topic = rospy.get_param("~speed_topic", "/zeus_speed")
        self.dwa_status_topic = rospy.get_param("~dwa_status_topic", "/move_base/status")
        self.rl_status_topic = rospy.get_param("~rl_status_topic", "/rl_local_goal_status")
        self.path_topic = rospy.get_param("~path_topic", "/trajectory")

        # ===== 保存与采样参数 =====
        self.save_dir = os.path.expanduser(rospy.get_param("~save_dir", "/home/ros/save_dir"))
        self.record_rate_hz = float(rospy.get_param("~record_rate_hz", 10.0))
        self.min_traj_dist = float(rospy.get_param("~min_traj_dist", 0.02))
        self.max_path_points = int(rospy.get_param("~max_path_points", 5000))
        self.path_clear_delay_sec = float(rospy.get_param("~path_clear_delay_sec", 5.0))

        # ===== 模式 =====
        self.valid_modes = set(["rl", "dwa"])

        # ===== 创建目录 =====
        self.rl_save_dir = os.path.join(self.save_dir, "rl")
        self.dwa_save_dir = os.path.join(self.save_dir, "dwa")
        for d in [self.save_dir, self.rl_save_dir, self.dwa_save_dir]:
            if not os.path.exists(d):
                os.makedirs(d)

        # ===== 当前系统状态 =====
        self.active_mode = ""
        self.latest_goal = None
        self.latest_pose = None
        self.latest_speed = None

        # RL / DWA 状态
        self.rl_goal_status = "none"
        self.rl_goal_started = False

        self.dwa_goal_started = False
        self.dwa_terminal_status = None
        self.dwa_terminal_text = ""

        # ===== 实验记录状态 =====
        self.recording = False
        self.current_record = None
        self.current_mode = ""
        self.experiment_id = self.get_next_experiment_id()
        self.pending_finish_reason = None

        # ===== RViz 轨迹 =====
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.path_msg = Path()
        self.last_path_x = None
        self.last_path_y = None
        self.path_active = False
        self.path_clear_deadline = None

        # ===== 订阅 =====
        rospy.Subscriber(self.active_mode_topic, String, self.cb_active_mode, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.cb_goal, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.cb_pose, queue_size=50)
        rospy.Subscriber(self.speed_topic, Twist, self.cb_speed, queue_size=50)
        rospy.Subscriber(self.dwa_status_topic, GoalStatusArray, self.cb_dwa_status, queue_size=20)
        rospy.Subscriber(self.rl_status_topic, String, self.cb_rl_status, queue_size=20)

        # ===== 定时器 =====
        period = 1.0 / self.record_rate_hz if self.record_rate_hz > 0.0 else 0.1
        rospy.Timer(rospy.Duration(period), self.cb_timer)

        rospy.loginfo("实验记录+轨迹发布节点已启动")
        rospy.loginfo("active_mode_topic : %s", self.active_mode_topic)
        rospy.loginfo("goal_topic        : %s", self.goal_topic)
        rospy.loginfo("pose_topic        : %s", self.pose_topic)
        rospy.loginfo("speed_topic       : %s", self.speed_topic)
        rospy.loginfo("dwa_status_topic  : %s", self.dwa_status_topic)
        rospy.loginfo("rl_status_topic   : %s", self.rl_status_topic)
        rospy.loginfo("path_topic        : %s", self.path_topic)
        rospy.loginfo("save_dir          : %s", self.save_dir)
        rospy.loginfo("rl_save_dir       : %s", self.rl_save_dir)
        rospy.loginfo("dwa_save_dir      : %s", self.dwa_save_dir)
        rospy.loginfo("path_clear_delay  : %.2f s", self.path_clear_delay_sec)

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
                    except Exception:
                        pass
        return max(existing) + 1 if existing else 1

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def clear_path(self, frame_id="map"):
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = rospy.Time.now()
        self.last_path_x = None
        self.last_path_y = None
        self.path_pub.publish(self.path_msg)

    def schedule_clear_path(self):
        self.path_clear_deadline = rospy.Time.now().to_sec() + self.path_clear_delay_sec
        rospy.loginfo("轨迹将在 %.2f 秒后自动清空", self.path_clear_delay_sec)

    def append_path_point(self, pose_dict):
        x = pose_dict["x"]
        y = pose_dict["y"]
        frame_id = pose_dict["frame_id"]

        if not self.path_msg.header.frame_id:
            self.path_msg.header.frame_id = frame_id

        if self.last_path_x is None or self.last_path_y is None:
            need_add = True
        else:
            need_add = self.distance(x, y, self.last_path_x, self.last_path_y) >= self.min_traj_dist

        if not need_add:
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path_msg)
            return

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = frame_id
        ps.pose.position.x = pose_dict["x"]
        ps.pose.position.y = pose_dict["y"]
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = pose_dict["qx"]
        ps.pose.orientation.y = pose_dict["qy"]
        ps.pose.orientation.z = pose_dict["qz"]
        ps.pose.orientation.w = pose_dict["qw"]

        self.path_msg.poses.append(ps)

        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses.pop(0)

        self.last_path_x = x
        self.last_path_y = y
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)

    def get_mode_save_dir(self, mode):
        if mode == "rl":
            return self.rl_save_dir
        elif mode == "dwa":
            return self.dwa_save_dir
        return self.save_dir

    def cb_active_mode(self, msg):
        new_mode = msg.data.strip().lower()
        old_mode = self.active_mode
        self.active_mode = new_mode

        if new_mode != old_mode:
            rospy.loginfo("实际控制模式切换: %s -> %s",
                          old_mode if old_mode else "none", new_mode)

        if self.recording and new_mode != self.current_mode:
            self.pending_finish_reason = "mode_switched_to_%s" % new_mode

    def cb_goal(self, msg):
        self.latest_goal = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw": quat_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ),
            "frame_id": msg.header.frame_id if msg.header.frame_id else "map",
            "stamp": msg.header.stamp.to_sec()
        }

        self.rl_goal_status = "none"
        self.rl_goal_started = False

        self.dwa_goal_started = False
        self.dwa_terminal_status = None
        self.dwa_terminal_text = ""

        if self.recording:
            self.pending_finish_reason = "new_goal_received"

        self.path_clear_deadline = None
        self.clear_path(frame_id=self.latest_goal["frame_id"])
        self.path_active = True

        rospy.loginfo("收到新目标点: x=%.3f, y=%.3f, frame=%s",
                      self.latest_goal["x"], self.latest_goal["y"], self.latest_goal["frame_id"])

    def cb_pose(self, msg):
        self.latest_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w,
            "yaw": quat_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ),
            "frame_id": msg.header.frame_id if msg.header.frame_id else "map",
            "stamp": msg.header.stamp.to_sec()
        }

        if self.recording and self.path_active and self.current_mode in self.valid_modes:
            self.append_path_point(self.latest_pose)
        else:
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path_msg)

    def cb_speed(self, msg):
        self.latest_speed = {
            "v": msg.linear.x,
            "w": msg.angular.z,
            "stamp": rospy.Time.now().to_sec()
        }

    def cb_rl_status(self, msg):
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
        if not msg.status_list:
            return

        saw_active = False
        terminal_status = None
        terminal_text = ""

        for st in msg.status_list:
            if st.status == 1:
                saw_active = True
            elif st.status in (2, 3, 4, 5, 8, 9):
                terminal_status = st.status
                terminal_text = st.text

        if saw_active:
            self.dwa_goal_started = True

        if terminal_status is not None and self.dwa_goal_started:
            self.dwa_terminal_status = terminal_status
            self.dwa_terminal_text = terminal_text

            if self.recording and self.current_mode == "dwa":
                if terminal_status == 3:
                    self.pending_finish_reason = "dwa_goal_reached"
                else:
                    self.pending_finish_reason = "dwa_goal_terminated_status_%d" % terminal_status

    def cb_timer(self, event):
        now_sec = rospy.Time.now().to_sec()

        if (not self.recording) and (self.path_clear_deadline is not None):
            if now_sec >= self.path_clear_deadline:
                frame_id = self.path_msg.header.frame_id if self.path_msg.header.frame_id else "map"
                self.clear_path(frame_id=frame_id)
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
        if self.active_mode not in self.valid_modes:
            return
        if self.latest_goal is None:
            return
        if self.latest_pose is None:
            return
        if self.latest_speed is None:
            return

        self.start_recording()

    def start_recording(self):
        self.current_mode = self.active_mode
        self.pending_finish_reason = None
        self.path_clear_deadline = None

        frame_id = self.latest_pose["frame_id"] if self.latest_pose else (
            self.latest_goal["frame_id"] if self.latest_goal else "map"
        )
        self.clear_path(frame_id=frame_id)
        self.path_active = True

        self.current_record = {
            "experiment_id": self.experiment_id,
            "control_mode": self.current_mode,
            "start_time": rospy.Time.now().to_sec(),
            "start_pose": dict(self.latest_pose),
            "goal_pose": dict(self.latest_goal),
            "end_time": None,
            "end_pose": None,
            "finish_reason": "",
            "rl_goal_status": "",
            "dwa_status_code": None,
            "dwa_status_text": "",
            "samples": []
        }

        self.recording = True
        self.append_sample()

        rospy.loginfo("开始记录实验 #%04d, mode=%s",
                      self.experiment_id, self.current_mode)

    def append_sample(self):
        if self.current_record is None:
            return
        if self.latest_pose is None or self.latest_speed is None:
            return

        s = {
            "t": rospy.Time.now().to_sec(),
            "x": self.latest_pose["x"],
            "y": self.latest_pose["y"],
            "yaw": self.latest_pose["yaw"],
            "v": self.latest_speed["v"],
            "w": self.latest_speed["w"]
        }
        self.current_record["samples"].append(s)

    def finish_recording(self, reason):
        if not self.recording or self.current_record is None:
            return

        self.current_record["end_time"] = rospy.Time.now().to_sec()
        self.current_record["end_pose"] = dict(self.latest_pose) if self.latest_pose else None
        self.current_record["finish_reason"] = reason
        self.current_record["rl_goal_status"] = self.rl_goal_status
        self.current_record["dwa_status_code"] = self.dwa_terminal_status
        self.current_record["dwa_status_text"] = self.dwa_terminal_text

        self.save_record_to_txt(self.current_record)

        rospy.loginfo("实验 #%04d 已结束, mode=%s, reason=%s, sample_count=%d",
                      self.current_record["experiment_id"],
                      self.current_record["control_mode"],
                      reason,
                      len(self.current_record["samples"]))

        self.path_active = False
        self.schedule_clear_path()

        self.recording = False
        self.current_record = None
        self.current_mode = ""
        self.pending_finish_reason = None
        self.experiment_id += 1

    def save_record_to_txt(self, record):
        exp_id = record["experiment_id"]
        mode = record["control_mode"]

        save_root = self.get_mode_save_dir(mode)
        if not os.path.exists(save_root):
            os.makedirs(save_root)

        filename = "exp_%04d_%s.txt" % (exp_id, mode)
        filepath = os.path.join(save_root, filename)

        samples = record["samples"]
        start_pose = record["start_pose"]
        goal_pose = record["goal_pose"]
        end_pose = record["end_pose"]

        duration = 0.0
        if record["end_time"] is not None and record["start_time"] is not None:
            duration = record["end_time"] - record["start_time"]

        v_list = [s["v"] for s in samples]
        w_list = [s["w"] for s in samples]

        avg_v = sum(v_list) / len(v_list) if v_list else 0.0
        avg_w = sum(w_list) / len(w_list) if w_list else 0.0
        max_v = max(v_list) if v_list else 0.0
        min_v = min(v_list) if v_list else 0.0
        max_w = max(w_list) if w_list else 0.0
        min_w = min(w_list) if w_list else 0.0

        with open(filepath, "w") as f:
            f.write("=" * 80 + "\n")
            f.write("实验编号: %04d\n" % exp_id)
            f.write("控制模式: %s\n" % mode)
            f.write("记录起始时间戳: %.6f\n" % record["start_time"])
            f.write("记录终止时间戳: %.6f\n" % record["end_time"])
            f.write("实验持续时长(s): %.6f\n" % duration)
            f.write("结束原因: %s\n" % record["finish_reason"])
            f.write("RL状态话题最终值: %s\n" % record["rl_goal_status"])
            f.write("DWA状态码: %s\n" % str(record["dwa_status_code"]))
            f.write("DWA状态文本: %s\n" % record["dwa_status_text"])
            f.write("=" * 80 + "\n\n")

            f.write("[起点信息]\n")
            f.write("start_x: %.6f\n" % start_pose["x"])
            f.write("start_y: %.6f\n" % start_pose["y"])
            f.write("start_yaw: %.6f\n" % start_pose["yaw"])
            f.write("start_frame: %s\n\n" % start_pose["frame_id"])

            f.write("[目标点信息]\n")
            f.write("goal_x: %.6f\n" % goal_pose["x"])
            f.write("goal_y: %.6f\n" % goal_pose["y"])
            f.write("goal_yaw: %.6f\n" % goal_pose["yaw"])
            f.write("goal_frame: %s\n\n" % goal_pose["frame_id"])

            if end_pose is not None:
                f.write("[结束位姿信息]\n")
                f.write("end_x: %.6f\n" % end_pose["x"])
                f.write("end_y: %.6f\n" % end_pose["y"])
                f.write("end_yaw: %.6f\n" % end_pose["yaw"])
                f.write("end_frame: %s\n\n" % end_pose["frame_id"])

            f.write("[速度统计信息]\n")
            f.write("sample_count: %d\n" % len(samples))
            f.write("avg_linear_x: %.6f\n" % avg_v)
            f.write("avg_angular_z: %.6f\n" % avg_w)
            f.write("max_linear_x: %.6f\n" % max_v)
            f.write("min_linear_x: %.6f\n" % min_v)
            f.write("max_angular_z: %.6f\n" % max_w)
            f.write("min_angular_z: %.6f\n\n" % min_w)

            f.write("[速度信息]\n")
            f.write("index,t,linear_x,angular_z\n")
            for i, s in enumerate(samples):
                f.write("%d,%.6f,%.6f,%.6f\n" % (i, s["t"], s["v"], s["w"]))
            f.write("\n")

            f.write("[轨迹信息]\n")
            f.write("index,t,x,y,yaw,linear_x,angular_z\n")
            for i, s in enumerate(samples):
                f.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" %
                        (i, s["t"], s["x"], s["y"], s["yaw"], s["v"], s["w"]))

        rospy.loginfo("实验记录已保存: %s", filepath)


if __name__ == "__main__":
    try:
        node = ExperimentLoggerWithPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
