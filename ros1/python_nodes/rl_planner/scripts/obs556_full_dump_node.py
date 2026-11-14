#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import datetime
import threading

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String as StringMsg

OBS_DIM = 556


class Obs556FullRecorderNode(object):
    def __init__(self):
        rospy.init_node("obs556_full_recorder_node", anonymous=False)

        self.obs_topic = rospy.get_param("~obs_topic", "/rl/obs_556")
        self.goal_status_topic = rospy.get_param("~goal_status_topic", "/rl_local_goal_status")
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/save_dir/lidar")
        )
        self.file_prefix = rospy.get_param("~file_prefix", "obs556_exp")
        self.auto_start_on_tracking = bool(rospy.get_param("~auto_start_on_tracking", True))

        self.lock = threading.RLock()

        self.latest_goal_status = "none"
        self.prev_goal_status = "none"

        self.recording = False
        self.frame_count = 0
        self.exp_count = 0
        self.fp = None
        self.current_file_path = None
        self.current_start_wall_time = None
        self.current_start_ros_time = None

        self._ensure_dir(self.output_dir)

        self.sub_obs = rospy.Subscriber(
            self.obs_topic,
            Float32MultiArray,
            self.cb_obs,
            queue_size=50
        )
        self.sub_goal_status = rospy.Subscriber(
            self.goal_status_topic,
            StringMsg,
            self.cb_goal_status,
            queue_size=50
        )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[obs556_recorder] subscribe obs topic        : %s", self.obs_topic)
        rospy.loginfo("[obs556_recorder] subscribe goal_status topic: %s", self.goal_status_topic)
        rospy.loginfo("[obs556_recorder] output dir                 : %s", self.output_dir)
        rospy.loginfo("[obs556_recorder] file prefix                : %s", self.file_prefix)
        rospy.loginfo("[obs556_recorder] waiting for experiment start...")

    def _ensure_dir(self, path):
        if not os.path.isdir(path):
            try:
                os.makedirs(path)
            except OSError:
                if not os.path.isdir(path):
                    raise

    def _now(self):
        return datetime.datetime.now()

    def _make_file_path(self):
        now = self._now()
        timestamp = now.strftime("%Y%m%d_%H%M%S_%f")
        filename = "{}_{}.txt".format(self.file_prefix, timestamp)
        return os.path.join(self.output_dir, filename)

    def _start_new_record(self, trigger_status):
        if self.recording:
            self._finish_record("forced_restart")

        self.exp_count += 1
        self.frame_count = 0
        self.current_start_wall_time = self._now()
        self.current_start_ros_time = rospy.Time.now().to_sec()
        self.current_file_path = self._make_file_path()
        self.fp = open(self.current_file_path, "w")

        self.fp.write("=" * 80 + "\n")
        self.fp.write("obs_556 Full Record For One Experiment\n")
        self.fp.write("experiment_index  : {}\n".format(self.exp_count))
        self.fp.write("start_wall_time   : {}\n".format(
            self.current_start_wall_time.strftime("%Y-%m-%d %H:%M:%S.%f")))
        self.fp.write("start_ros_time    : {:.6f}\n".format(self.current_start_ros_time))
        self.fp.write("start_trigger     : {}\n".format(trigger_status))
        self.fp.write("obs_topic         : {}\n".format(self.obs_topic))
        self.fp.write("goal_status_topic : {}\n".format(self.goal_status_topic))
        self.fp.write("output_file       : {}\n".format(self.current_file_path))
        self.fp.write("=" * 80 + "\n\n")
        self.fp.flush()

        self.recording = True
        rospy.loginfo(
            "[obs556_recorder] start experiment #%d -> %s",
            self.exp_count,
            self.current_file_path
        )

    def _finish_record(self, reason):
        if not self.recording:
            return

        stop_wall_time = self._now()
        stop_ros_time = rospy.Time.now().to_sec()
        duration = 0.0
        if self.current_start_ros_time is not None:
            duration = stop_ros_time - self.current_start_ros_time

        try:
            if self.fp is not None:
                self.fp.write("\n" + "=" * 80 + "\n")
                self.fp.write("record_stop_reason : {}\n".format(reason))
                self.fp.write("stop_wall_time     : {}\n".format(
                    stop_wall_time.strftime("%Y-%m-%d %H:%M:%S.%f")))
                self.fp.write("stop_ros_time      : {:.6f}\n".format(stop_ros_time))
                self.fp.write("duration_sec       : {:.6f}\n".format(duration))
                self.fp.write("total_frames       : {}\n".format(self.frame_count))
                self.fp.write("last_goal_status   : {}\n".format(self.latest_goal_status))
                self.fp.write("=" * 80 + "\n")
                self.fp.flush()
                self.fp.close()
        except Exception as e:
            rospy.logerr("[obs556_recorder] 写停止信息失败: %s", str(e))
        finally:
            rospy.loginfo(
                "[obs556_recorder] finish experiment #%d, reason=%s, frames=%d, file=%s",
                self.exp_count,
                reason,
                self.frame_count,
                self.current_file_path
            )
            self.fp = None
            self.recording = False
            self.frame_count = 0
            self.current_file_path = None
            self.current_start_wall_time = None
            self.current_start_ros_time = None

    def cb_goal_status(self, msg):
        status = msg.data.strip()
        if not status:
            status = "none"

        with self.lock:
            prev_status = self.latest_goal_status
            self.prev_goal_status = prev_status
            self.latest_goal_status = status

            if status == "goal_received":
                # 新目标到来，若上一实验尚未正常结束，则先收尾，再开启新实验
                if self.recording and prev_status not in ("goal_received", "goal_tracking"):
                    self._finish_record("new_goal_received")
                if not self.recording:
                    self._start_new_record("goal_received")
                return

            if status == "goal_tracking":
                # 某些系统可能不会显式先发 goal_received，因此允许在 tracking 时自动起录
                if self.auto_start_on_tracking and not self.recording:
                    self._start_new_record("goal_tracking")
                return

            if status == "goal_reached":
                self._finish_record("goal_reached")
                return

            if status in ("goal_failed", "goal_aborted", "goal_cancelled", "goal_lost"):
                self._finish_record(status)
                return

    def cb_obs(self, msg):
        data = list(msg.data)
        if len(data) != OBS_DIM:
            rospy.logwarn(
                "[obs556_recorder] 收到维度=%d，期望=%d，忽略本帧",
                len(data), OBS_DIM
            )
            return

        with self.lock:
            if not self.recording or self.fp is None:
                return

            self.frame_count += 1
            wall_time = self._now().strftime("%Y-%m-%d %H:%M:%S.%f")
            ros_time = rospy.Time.now().to_sec()

            try:
                self.fp.write("-" * 80 + "\n")
                self.fp.write("[Frame #{:d}]\n".format(self.frame_count))
                self.fp.write("wall_time   : {}\n".format(wall_time))
                self.fp.write("ros_time    : {:.6f}\n".format(ros_time))
                self.fp.write("goal_status : {}\n".format(self.latest_goal_status))
                self.fp.write("obs_dim     : {}\n".format(len(data)))
                self.fp.write("\n")

                for i, v in enumerate(data):
                    self.fp.write("obs[{:<3d}] = {:.10f}\n".format(i, float(v)))

                self.fp.write("\n")
                self.fp.flush()

                if self.frame_count % 10 == 0:
                    rospy.loginfo(
                        "[obs556_recorder] experiment #%d 已记录 %d 帧 obs_556",
                        self.exp_count,
                        self.frame_count
                    )

            except Exception as e:
                rospy.logerr("[obs556_recorder] 写文件失败: %s", str(e))

    def on_shutdown(self):
        with self.lock:
            if self.recording:
                self._finish_record("node_shutdown")
            else:
                try:
                    if self.fp is not None:
                        self.fp.flush()
                        self.fp.close()
                        self.fp = None
                except Exception:
                    pass

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Obs556FullRecorderNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
