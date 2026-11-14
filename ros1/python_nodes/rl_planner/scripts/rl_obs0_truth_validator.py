#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function

import collections
import math
import os
import threading

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Header


class RLObs0TruthValidator(object):
    def __init__(self):
        rospy.init_node('rl_obs0_truth_validator', anonymous=False)

        # Topics
        self.obs0_topic = rospy.get_param('~obs0_topic', '/rl/debug/obs0_181')
        self.lidar543_topic = rospy.get_param('~lidar543_topic', '/rl/lidar_543')
        self.stamps_topic = rospy.get_param('~stamps_topic', '/rl/lidar_543_stamps')
        self.header_topic = rospy.get_param('~header_topic', '/rl/lidar_181_stamp')

        # Expected dimensions
        self.obs_dim = int(rospy.get_param('~obs_dim', 181))
        self.stack_dim = int(rospy.get_param('~stack_dim', 543))

        # Matching windows
        self.sync_tolerance_sec = float(rospy.get_param('~sync_tolerance_sec', 0.20))
        self.source_stamp_tolerance_sec = float(rospy.get_param('~source_stamp_tolerance_sec', 0.20))
        self.cache_size = int(rospy.get_param('~cache_size', 50))

        # Comparison tolerance
        self.equal_eps = float(rospy.get_param('~equal_eps', 1e-6))
        self.warn_eps = float(rospy.get_param('~warn_eps', 1e-4))

        # Logging / output
        self.log_every = int(rospy.get_param('~log_every', 10))
        self.out_path = rospy.get_param('~out_path', os.path.expanduser('~/rl_obs0_truth_check.txt'))

        self.lock = threading.Lock()
        self.seq = 0
        self.pass_count = 0
        self.fail_count = 0
        self.miss_543_count = 0
        self.miss_stamps_count = 0
        self.miss_header_count = 0
        self.nonmono_count = 0
        self.frame_mismatch_count = 0
        self.source_stamp_mismatch_count = 0

        self.cache_543 = collections.deque(maxlen=self.cache_size)     # (recv_time, list[543])
        self.cache_stamps = collections.deque(maxlen=self.cache_size)  # (recv_time, list[3])
        self.cache_header = collections.deque(maxlen=self.cache_size)  # (recv_time, Header)

        out_dir = os.path.dirname(self.out_path)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        self.fp = open(self.out_path, 'a')
        if os.path.getsize(self.out_path) == 0:
            self.fp.write('seq,recv_time,match,max_abs_diff,mismatch_count,stamp_monotonic,frame_id,header_delta_sec,have_543,have_stamps,have_header\n')
            self.fp.flush()

        rospy.Subscriber(self.lidar543_topic, Float32MultiArray, self.cb_543, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.stamps_topic, Float64MultiArray, self.cb_stamps, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.header_topic, Header, self.cb_header, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.obs0_topic, Float32MultiArray, self.cb_obs0, queue_size=100, tcp_nodelay=True)

        rospy.on_shutdown(self.on_shutdown)

        rospy.logwarn('======================================')
        rospy.logwarn('[rl_obs0_truth_validator] started')
        rospy.logwarn('  obs0_topic                 = %s', self.obs0_topic)
        rospy.logwarn('  lidar543_topic             = %s', self.lidar543_topic)
        rospy.logwarn('  stamps_topic               = %s', self.stamps_topic)
        rospy.logwarn('  header_topic               = %s', self.header_topic)
        rospy.logwarn('  obs_dim / stack_dim        = %d / %d', self.obs_dim, self.stack_dim)
        rospy.logwarn('  sync_tolerance_sec         = %.3f', self.sync_tolerance_sec)
        rospy.logwarn('  source_stamp_tolerance_sec = %.3f', self.source_stamp_tolerance_sec)
        rospy.logwarn('  equal_eps / warn_eps       = %.2e / %.2e', self.equal_eps, self.warn_eps)
        rospy.logwarn('  out_path                   = %s', self.out_path)
        rospy.logwarn('======================================')

    def _append_csv(self, row):
        self.fp.write(row + '\n')
        self.fp.flush()

    def _closest(self, cache, now_sec):
        if not cache:
            return None, None
        best = None
        best_dt = None
        for item in cache:
            dt = abs(item[0] - now_sec)
            if best is None or dt < best_dt:
                best = item
                best_dt = dt
        return best, best_dt

    def cb_543(self, msg):
        data = list(msg.data)
        if len(data) != self.stack_dim:
            rospy.logwarn_throttle(1.0, '[validator] /rl/lidar_543 length=%d != %d', len(data), self.stack_dim)
            return
        with self.lock:
            self.cache_543.append((rospy.Time.now().to_sec(), data))

    def cb_stamps(self, msg):
        data = list(msg.data)
        if len(data) < 3:
            rospy.logwarn_throttle(1.0, '[validator] /rl/lidar_543_stamps length=%d < 3', len(data))
            return
        with self.lock:
            self.cache_stamps.append((rospy.Time.now().to_sec(), data[:3]))

    def cb_header(self, msg):
        with self.lock:
            self.cache_header.append((rospy.Time.now().to_sec(), msg))

    def cb_obs0(self, msg):
        obs = list(msg.data)
        if len(obs) != self.obs_dim:
            rospy.logwarn_throttle(1.0, '[validator] /rl/debug/obs0_181 length=%d != %d', len(obs), self.obs_dim)
            return

        now_sec = rospy.Time.now().to_sec()
        with self.lock:
            self.seq += 1
            seq = self.seq

            item543, dt543 = self._closest(self.cache_543, now_sec)
            itemst, dtst = self._closest(self.cache_stamps, now_sec)
            itemhd, dthd = self._closest(self.cache_header, now_sec)

        have_543 = item543 is not None and dt543 <= self.sync_tolerance_sec
        have_stamps = itemst is not None and dtst <= self.sync_tolerance_sec
        have_header = itemhd is not None and dthd <= self.sync_tolerance_sec

        if not have_543:
            self.miss_543_count += 1
        if not have_stamps:
            self.miss_stamps_count += 1
        if not have_header:
            self.miss_header_count += 1

        max_abs_diff = float('nan')
        mismatch_count = -1
        match = False

        if have_543:
            lidar543 = item543[1]
            t0 = lidar543[:self.obs_dim]
            diffs = [abs(float(a) - float(b)) for a, b in zip(obs, t0)]
            max_abs_diff = max(diffs) if diffs else 0.0
            mismatch_count = sum(1 for d in diffs if d > self.equal_eps)
            match = (mismatch_count == 0)
            if match:
                self.pass_count += 1
            else:
                self.fail_count += 1

        stamp_monotonic = False
        t0_src = None
        if have_stamps:
            stamps = itemst[1]
            stamp_monotonic = (stamps[0] >= stamps[1] >= stamps[2])
            t0_src = stamps[0]
            if not stamp_monotonic:
                self.nonmono_count += 1

        frame_id = ''
        header_delta_sec = float('nan')
        if have_header:
            header_msg = itemhd[1]
            frame_id = header_msg.frame_id
            if frame_id != 'laser_link_aligned':
                self.frame_mismatch_count += 1
            if t0_src is not None:
                hdr_sec = header_msg.stamp.to_sec()
                header_delta_sec = abs(hdr_sec - t0_src)
                if header_delta_sec > self.source_stamp_tolerance_sec:
                    self.source_stamp_mismatch_count += 1

        row = '{seq},{recv:.6f},{match},{mad},{mc},{mono},{fid},{hds},{h543},{hst},{hhd}'.format(
            seq=seq,
            recv=now_sec,
            match=('True' if match else 'False'),
            mad=('nan' if math.isnan(max_abs_diff) else '%.9f' % max_abs_diff),
            mc=mismatch_count,
            mono=('True' if stamp_monotonic else 'False'),
            fid=(frame_id if frame_id else 'None'),
            hds=('nan' if math.isnan(header_delta_sec) else '%.6f' % header_delta_sec),
            h543=('True' if have_543 else 'False'),
            hst=('True' if have_stamps else 'False'),
            hhd=('True' if have_header else 'False'),
        )
        self._append_csv(row)

        if seq % max(1, self.log_every) == 1:
            mad_str = 'nan' if math.isnan(max_abs_diff) else '%.9f' % max_abs_diff
            hds_str = 'nan' if math.isnan(header_delta_sec) else '%.6f' % header_delta_sec
            level = rospy.logwarn if (have_543 and not match) or (have_stamps and not stamp_monotonic) else rospy.loginfo
            level(
                '[validator] seq=%d | have_543=%s have_stamps=%s have_header=%s | '
                'match=%s max_abs_diff=%s mismatch_count=%d | '
                'stamp_monotonic=%s | frame_id=%s | |header.t0 - stamps[0]|=%s | '
                'pass=%d fail=%d miss543=%d missst=%d misshd=%d nonmono=%d frame_bad=%d srcstamp_bad=%d',
                seq, have_543, have_stamps, have_header,
                match, mad_str, mismatch_count,
                stamp_monotonic, (frame_id if frame_id else 'None'), hds_str,
                self.pass_count, self.fail_count, self.miss_543_count,
                self.miss_stamps_count, self.miss_header_count,
                self.nonmono_count, self.frame_mismatch_count,
                self.source_stamp_mismatch_count,
            )

    def on_shutdown(self):
        try:
            self.fp.close()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        RLObs0TruthValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
