#!/usr/bin/env python3
import os
import csv
import json

def analyze_csv_time(path, name):
    print("=" * 60)
    print(name)
    print("文件:", path)
    if not os.path.exists(path):
        print("文件不存在")
        return

    times = []
    try:
        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if "time" in row:
                    try:
                        times.append(float(row["time"]))
                    except:
                        pass
    except Exception as e:
        print("读取失败:", e)
        return

    if len(times) < 2:
        print("有效时间戳数量不足:", len(times))
        return

    dt = times[-1] - times[0]
    hz = (len(times) - 1) / dt if dt > 0 else 0.0
    print("记录条数:", len(times))
    print("起始时间:", times[0])
    print("终止时间:", times[-1])
    print("持续时间: %.6f s" % dt)
    print("估算频率: %.3f Hz" % hz)

def analyze_jsonl_action(path, name):
    print("=" * 60)
    print(name)
    print("文件:", path)
    if not os.path.exists(path):
        print("文件不存在")
        return

    times = []
    obs_count = 0
    action_count = 0

    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except:
                    continue
                if obj.get("type") == "obs_meta":
                    obs_count += 1
                if obj.get("type") == "action_meta":
                    action_count += 1
                    if "t_action" in obj:
                        try:
                            times.append(float(obj["t_action"]))
                        except:
                            pass
    except Exception as e:
        print("读取失败:", e)
        return

    print("obs_meta 数量:", obs_count)
    print("action_meta 数量:", action_count)

    if len(times) < 2:
        print("有效 action 时间戳数量不足:", len(times))
        return

    dt = times[-1] - times[0]
    hz = (len(times) - 1) / dt if dt > 0 else 0.0
    print("action 时间戳数量:", len(times))
    print("起始时间:", times[0])
    print("终止时间:", times[-1])
    print("持续时间: %.6f s" % dt)
    print("根据 action_meta 估算模型输出频率: %.3f Hz" % hz)

analyze_csv_time("/home/ros/4.txt", "RL 动作日志 /home/ros/4.txt")
analyze_jsonl_action("/home/ros/obs_meta.jsonl", "RL 元信息日志 /home/ros/obs_meta.jsonl")
