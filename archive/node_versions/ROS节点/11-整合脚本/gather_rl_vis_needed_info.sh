#!/usr/bin/env bash
set -u

OUT_FILE="$HOME/形成_补充信息.txt"
TMP_DIR="/tmp/rl_vis_needed_$$"
mkdir -p "$TMP_DIR"

# 如有需要，请取消注释并改成你的实际环境
# source ~/.bashrc
# source /opt/ros/melodic/setup.bash
# source ~/catkin_ws/devel/setup.bash

exec > >(tee "$OUT_FILE") 2>&1

echo "========================================"
echo "RL 真正输入181维可视化 - 补充信息采集"
echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "输出文件: $OUT_FILE"
echo "主机: $(hostname)"
echo "用户: $(whoami)"
echo "========================================"
echo

section() {
  echo
  echo "--------------------------------------------------"
  echo "$1"
  echo "--------------------------------------------------"
}

run_cmd() {
  local title="$1"
  shift
  echo "[命令] $*"
  echo "[说明] $title"
  "$@"
  local rc=$?
  echo "[返回码] $rc"
  echo
}

run_timeout_cmd() {
  local title="$1"
  local sec="$2"
  shift 2
  echo "[命令] timeout ${sec}s $*"
  echo "[说明] $title"
  timeout "${sec}s" "$@"
  local rc=$?
  echo "[返回码] $rc"
  if [ "$rc" -eq 124 ]; then
    echo "[提示] 该命令因超时结束，这对 tf_echo 属于正常现象。"
  fi
  echo
}

section "一、基础存在性快速确认"
run_cmd "查看关键 TF / 参数相关节点与话题是否存在" bash -lc 'echo "[TF frames rough check]"; rostopic list | grep -E "^/tf$|^/tf_static$" || true; echo; echo "[Key topics]"; rostopic list | grep -E "^/scan_aligned$|^/rl/debug/obs0_181$|^/rl/lidar_543$|^/rl/lidar_181_stamp$" || true'

section "二、机器人主车体 frame 关系"
run_timeout_cmd "查看 odom 到 base_footprint 的 TF" 3 rosrun tf tf_echo odom base_footprint
run_timeout_cmd "查看 base_footprint 到 car_base_link 的 TF" 3 rosrun tf tf_echo base_footprint car_base_link

section "三、雷达 frame 与方向校正 frame 关系"
run_timeout_cmd "查看 base_footprint 到 laser_link 的 TF" 3 rosrun tf tf_echo base_footprint laser_link
run_timeout_cmd "查看 base_footprint 到 laser_link_aligned 的 TF" 3 rosrun tf tf_echo base_footprint laser_link_aligned

section "四、自动列出当前 TF 树中与 base / laser 相关的 frame 名称"
run_cmd "从 tf 树报错文本中提取候选 frame（若有）" bash -lc 'python - <<"PY"
import subprocess, re
cmd = ["bash", "-lc", "timeout 2s rosrun tf tf_echo map unknown_frame 2>&1 || true"]
out = subprocess.check_output(cmd, text=True)
frames = re.findall(r"Frame\s+([^\s]+)\s+exists\s+with\s+parent\s+([^\.]+)", out)
if not frames:
    print("未能自动提取 frame 列表，可忽略。")
else:
    print("提取到的 frame 列表:")
    for child, parent in frames:
        if any(k in child for k in ["base", "laser", "odom", "map", "car"]) or any(k in parent for k in ["base", "laser", "odom", "map", "car"]):
            print("  {} <- {}".format(child, parent.strip()))
PY'

section "五、参数命名空间探测"
run_cmd "列出包含关键参数名的参数路径" bash -lc 'rosparam list | grep -E "max_distance|fov_min_deg|fov_max_deg|reverse_output|enable_repair_stabilize|output_frame_id" || true'

section "六、节点命名空间探测"
run_cmd "列出可能相关的节点名" bash -lc 'rosnode list | grep -E "laser|lidar|rl|stack|obs|switch" || true'

section "七、补充：关键消息头与真值源再确认"
run_cmd "查看 /scan_aligned/header" rostopic echo -n 1 /scan_aligned/header
run_cmd "查看 /rl/lidar_181_stamp" rostopic echo -n 1 /rl/lidar_181_stamp
run_cmd "查看 /rl/debug/obs0_181 的消息类型" rostopic type /rl/debug/obs0_181
run_cmd "查看 /rl/lidar_543 的消息类型" rostopic type /rl/lidar_543

section "八、当前阶段结论占位"
echo "1) 若 base_footprint->laser_link 有效，而 base_footprint->laser_link_aligned 无效，则说明 laser_link_aligned 不是现有 TF frame。"
echo "2) 这种情况下，可视化节点不应直接依赖 laser_link_aligned 作为 TF 挂接坐标系，除非额外发布该 TF。"
echo "3) 真正推荐用于 RL 当前真正吃进去的 181 维主输入话题：/rl/debug/obs0_181。"
echo "4) 若运行时参数没有被 launch 覆盖，则可按代码默认值处理：max_distance=16.0，FOV=[-135,135]，reverse_output=False，enable_repair_stabilize=True。"
echo

echo "========================================"
echo "采集完成。结果已保存到: $OUT_FILE"
echo "========================================"

rm -rf "$TMP_DIR"
