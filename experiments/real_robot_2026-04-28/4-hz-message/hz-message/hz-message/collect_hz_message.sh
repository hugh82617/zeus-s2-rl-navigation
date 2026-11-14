#!/usr/bin/env bash
# ============================================================
# ROS 系统运行频率采集脚本
# 保存位置：~/hz-message
# 采集内容：
# 1. LiDAR 更新频率
# 2. 策略模型推理频率
# 3. 控制指令发布频率
# 4. 底盘反馈频率
# 5. 相关 ROS 参数与节点信息
# ============================================================

set -u

OUTDIR="$HOME/hz-message"
STAMP=$(date +"%Y%m%d_%H%M%S")
REPORT="$OUTDIR/hz_report_${STAMP}.txt"
TOPIC_DIR="$OUTDIR/topic_hz_${STAMP}"
PARAM_FILE="$OUTDIR/ros_params_${STAMP}.txt"
NODE_FILE="$OUTDIR/ros_nodes_${STAMP}.txt"
LOG_ANALYSIS="$OUTDIR/inference_log_analysis_${STAMP}.txt"

MEASURE_SEC=${1:-12}

mkdir -p "$OUTDIR"
mkdir -p "$TOPIC_DIR"

# 自动加载 ROS 环境
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi

if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source "$HOME/catkin_ws/devel/setup.bash"
fi

echo "============================================================" | tee "$REPORT"
echo "ROS 系统运行频率采集报告" | tee -a "$REPORT"
echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$REPORT"
echo "保存目录: $OUTDIR" | tee -a "$REPORT"
echo "单个话题测量时长: ${MEASURE_SEC}s" | tee -a "$REPORT"
echo "============================================================" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

echo "[1] ROS Master 检查" | tee -a "$REPORT"
if ! rostopic list >/dev/null 2>&1; then
    echo "错误：当前未检测到 ROS Master，请先启动 roscore 或完整机器人系统。" | tee -a "$REPORT"
    exit 1
fi
echo "ROS Master 正常。" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

echo "[2] 当前 ROS 话题列表" | tee -a "$REPORT"
rostopic list | tee "$OUTDIR/topic_list_${STAMP}.txt" >/dev/null
echo "已保存: $OUTDIR/topic_list_${STAMP}.txt" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

echo "[3] 当前 ROS 节点列表" | tee -a "$REPORT"
rosnode list | tee "$NODE_FILE" >/dev/null
echo "已保存: $NODE_FILE" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

echo "[4] 相关 ROS 参数导出" | tee -a "$REPORT"
{
    echo "===== 所有 frequency/rate/hz/controller 相关参数 ====="
    rosparam list | grep -Ei "freq|frequency|rate|hz|controller|planner" || true
    echo ""
    echo "===== RL 推理节点相关参数 ====="
    rosparam list | grep -Ei "rl|onnx|model_path|lidar_topic|speed_topic|out_cmd_topic|obs_topic|goal_topic" || true
    echo ""
    echo "===== 参数具体取值 ====="
    for p in $(rosparam list | grep -Ei "freq|frequency|rate|hz|controller|planner|rl|onnx|model_path|lidar_topic|speed_topic|out_cmd_topic|obs_topic|goal_topic"); do
        echo "----- $p -----"
        rosparam get "$p" 2>/dev/null || true
        echo ""
    done
} > "$PARAM_FILE"
echo "已保存: $PARAM_FILE" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

# 检查话题是否存在
topic_exists() {
    rostopic list | grep -Fxq "$1"
}

measure_topic_hz() {
    local topic="$1"
    local tag="$2"
    local outfile="$TOPIC_DIR/${tag}.txt"

    echo "------------------------------------------------------------" | tee -a "$REPORT"
    echo "测量对象: $tag" | tee -a "$REPORT"
    echo "话题名称: $topic" | tee -a "$REPORT"

    if topic_exists "$topic"; then
        echo "话题存在，开始测量 ${MEASURE_SEC}s ..." | tee -a "$REPORT"
        {
            echo "话题: $topic"
            echo "测量时长: ${MEASURE_SEC}s"
            echo "开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
            echo ""
            timeout "${MEASURE_SEC}s" rostopic hz "$topic"
            echo ""
            echo "结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
        } > "$outfile" 2>&1

        echo "结果文件: $outfile" | tee -a "$REPORT"
        echo "关键结果预览:" | tee -a "$REPORT"
        grep -E "average rate|std dev|max|min|window" "$outfile" | tail -n 5 | tee -a "$REPORT" || true
    else
        echo "话题不存在，跳过。" | tee -a "$REPORT"
        echo "话题不存在: $topic" > "$outfile"
    fi
    echo "" | tee -a "$REPORT"
}

echo "[5] 话题频率实际测量 rostopic hz" | tee -a "$REPORT"
echo "说明：以下频率均为 ROS 话题实际测量结果，不是配置参数。" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

# A. LiDAR 更新频率
measure_topic_hz "/scan" "01_lidar_raw_scan_hz"
measure_topic_hz "/scan_aligned" "02_lidar_scan_aligned_hz"
measure_topic_hz "/rl/lidar_181" "03_lidar_181_hz"
measure_topic_hz "/rl/lidar_543" "04_lidar_543_model_input_hz"

# B. 策略模型推理频率
# /rl/obs_556：有效目标状态下，每次模型输入发布一次，可作为推理输入频率参考
# /rl_cmd_vel：每次有效推理并完成动作映射后发布控制指令，可作为推理输出频率参考
measure_topic_hz "/rl/obs_556" "05_rl_obs_556_inference_input_hz"
measure_topic_hz "/rl_cmd_vel" "06_rl_cmd_vel_inference_output_hz"

# C. 控制指令链路频率
measure_topic_hz "/raw_cmd_vel" "07_raw_cmd_vel_after_switch_hz"
measure_topic_hz "/cmd_vel" "08_cmd_vel_standard_hz"

# D. 底盘反馈频率
measure_topic_hz "/zeus_speed" "09_base_feedback_zeus_speed_hz"
measure_topic_hz "/odom" "10_base_feedback_odom_hz"

# E. 同步诊断频率
measure_topic_hz "/rl/debug/speed_age_ms" "11_speed_age_debug_hz"
measure_topic_hz "/rl/debug/goal_age_ms" "12_goal_age_debug_hz"
measure_topic_hz "/rl/debug/sync_age_pair_ms" "13_sync_age_pair_debug_hz"

echo "[6] 根据日志文件估算模型推理/控制输出频率" | tee -a "$REPORT"
cat > "$OUTDIR/analyze_inference_log_${STAMP}.py" <<'PYEOF'
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
PYEOF

chmod +x "$OUTDIR/analyze_inference_log_${STAMP}.py"
python3 "$OUTDIR/analyze_inference_log_${STAMP}.py" > "$LOG_ANALYSIS" 2>&1 || true
echo "日志分析结果已保存: $LOG_ANALYSIS" | tee -a "$REPORT"
cat "$LOG_ANALYSIS" | tee -a "$REPORT"
echo "" | tee -a "$REPORT"

echo "[7] 自动生成结论说明模板" | tee -a "$REPORT"
cat >> "$REPORT" <<'EOF2'

============================================================
结论填写说明
============================================================

1. LiDAR 更新频率
   - 原始激光雷达频率优先查看：
     ~/hz-message/topic_hz_xxx/01_lidar_raw_scan_hz.txt
   - 如果论文中讨论的是进入 RL 模型前的激光观测频率，则优先查看：
     ~/hz-message/topic_hz_xxx/04_lidar_543_model_input_hz.txt
   - 该频率属于 rostopic hz 实际测量结果，不应简单写成配置参数。

2. 策略模型推理频率
   - 本系统的 RL 推理节点不是固定 rospy.Rate 周期循环。
   - 推理由 /rl/lidar_543 的回调触发，因此模型推理频率通常受 /rl/lidar_543 更新频率、CPU 推理耗时和目标有效状态共同影响。
   - 推荐查看：
     ~/hz-message/topic_hz_xxx/05_rl_obs_556_inference_input_hz.txt
     ~/hz-message/topic_hz_xxx/06_rl_cmd_vel_inference_output_hz.txt
     ~/hz-message/inference_log_analysis_xxx.txt
   - 该频率属于实际测量结果，不是配置参数。

3. 控制指令发布频率
   - RL 原始控制输出频率查看 /rl_cmd_vel。
   - 如果系统中存在速度选择/切换节点，则最终进入底盘前的控制频率还应查看 /raw_cmd_vel 或 /cmd_vel。
   - 该频率属于实际测量结果。

4. 底盘反馈频率
   - 当前 RL 节点使用 /zeus_speed 作为速度反馈输入。
   - 因此底盘反馈频率优先查看：
     ~/hz-message/topic_hz_xxx/09_base_feedback_zeus_speed_hz.txt
   - 如果系统同时发布 /odom，可将 /odom 作为里程计反馈频率补充说明。
   - 该频率属于实际测量结果。

5. 配置参数与实际测量的关系
   - 配置参数只表示系统期望频率或节点内部设定。
   - rostopic hz 表示真实 ROS 通信链路中的实际发布/接收频率。
   - 论文中建议写法：
     “系统运行频率通过 ROS 话题实际测量获得，而非仅依据配置文件给出。其中 LiDAR 更新频率由 /scan 或 /rl/lidar_543 测得，策略推理频率由 /rl/obs_556 与 /rl_cmd_vel 测得，控制指令发布频率由 /rl_cmd_vel 或 /raw_cmd_vel 测得，底盘反馈频率由 /zeus_speed 测得。”

EOF2

echo "============================================================" | tee -a "$REPORT"
echo "采集完成。" | tee -a "$REPORT"
echo "总报告文件: $REPORT" | tee -a "$REPORT"
echo "话题频率目录: $TOPIC_DIR" | tee -a "$REPORT"
echo "============================================================" | tee -a "$REPORT"

