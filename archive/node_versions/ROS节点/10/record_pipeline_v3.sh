#!/usr/bin/env bash
# ===================================================================
# record_pipeline_v3.sh
# -------------------------------------------------------------------
# 为 LiDAR 预处理链路的 13 个话题分别录制 rosbag。
# 每个话题一个 .bag 文件,文件名由话题路径转义而来,统一保存到:
#     /home/ros/save_dir/lidar/
#
# 话题名到文件名的转义规则:
#     前导 "/" 去掉;剩余 "/" 替换成 "__"
#     例: /rl/lidar_181_stamp  ->  rl__lidar_181_stamp.bag
#         /scan                ->  scan.bag
#
# 同一个话题被多次录制时,文件会加 "_YYYYMMDD_HHMMSS" 时间戳防覆盖。
# ===================================================================
set -e

OUT_DIR="${OUT_DIR:-/home/ros/save_dir/lidar}"
mkdir -p "$OUT_DIR"

TS=$(date +%Y%m%d_%H%M%S)

TOPICS=(
    /scan
    /scan_aligned
    /scan_aligned_invalid_mask
    /scan_aligned_invalid_stats
    /rl/lidar_181
    /rl/lidar_181_stamp
    /rl/lidar_181_src_map
    /rl/lidar_181_repair_mask
    /rl/lidar_181_fill_stats
    /rl/lidar_543
    /rl/lidar_543_ids
    /rl/lidar_543_ids_vis
    /rl/lidar_543_stamps
)

# 话题 -> 文件名(不带 .bag 扩展,rosbag record -O 会自动补)
# 例: /rl/lidar_181_stamp  ->  rl__lidar_181_stamp
#     /scan                ->  scan
topic_to_name() {
    local t="$1"
    # 去前导 /,然后把每个 / 替换成 __
    local s="${t#/}"
    echo "${s//\//__}"
}

echo "========================================"
echo "[record_pipeline_v3]"
echo "  output dir    : $OUT_DIR"
echo "  timestamp tag : $TS"
echo "  topics        : ${#TOPICS[@]}"
echo "  press Ctrl+C once to stop ALL recorders"
echo "========================================"

# 存放每个子进程 PID,以便 Ctrl+C 时统一收尾
PIDS=()
cleanup() {
    echo ""
    echo "[record_pipeline_v3] received stop signal, terminating ${#PIDS[@]} recorders..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    # 等所有子进程退出(rosbag record 收到 SIGINT 会 flush 再退出)
    wait 2>/dev/null || true
    echo "[record_pipeline_v3] done. Files in $OUT_DIR:"
    ls -lh "$OUT_DIR"/*_"$TS".bag 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# 为每个话题起一个独立 rosbag record 进程
for t in "${TOPICS[@]}"; do
    name="$(topic_to_name "$t")"
    out="$OUT_DIR/${name}_${TS}"
    echo "  [spawn] $t -> ${out}.bag"
    # -O: 明确指定输出路径(rosbag 会自动补 .bag)
    # --buffsize 增大内核缓冲,避免高频话题丢帧
    rosbag record -O "$out" --buffsize=512 "$t" \
        > "$OUT_DIR/${name}_${TS}.log" 2>&1 &
    PIDS+=($!)
done

echo "[record_pipeline_v3] all ${#PIDS[@]} recorders running; waiting..."
wait
