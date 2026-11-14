#!/usr/bin/env bash
# ===================================================================
# record_pipeline_v4.sh
# -------------------------------------------------------------------
# 为 LiDAR 预处理链路的 13 个话题分别录制 rosbag。
#
# 目录布局:
#   /home/ros/save_dir/lidar/
#       └── 20260420_174500/          <- 本次录制按时间命名的子文件夹
#             ├── scan.bag
#             ├── scan_aligned.bag
#             ├── scan_aligned_invalid_mask.bag
#             ├── scan_aligned_invalid_stats.bag
#             ├── rl__lidar_181.bag
#             ├── rl__lidar_181_stamp.bag
#             ├── rl__lidar_181_src_map.bag
#             ├── rl__lidar_181_repair_mask.bag
#             ├── rl__lidar_181_fill_stats.bag
#             ├── rl__lidar_543.bag
#             ├── rl__lidar_543_ids.bag
#             ├── rl__lidar_543_ids_vis.bag
#             └── rl__lidar_543_stamps.bag
#
# 话题名到文件名转义规则: 去前导 "/",其余 "/" 变 "__"
#   /scan                 -> scan.bag
#   /rl/lidar_181_stamp   -> rl__lidar_181_stamp.bag
# ===================================================================
set -e

BASE_DIR="${BASE_DIR:-/home/ros/save_dir/lidar}"
TS=$(date +%Y%m%d_%H%M%S)
OUT_DIR="$BASE_DIR/$TS"
mkdir -p "$OUT_DIR"

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

# 话题 -> 文件名 (不含 .bag,rosbag -O 会自动补)
topic_to_name() {
    local t="$1"
    local s="${t#/}"
    echo "${s//\//__}"
}

echo "========================================"
echo "[record_pipeline_v4]"
echo "  output dir : $OUT_DIR"
echo "  topics     : ${#TOPICS[@]}"
echo "  press Ctrl+C once to stop ALL recorders"
echo "========================================"

PIDS=()
cleanup() {
    echo ""
    echo "[record_pipeline_v4] received stop signal, terminating ${#PIDS[@]} recorders..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    # rosbag record 收到 SIGINT 会 flush 再退出
    wait 2>/dev/null || true
    echo "[record_pipeline_v4] done. Bag files in $OUT_DIR:"
    ls -lh "$OUT_DIR"/*.bag 2>/dev/null || echo "  (no bag files produced)"
    exit 0
}
trap cleanup INT TERM

# 为每个话题起一个独立 rosbag record 进程
# --buffsize=512 增大内核缓冲,高频话题不丢帧
# stdout/stderr 丢到 /dev/null,不生成 log 文件
for t in "${TOPICS[@]}"; do
    name="$(topic_to_name "$t")"
    out="$OUT_DIR/${name}"
    echo "  [spawn] $t -> ${name}.bag"
    rosbag record -O "$out" --buffsize=512 "$t" >/dev/null 2>&1 &
    PIDS+=($!)
done

echo "[record_pipeline_v4] all ${#PIDS[@]} recorders running; waiting..."
wait
