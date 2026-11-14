#!/bin/bash
# ============================================================
# record_pipeline_v2.sh
# ------------------------------------------------------------
# LiDAR 预处理链(stage 1 v3 + stage 2 v3 + stage 3 v2)统一录制脚本。
# 按《总体修改意见》第四节"统一录制与导出规范"生成一个 bag,包含:
#
#     主链路话题         诊断话题
#     ------------        -----------------------------
#     /scan               /scan_aligned_invalid_mask
#     /scan_aligned       /scan_aligned_invalid_stats
#     /rl/lidar_181       /rl/lidar_181_stamp
#     /rl/lidar_543       /rl/lidar_181_src_map
#                         /rl/lidar_181_repair_mask
#                         /rl/lidar_181_fill_stats
#                         /rl/lidar_543_ids
#                         /rl/lidar_543_ids_vis
#                         /rl/lidar_543_stamps
#
# 用法:
#   ./record_pipeline_v2.sh                 # 默认位置:/home/ros/save_dir/lidar/
#   BAG_DIR=/tmp ./record_pipeline_v2.sh    # 自定义目录
#   BAG_NAME=my_run ./record_pipeline_v2.sh # 自定义 bag 名(不含扩展名)
#
# 环境要求:
#   - 已 source ROS 环境(setup.bash)
#   - 三个节点都在运行且发布以上 13 个话题
# ============================================================

set -e

BAG_DIR="${BAG_DIR:-/home/ros/save_dir/lidar}"
BAG_NAME="${BAG_NAME:-pipeline_v2_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$BAG_DIR"
OUT_PATH="${BAG_DIR}/${BAG_NAME}.bag"

echo "========================================"
echo "[record_pipeline_v2] OUT_PATH = $OUT_PATH"
echo "[record_pipeline_v2] press Ctrl+C to stop recording"
echo "========================================"

rosbag record \
    -O "${OUT_PATH}" \
    /scan \
    /scan_aligned \
    /scan_aligned_invalid_mask \
    /scan_aligned_invalid_stats \
    /rl/lidar_181 \
    /rl/lidar_181_stamp \
    /rl/lidar_181_src_map \
    /rl/lidar_181_repair_mask \
    /rl/lidar_181_fill_stats \
    /rl/lidar_543 \
    /rl/lidar_543_ids \
    /rl/lidar_543_ids_vis \
    /rl/lidar_543_stamps

echo ""
echo "[record_pipeline_v2] saved -> $OUT_PATH"
ls -lh "${OUT_PATH}"
