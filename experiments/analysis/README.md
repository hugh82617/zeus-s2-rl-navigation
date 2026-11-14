# 统计与数据来源

- `analyze_episodes.py`：读取仓库内4份episode CSV，检查900条唯一ID、outcome与scenario/speed/start_dist对应，输出全量计数及分组统计。
- `episode_statistics_recomputed.json`：在本仓库目录结构内运行上述脚本生成的结果。
- `episode_statistics_extracted.json`：上一轮本地资料提取时对同4份CSV的统计，保留更多原始字段说明。
- `real_robot_summaries.json`：75份实车summary的字段提取，不等于统一的无碰撞成功率统计。
- `rosbag_topics.json`：原始bag连接元数据，话题本身属于Navfn/DWA运行。

JSON内文件位置采用仓库相对路径；`source:`前缀代表未在此复制的原始知识库相对路径。原始数据与结论的区别、论文初稿和CSV的数值冲突见[实验说明](../README.md)。
