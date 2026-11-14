本压缩包为真机导航系统配置导出结果，重点用于排查和复现 move_base 相关配置。

目录说明：
1. source_files/
   保存从 catkin_ws/src 中搜索到的与 move_base、DWA、global planner、costmap 等相关的 launch/yaml 文件。
   这些文件反映源码层配置。

2. runtime_params/
   保存当前 ROS 参数服务器中 /move_base 及其关键子命名空间下的实际运行参数。
   这些文件优先反映“真机当前实际生效配置”。

3. runtime_info/
   保存 move_base 节点信息、topic 列表、参数列表、插件信息、关键参数摘要等辅助信息。

重点查看建议：
- planner_types.txt
- move_base_full.yaml
- global_costmap_runtime.yaml
- local_costmap_runtime.yaml
- DWAPlannerROS_runtime.yaml
- GlobalPlanner_runtime.yaml 或 NavfnROS_runtime.yaml
- key_params_summary.txt

说明：
如果源码里某些参数未单独写入 yaml，而是通过 launch、默认值或运行时动态加载进入参数服务器，
则 runtime_params/ 中的内容通常更接近真机实际配置。
