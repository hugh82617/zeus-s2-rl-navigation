# 2026年6月22日DWA和Navfn路径导出

保存NavfnROS全局plan、DWAPlannerROS local_plan和global_plan三个CSV。它们是传统导航的路径记录，可用于观察全局与局部路径的关系，不能证明后续RL局部规划器插件已经运行。

对应可读bag的15个连接见[bag话题元数据](../analysis/rosbag_topics.json)，未发现RL观测或RL命令连接。本目录仅保存路径CSV，不包含大型bag二进制。
