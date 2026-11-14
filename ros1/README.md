# ROS1导航实现

这里保存两个不同阶段的实现，均保持归档源码逻辑。

| 目录 | 内容 | 验证边界 |
|---|---|---|
| [python_nodes/rl_planner](python_nodes/rl_planner) | 激光三级处理、最终目标转换、556维观测、ONNX推理及记录器 | 保存的多节点实车部署快照；launch存在缺失同名yaml/脚本引用，须先核查 |
| [local_planner_plugin/rl_planner](local_planner_plugin/rl_planner) | C++ nav_core::BaseLocalPlanner插件 | 历史工作总结明确未完成ROS/catkin真实编译和实车验证 |

两个目录的包名均为 `rl_planner`，不要同时放入同一catkin工作空间。历史修订在 [archive](../archive) 中；不能按“fixed”文件名或修改时间认定最新版。

主包模型文件保留在各自 `models` 内，以保留相对路径关系。模型输入556维、输出2维，小体积ONNX需要同名 `.onnx.data`。源码中的机器路径、话题名、设备及参数属于历史环境，迁移前需核对。

Python目标节点直接读取RViz最终目标，并非全局路径前瞻点。C++通过 `setPlan()`接收路径，其已保存实现从最近路径段/点之后按相对车体欧氏距离达到lookahead阈值选点，没有累计路径弧长。聊天提出过的弧长选点方案不能当作现存代码行为。

Python与C++的激光修复/中位处理、终点yaw和动作参数存在差异，迁移前应逐帧核对543/556维观测。当前仓库整理不表示这些分支已完成等价性验证。
