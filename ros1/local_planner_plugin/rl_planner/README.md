# C++强化学习局部规划器快照

插件类 `rl_planner::RLLocalPlannerROS` 实现 `nav_core::BaseLocalPlanner`。主要部件为 `LidarPipeline`、`WaypointExtractor`、`OnnxPolicy`，由move_base的 `setPlan()` 接收全局路径。

模型保存在 `models`，必须同时保留 `.onnx` 与 `.onnx.data`。安装和运行说明在 [docs](docs)；它们是历史记录，其参数可能与实际YAML不同。当前配置为lookahead=0.8m、xy/yaw容差0.1m/0.1rad、v[-0.2,0.3]、w[-0.5,0.8]、bias_w=-0.13。请以实际加载参数为准。

保存代码按车体到路径点的欧氏距离选lookahead点，并非累计路径弧长。近终点时将goal4第四项切换为最终姿态误差，与训练接口所述目标方位角语义不同，控制效果需要验证。

**本次仅整理归档。历史总结明确没有ROS/catkin与ONNX Runtime C++环境，未完成真实编译和实车验证。** Python/C++激光观测处理亦不可默认逐帧等价。[另一个修订](../../../archive/plugin_fixed) 保留用于比较，不与此目录同时作为同名包编译。
