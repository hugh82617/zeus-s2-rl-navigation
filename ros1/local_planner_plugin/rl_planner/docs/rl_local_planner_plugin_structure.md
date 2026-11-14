# RL local planner 单插件结构说明

当前包已经整理为标准 ROS1 `nav_core::BaseLocalPlanner` 插件结构。

## 核心文件

```text
rl_planner/
├── include/rl_planner/rl_local_planner_ros.h
├── src/rl_local_planner_ros.cpp
├── config/rl_local_planner_params.yaml
├── launch/rl_local_planner_plugin.launch
├── rl_local_planner_plugin.xml
├── CMakeLists.txt
├── package.xml
└── models/model_1200_actor.onnx
```

## 插件内部职责

`RLLocalPlannerROS` 直接实现 `nav_core::BaseLocalPlanner` 的四个接口：

- `initialize()`：读取参数、加载 ONNX 模型、订阅 `/scan` 和速度话题。
- `setPlan()`：接收 `move_base` 传入的完整全局路径。
- `computeVelocityCommands()`：生成局部目标、生成 543 维 LiDAR、拼接 `obs_556`、执行 ONNX C++ 推理并返回 `cmd_vel`。
- `isGoalReached()`：返回目标到达状态。

## 与旧多节点链路的区别

旧链路：

```text
laser_switch.py
  -> lidar_obs_single_frame.py
  -> lidar_stack_543_node.py
  -> rl_local_goal_provider.py
  -> rl_planner_node.py
  -> /rl_cmd_vel
```

单插件链路：

```text
move_base
  -> rl_planner/RLLocalPlannerROS
       -> LidarPipeline
       -> WaypointExtractor
       -> OnnxPolicy
       -> cmd_vel
```

## 编译前提

必须安装 ONNX Runtime C++：

```text
/opt/onnxruntime/include/onnxruntime_cxx_api.h
/opt/onnxruntime/lib/libonnxruntime.so
```

安装方式见：

```text
docs/onnxruntime_cpp_install.md
```
