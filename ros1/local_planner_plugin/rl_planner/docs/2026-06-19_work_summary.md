# 2026-06-19 工作总结记录

## 1. 今日目标

今天的主要目标是把原来由多个 ROS Python 节点串联起来的强化学习局部规划器，整理成标准 ROS1 `nav_core::BaseLocalPlanner` 插件形式，使其可以被 `move_base` 通过 pluginlib 加载。

改造后的目标运行方式是：

- `move_base` 调用 `setPlan()` 传入全局路径；
- 插件内部提取局部 lookahead 目标点；
- 插件内部处理 LiDAR 并构造 556 维 RL 观测；
- 插件内部使用 ONNX Runtime C++ 进行模型推理；
- `computeVelocityCommands()` 直接把速度返回给 `move_base`；
- 同时继续发布同一条速度到 `/rl_cmd_vel`，保留原系统调试/观察接口。

## 2. 仓库清理结果

已经把仓库清理成单插件结构。旧 Python 节点、旧 launch、Rviz 调试文件、Visual Studio 缓存、CMake `out/` 缓存以及 `models/test_onnx.py` 均已删除。

当前保留结构如下：

```text
rl_planner/
├── .gitignore
├── CMakeLists.txt
├── package.xml
├── rl_local_planner_plugin.xml
├── config/
│   └── rl_local_planner_params.yaml
├── docs/
│   ├── 2026-06-19_work_summary.md
│   ├── onnxruntime_cpp_install.md
│   ├── rl_local_planner_plugin_structure.md
│   └── run_rl_plugin.md
├── include/
│   └── rl_planner/
│       └── rl_local_planner_ros.h
├── launch/
│   └── rl_local_planner_plugin.launch
├── models/
│   ├── model_1200_actor.onnx
│   └── model_1200_actor.onnx.data
└── src/
    └── rl_local_planner_ros.cpp
```

新增 `.gitignore`，用于阻止 `.vs/`、`out/`、catkin 构建产物、CMake 中间文件和 Python 缓存再次混入工作目录。

## 3. 当前插件身份

插件类：

```cpp
rl_planner::RLLocalPlannerROS
```

pluginlib 导出名：

```text
rl_planner/RLLocalPlannerROS
```

插件描述文件：

```text
rl_local_planner_plugin.xml
```

配置文件中已经设置：

```yaml
base_local_planner: "rl_planner/RLLocalPlannerROS"
```

这意味着在 `move_base` 中，它会替代 DWA/TEB 这类局部规划器，作为当前导航系统的局部规划器插件运行。

## 4. 插件内部模块

### 4.1 LidarPipeline

`LidarPipeline` 代替原来的 LiDAR 预处理节点链。

当前功能：

- 订阅 `/scan`；
- 对 scan 做半圈旋转；
- 修复非法距离值；
- 重采样为 181 维；
- 堆叠最近 3 帧，形成 543 维 LiDAR 输入；
- 可发布 `/rl/lidar_543` 调试话题。

说明：当前 C++ 版本是简化版 LiDAR 管线，后续在机器人上建议与旧 Python 输出逐帧对比，确认 543 维观测是否完全一致。

### 4.2 WaypointExtractor

`WaypointExtractor` 代替原来的 `rl_local_goal_provider.py`。

当前功能：

- `setPlan()` 接收并保存 `move_base` 传入的全局路径；
- 每个控制周期从路径中选取 lookahead 目标点；
- 使用 TF 把目标点从全局坐标系转换到机器人本体坐标系；
- 输出 RL 需要的 4 维目标：

```text
[dx, dy, dz, dyaw]
```

今天已完成进一步完善：

- 新增 `local_target_` 成员变量，用于保存已经转换到 `base_frame` 的局部目标点；
- 新增 `local_target_valid_` 状态标志；
- 新增 `getStoredLocalTarget()` 和 `hasValidLocalTarget()` 访问接口；
- `setPlan()` 收到新路径时会清空旧局部目标，并把 `local_target_valid_` 置为 `false`；
- `getLocalGoal()` 成功选出目标后，会保存 `local_target_` 并置 `local_target_valid_ = true`；
- 如果 plan 为空、TF 转换失败、转换后没有有效路径点，会置 `local_target_valid_ = false`，并让上层 `computeVelocityCommands()` 返回 `false`；
- TF 转换时使用 `ros::Time(0)` 获取最新可用变换，避免直接使用旧路径时间戳造成局部目标滞后。

坐标系结论：

- `setPlan()` 收到的路径通常在 `map` 或全局代价地图坐标系下；
- RL 模型使用的局部目标不是 map 坐标；
- 当前实现会在提取目标时通过 TF 转换到 `base_frame`；
- `base_frame` 默认来自 `costmap_ros_->getBaseFrameID()`，通常是 `base_footprint`；
- 因此进入 RL 观测的目标点是在机器人本体坐标系下运行。

### 4.3 OnnxPolicy

`OnnxPolicy` 代替原来的 Python ONNX 推理节点。

当前功能：

- 使用 ONNX Runtime C++ 加载 `model_1200_actor.onnx`；
- 输入 556 维观测；
- 输出 2 维动作；
- 按参数映射成线速度和角速度；
- 返回 `geometry_msgs::Twist`。

当前模型输入输出名由 YAML 参数控制：

```yaml
input_name: "obs"
output_name: "action"
```

如果目标机器人上的 ONNX 模型输入输出名不同，需要同步修改该参数。

## 5. 导航运行流程

单插件在 `move_base` 中的运行流程如下：

1. `move_base` 启动时读取 `base_local_planner` 参数。
2. pluginlib 加载 `rl_planner/RLLocalPlannerROS`。
3. `initialize()` 初始化参数、TF、代价地图指针、ONNX 模型、LiDAR 订阅和调试话题。
4. 全局规划器产生路径后，`move_base` 调用 `setPlan()`。
5. 插件保存全局路径，并重置旧局部目标状态。
6. 每个控制周期，`move_base` 调用 `computeVelocityCommands()`。
7. 插件从全局路径中提取 lookahead 点，并转换到 `base_footprint`。
8. 插件构造 556 维 RL 观测：

```text
速度 6 维 + 固定状态 3 维 + 局部目标 4 维 + LiDAR 543 维 = 556 维
```

9. 插件调用 ONNX Runtime C++ 进行策略推理。
10. 插件把推理结果映射成 `cmd_vel`。
11. 插件将 `cmd_vel` 返回给 `move_base`。
12. 插件同时把同一条速度发布到 `/rl_cmd_vel`。

## 6. 重要参数

参数文件：

```text
config/rl_local_planner_params.yaml
```

关键参数：

```yaml
scan_topic: "/scan"
```

LiDAR 输入话题。

```yaml
speed_topic: "/zeus_speed"
```

机器人当前速度输入话题，用于构造 RL 观测中的速度部分。

```yaml
model_path: "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx"
```

ONNX 模型路径，部署时必须确认机器人上路径真实存在。

```yaml
out_cmd_topic: "/rl_cmd_vel"
```

RL 推理速度额外发布话题。即使插件已经把速度返回给 `move_base`，该话题仍然保留。

```yaml
lookahead_dist: 0.8
goal_tolerance: 0.5
tf_timeout: 0.1
```

路径目标提取和 TF 转换相关参数。

```yaml
max_distance: 12.0
range_min: 0.05
fov_min_deg: -135.0
fov_max_deg: 135.0
rotate_half_turn: true
```

LiDAR 预处理相关参数。

```yaml
linear_min: -0.2
linear_max: 0.5
angular_min: -0.5
angular_max: 0.8
action_bias_w: -0.13
```

RL 动作到实际速度命令的映射参数。

## 7. ONNX Runtime C++ 依赖

真正单插件方案需要在 C++ 中直接推理 ONNX 模型，因此目标机器人需要安装 ONNX Runtime C++。

当前 CMake 默认期望：

```text
/opt/onnxruntime/include/onnxruntime_cxx_api.h
/opt/onnxruntime/lib/libonnxruntime.so
```

安装说明已记录：

```text
docs/onnxruntime_cpp_install.md
```

建议优先使用 ONNX Runtime C++ CPU 版本 1.18.1，除非目标机器人需要 CUDA/TensorRT 或特殊架构版本。

## 8. 如何从 DWA 切换到 RL 插件

当前系统中如果已经有 DWA 局部规划器，本质上只需要替换 `move_base` 的局部规划器插件参数。

原 DWA 常见配置：

```yaml
base_local_planner: "dwa_local_planner/DWAPlannerROS"
```

切换到 RL：

```yaml
base_local_planner: "rl_planner/RLLocalPlannerROS"
```

重要注意：

- DWA 和 RL 插件是同一个 `BaseLocalPlanner` 插槽下的二选一关系；
- 不应该让 DWA 和 RL 同时作为 `move_base` 的局部规划器运行；
- `move_base` 只在启动时加载局部规划器；
- 如果 `move_base` 已经启动，之后再加载参数不会自动从 DWA 切到 RL；
- 正确做法是在启动 `move_base` 之前加载 `rl_local_planner_params.yaml`，或者修改原导航总 launch。

推荐在原导航 launch 的 `move_base` 节点中加入：

```xml
<rosparam file="$(find rl_planner)/config/rl_local_planner_params.yaml" command="load" />
```

运行说明已单独记录：

```text
docs/run_rl_plugin.md
```

验证插件是否能被发现：

```bash
rospack plugins --attrib=plugin nav_core
```

运行时验证：

```bash
rostopic echo /rl_cmd_vel
```

并观察日志中是否出现：

```text
[RLLocalPlannerROS] initialized
```

## 9. 今日已完成事项

1. 将仓库清理为单插件干净结构。
2. 删除旧 Python 节点和旧 launch 文件。
3. 保留并整理标准 ROS 插件文件：
   - `include/rl_planner/rl_local_planner_ros.h`
   - `src/rl_local_planner_ros.cpp`
   - `rl_local_planner_plugin.xml`
   - `CMakeLists.txt`
   - `package.xml`
   - `config/rl_local_planner_params.yaml`
4. 保留 ONNX 模型文件。
5. 重写 `rl_local_planner_plugin.launch`，去除乱码注释。
6. 新增 `.gitignore`。
7. 补齐 `WaypointExtractor` 的 `local_target_` 和 `local_target_valid_`。
8. 增强 TF 失败时的无效状态处理。
9. 新增 `docs/run_rl_plugin.md`，说明如何从 DWA 切换到 RL 插件运行。
10. 更新本工作总结。

## 10. 尚未完成或后续需要验证

1. 本地没有 ROS/catkin 环境，也没有 ONNX Runtime C++，因此今天没有进行真实编译。
2. 需要在目标机器人或 ROS 环境中执行 `catkin_make`。
3. 需要确认 ONNX 模型输入输出名是否确实是 `obs` 和 `action`。
4. 需要确认机器人上 `model_path` 是否正确。
5. 需要对比新 C++ `LidarPipeline` 与旧 Python 管线的 543 维输出一致性。
6. 需要在真实导航中验证：
   - `setPlan()` 是否正常收到全局路径；
   - 局部目标是否稳定处于 `base_footprint`；
   - `/rl/obs_556` 是否维度和数值合理；
   - `/rl_cmd_vel` 是否与 `move_base` 返回速度一致；
   - 机器人是否能沿全局路径稳定行驶。

## 11. 下一步建议

1. 在机器人或 ROS 虚拟机中安装 ONNX Runtime C++。
2. 编译当前 `rl_planner` 包。
3. 使用 `rospack plugins --attrib=plugin nav_core` 验证插件注册。
4. 把 `rl_local_planner_params.yaml` 合并到原导航总 launch 的 `move_base` 启动流程中。
5. 先在空旷环境中测试 RL 插件。
6. 再进入走廊、转弯、障碍物等复杂场景测试。

