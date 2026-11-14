# 如何在已有 DWA 导航系统中运行新的 RL 局部规划器插件

## 1. 核心切换方式

ROS1 `move_base` 一次只会加载一个 `nav_core::BaseLocalPlanner` 局部规划器插件。

如果当前系统使用 DWA，通常配置里会有类似参数：

```yaml
base_local_planner: "dwa_local_planner/DWAPlannerROS"
```

要切换到当前 RL 插件，需要改成：

```yaml
base_local_planner: "rl_planner/RLLocalPlannerROS"
```

本包已经在以下配置文件中写好：

```text
config/rl_local_planner_params.yaml
```

其中第一行就是：

```yaml
base_local_planner: "rl_planner/RLLocalPlannerROS"
```

## 2. 重要原则

`move_base` 只在启动时读取并加载局部规划器插件。

因此不能在 `move_base` 已经运行后，仅仅重新加载参数就指望它从 DWA 自动切到 RL。正确做法是：

1. 在启动 `move_base` 之前加载 RL 插件参数；
2. 或者修改原导航 launch，使其启动 `move_base` 时直接加载 `rl_local_planner_params.yaml`；
3. 如果 `move_base` 已经启动，需要重启 `move_base`。

## 3. 推荐接入方式

假设原来的导航启动文件中有：

```xml
<node pkg="move_base" type="move_base" name="move_base" output="screen">
  ...
</node>
```

可以在这个 `move_base` 节点内部加入：

```xml
<rosparam file="$(find rl_planner)/config/rl_local_planner_params.yaml" command="load" />
```

示例：

```xml
<node pkg="move_base" type="move_base" name="move_base" output="screen">
  <rosparam file="$(find your_robot_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
  <rosparam file="$(find your_robot_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
  <rosparam file="$(find your_robot_navigation)/config/global_costmap_params.yaml" command="load" />
  <rosparam file="$(find your_robot_navigation)/config/local_costmap_params.yaml" command="load" />
  <rosparam file="$(find rl_planner)/config/rl_local_planner_params.yaml" command="load" />
</node>
```

原来的 costmap、global planner、地图、AMCL、TF、底盘驱动等配置不需要因为切换局部规划器而删除。

## 4. 使用本包提供的 launch

本包保留了一个单插件启动辅助文件：

```text
launch/rl_local_planner_plugin.launch
```

如果你的完整导航系统已经会启动 `move_base`，则应该把这个文件的参数加载逻辑合并到原导航 launch 中，而不是等 `move_base` 启动后再加载。

如果只是临时测试参数加载，可以运行：

```bash
roslaunch rl_planner rl_local_planner_plugin.launch start_move_base:=false move_base_name:=move_base
```

注意：这只会把参数加载到 `/move_base` 命名空间。若 `/move_base` 已经运行，它不会自动卸载 DWA 并切换到 RL。

如果只是快速启动一个示例 `move_base`：

```bash
roslaunch rl_planner rl_local_planner_plugin.launch start_move_base:=true move_base_name:=move_base
```

注意：真实机器人通常还需要同时加载完整的 costmap、地图、定位、TF 和底盘驱动参数，所以更推荐合并到原导航总启动文件。

## 5. 编译前依赖

目标机器人需要有 ONNX Runtime C++：

```text
/opt/onnxruntime/include/onnxruntime_cxx_api.h
/opt/onnxruntime/lib/libonnxruntime.so
```

然后在 catkin 工作空间中编译：

```bash
catkin_make \
  -DONNXRUNTIME_INCLUDE_DIR=/opt/onnxruntime/include \
  -DONNXRUNTIME_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so
```

编译完成后：

```bash
source devel/setup.bash
```

## 6. 验证插件是否能被 ROS 发现

运行：

```bash
rospack plugins --attrib=plugin nav_core
```

应该能看到类似：

```text
rl_planner .../rl_local_planner_plugin.xml
```

如果看不到，优先检查：

- 是否 `source devel/setup.bash`；
- `package.xml` 是否导出了 plugin；
- `rl_local_planner_plugin.xml` 是否安装/存在；
- `CMakeLists.txt` 是否正确安装 plugin xml 和库文件。

## 7. 运行后如何确认正在使用 RL 插件

启动导航后检查日志，应该看到：

```text
[RLLocalPlannerROS] initialized
```

同时检查话题：

```bash
rostopic echo /rl_cmd_vel
```

该话题会发布 RL 模型推理出的速度。

同时，`move_base` 仍然会按标准流程发布最终速度到：

```text
/cmd_vel
```

当前插件逻辑是：

- `computeVelocityCommands()` 把速度返回给 `move_base`；
- 插件额外把同一条速度发布到 `/rl_cmd_vel`，用于保留原系统调试/观察接口。

## 8. 如何切回 DWA

把参数改回：

```yaml
base_local_planner: "dwa_local_planner/DWAPlannerROS"
```

然后重启 `move_base`。

DWA 和 RL 插件不应该作为 `move_base` 的局部规划器同时运行。它们是同一个插件接口下的二选一关系。

