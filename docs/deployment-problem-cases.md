# 从仿真策略到实车：关键部署问题与处理

部署中有三类不同的问题，需要分别说明处理方法与完成情况。

| 问题层次 | 具体表现 | 处理与状态 |
|---|---|---|
| 坐标与感知方向 | 雷达扫描前向与车体真实前向相反约180° | 已完成扫描重排及配套TF修正，并用于ROS1实车预处理链路 |
| 场景与导航任务 | 8×8米场景训练的策略进入较大环境，直接追最终目标时出现绕远 | 已形成全局路径前瞻目标与C++插件源码；构建和大场景效果仍待验证 |
| 模型输入接口 | 激光量程、goal4维度、历史帧和统计归一化不匹配 | 主线按12米、goal4、三帧及配套normalizer构造输入；历史模型须分别配对 |

<a id="lidar-frame-alignment"></a>

## 1 已解决：激光雷达与车体前向相反约180°

**现象。** [原始部署说明](../documents/rl-deployment-guide.docx)明确记录：真实位于机器人前方的障碍，在原始雷达数组中可能表现为后方数据。问题是雷达坐标系与车体前向的对应关系，不能只概括为“安装不同”。实车[TF查询记录](../experiments/real_robot_2026-04-28/3-other-message/other-message/other-message/17_tf_echo_results.txt)中，`base_footprint → laser_link`多次记录yaw约3.140弧度，即179.909°；同一文件还保存了不同的TF输出，不能将其全部视为同一时刻的固定外参。

**处理。** 在进入策略的激光裁剪、重采样之前，先修正前后方向：

1. [`laser_switch.py`](../ros1/python_nodes/rl_planner/scripts/laser_switch.py)将完整扫描循环平移半周，`new[i] = old[(i + N//2) % N]`；偶数束扫描对应180°。它保持距离数值与角度元数据，同步重排长度匹配的`intensities`，并非把数组倒序。奇数束时存在一个采样间隔以内的离散误差。
2. 发布`/scan_aligned`，使用逻辑坐标系`laser_link_aligned`，使重排后的前方数据与车体前向语义一致。
3. 为该逻辑坐标系配置对应TF。归档有两种入口：[`laser_switch.launch`](../ros1/python_nodes/rl_planner/launch/laser_switch.launch)从`base_footprint`发布前向一致、x偏移0.370米的aligned坐标系；[`rl_obs0_181_rviz.launch`](../ros1/python_nodes/rl_planner/launch/rl_obs0_181_rviz.launch)则从原始`laser_link`绕Z轴旋转π。按当次TF树选用一个入口，不能同时为同一child发布两套变换，也不能重复旋转。
4. 对齐后再进行270°视场裁剪、181束重采样、12米归一化和三帧堆叠。它们负责视场、尺度和时间历史，不是180°修正本身。RViz观测节点提供模型实际181束输入的方向核对入口。

**完成依据。** 原始部署说明将“激光雷达前后方向反了180°”列为已解决问题；[实车运行记录](../experiments/real_robot_2026-04-28/0-问题回答/zj.txt)同时保存`laser_switch.py`运行进程，以及`/scan_aligned`累计667条消息、约14.211Hz的记录。因此本项作为**已解决并用于ROS1实车链路的工程工作**呈现。记录没有提供修正前后对照指标或完整外参标定残差，不将方向修正扩大表述为完整传感器标定。

<a id="scene-gap"></a>

## 2 单独处理：训练场景与实车实验场景不一致

**现象。** 原始研究讨论记录：模型在约8×8米的仿真场景训练，小场景表现可用，但进入更大的实验环境时出现绕远；早期部署又绕过全局规划器，直接把最终目标送给策略。这里既涉及训练与实验场景分布差异，也涉及仅用局部感知追逐远处最终目标时缺少全局路径引导的问题。仅凭现有记录不能将所有绕行归因于单一因素，相关讨论来源索引见[研究记录](research_discussions/README.md)。

**处理思路与已做工作。** 保留PPO、ONNX及原有556维/goal4接口，在策略外部接入全局路径，选择前瞻子目标，再变换到车体系：

```text
全局规划路径 → 前瞻子目标 → TF转换 → goal4 → 原ONNX策略 → 速度命令
```

这让局部策略处理下一段路径目标，并由全局路径提供绕障与路线引导。后续已保存[`nav_core::BaseLocalPlanner`插件源码](../ros1/local_planner_plugin/rl_planner/src/rl_local_planner_ros.cpp)，包含`setPlan()`接收路径、路径点坐标变换、前瞻点选择与策略推理接口。当前源码按路径点相对车体的**欧氏距离阈值**选前瞻点，历史讨论中的沿路径累计弧长方案不能当成该版本的实际实现。

**完成情况。** 这是已实现源码的适配方案，尚不能写为“大场景问题已验证解决”。[插件工作总结第10节](../ros1/local_planner_plugin/rl_planner/docs/2026-06-19_work_summary.md)明确说明缺少ROS/catkin和ONNX Runtime C++环境，尚未真实编译；全局路径接收、输入等价、目标稳定性及实车沿路径行驶仍需验证。保存了实车运行材料的[Python目标节点](../ros1/python_nodes/rl_planner/scripts/rl_local_goal_provider.py)仍订阅RViz最终目标，与这一后续插件阶段分别呈现。

**与输入对齐的区别。** 8×8米是场景尺寸，12米是激光距离归一化尺度；二者没有直接换算关系。统一归一化、补齐dz和导出normalizer能够修正模型输入接口，不能消除场景分布差异，也不能证明策略已经具备大场景泛化能力。课程学习、场景随机化、奖励调整等讨论内容，没有充分训练与评估记录支持的，不列为已完成的解决措施。

<a id="observation-contract"></a>

## 3 模型输入对齐另行保留

主线激光按12米归一化；目标保留`[dx,dy,dz,dyaw]`中的dz占位；三帧按当前、上一帧、上上帧排列；actor导出时封装配套统计normalizer。这些解决输入尺度、索引、时序和网络统计接口问题。早期16米说明、194维Stage1及556维Stage3按各自版本保留，不混用。

具体维度、参数与模型配对见[训练与部署接口](training-and-deployment.md)，其余部署调试及MATLAB实验见[项目经历](project-journey.md)。
