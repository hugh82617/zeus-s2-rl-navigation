# Zeus S2 智能小车强化学习导航与 ROS1 实车部署

将 PPO 导航策略通过 ONNX Runtime 接入 Zeus S2 智能小车的 ROS1 感知与控制链路，完成激光观测构造、目标坐标变换、策略推理、动作处理及实车实验记录。项目同时包含 MATLAB / Simulink 路径跟踪、激光 SLAM 与传统导航实验，以及后续 `move_base` 局部规划器插件源码。

**平台主线：ROS1、Noetic、Zeus S2、二维激光雷达、PPO、ONNX Runtime。** 早期 MATLAB / Simulink 模型另保存 ROS Melodic 目标配置；不同阶段的配置按原记录保留。

## 项目演示 · 视频1

https://github.com/user-attachments/assets/d4fc51cc-eefa-4091-9987-78dda76b44d4

毕业答辩目录中的“视频1”，时长38.88秒，保持原视频字节。播放器无法显示时，可[打开原始MP4](media/video1.mp4)。

[项目经历与问题解决](docs/project-journey.md) · [训练与部署](docs/training-and-deployment.md) · [系统架构](docs/architecture.md) · [MATLAB实验](docs/matlab-experiments.md) · [实验与数据](experiments/README.md) · [52段视频目录](media/README.md)

## 做了哪些工作

| 工作 | 实现与完成情况 | 直接查看 |
|---|---|---|
| PPO训练与ONNX导出 | 保存训练配置、检查点、194 / 556维模型；导出actor及观测normalizer | [训练快照](training/stage1/) · [导出脚本](models_and_export/RL-onnx/export_onnx.py) · [模型说明](models_and_export/README.md) |
| ROS1实车部署 | Python多节点组织感知、目标、ONNX推理与速度控制，保存实车运行材料 | [推理节点](ros1/python_nodes/rl_planner/scripts/rl_planner_node.py) · [4月28日运行记录](experiments/real_robot_2026-04-28/README.md) |
| 观测对齐与部署调试 | 激光方向及尺度对齐、三帧堆叠、输入真值校验、控制源切换、轨迹和obs556记录 | [三帧节点](ros1/python_nodes/rl_planner/scripts/lidar_stack_543_node.py) · [真值校验](ros1/python_nodes/rl_planner/scripts/rl_obs0_truth_validator.py) · [记录器](ros1/python_nodes/rl_planner/scripts/experiment_logger_with_obs556.py) |
| MATLAB / Simulink控制 | 已进行Pure Pursuit路径跟踪与远程实车实验，包含到点判定、零速切换与停止仿真逻辑 | [实验说明](docs/matlab-experiments.md) · [MATLAB脚本](matlab/scripts/) · [Simulink模型](matlab/simulink/exp_6_2020b.slx) |
| 激光SLAM与传统导航 | 保留激光预处理、里程计与闭环代码、地图、导航配置及运行演示 | [SLAM与地图](slam/README.md) · [ROS1运行配置](ros1/runtime_reference/) |
| 全局路径接入与插件封装 | 实现C++ `nav_core::BaseLocalPlanner`插件、路径前瞻点与配置；该分支的构建和实车验证尚待完成 | [插件源码](ros1/local_planner_plugin/rl_planner/) · [版本与验证状态](docs/version-history.md) |

<details>
<summary>查看 Zeus S2 实车平台</summary>

![Zeus S2实车平台](experiments/real_robot_2026-04-28/5-机器人实物图/机器人实物图-4.jpg)

[更多平台照片与实验记录](experiments/real_robot_2026-04-28/README.md)

</details>

## 强化学习如何接入小车

```mermaid
flowchart TD
    A[原始 LaserScan] --> B[方向对齐与181束重采样]
    B --> C[三帧543维激光]
    D[RViz最终目标] --> E[TF转换与车体系goal4]
    C --> F[556维观测]
    E --> F
    G[速度话题与固定重力投影] --> F
    F --> H[Normalizer与PPO Actor ONNX]
    H --> I[动作映射与限幅]
    I --> J[控制源切换 / 速度平滑 / 底盘]
    F --> K[观测校验与实验记录]
```

上图对应保存了实车运行材料的 **Python部署链路**。其中目标来自RViz最终目标，转换到车体系后进入策略；`/zeus_speed`在部分运行快照中可能是平滑命令，需按当次配置核对速度来源。

Stage3 观测为 `3线速度 + 3角速度 + 3重力 + 4目标 + 181×3激光 = 556维`。激光视场为 -135° 至 135°，按12m归一化，历史顺序为当前、上一帧、上上帧。Stage1另有194维模型，必须按配套配置使用。

**后续扩展：** C++插件通过 `move_base → setPlan() → 前瞻子目标 → ONNX → computeVelocityCommands()` 接入全局路径。该实现与上图分属不同阶段，构建与实车验证尚待完成；具体接口差异见[架构说明](docs/architecture.md)。

## 遇到的问题与处理方法

| 问题 | 已做的处理或形成的方案 | 需要保留的边界 |
|---|---|---|
| 真实雷达安装朝向与模型不同 | 半周索引对齐、270°裁剪、181束重采样，并在RViz核对 | 索引对齐与完整传感器外参不是同一件事 |
| 无效测距与动态障碍观测波动 | 空间/时间修复、三帧堆叠、修复mask及时间戳诊断 | Python和C++滤波细节不同，逐帧等价仍需验证 |
| 训练与部署输入尺度不一致 | 统一12m归一化、保留goal4的dz占位、随actor导出normalizer | 早期16m说明和194维旧版本仍可在归档中看到 |
| 小场景策略在大场景绕远 | 引入全局路径前瞻点，保留策略输入结构 | 聊天提出弧长方案；保存的C++按欧氏距离阈值选点 |
| 目标附近绕行、震荡与动作偏差 | 保存目标门控、限速、bias及近目标规则版本，记录raw/clip/cmd | 规则包裹与模型重新训练应分别说明；并非所有方案都有同等验证 |
| 难定位异常来自输入还是控制 | 保存obs556、源beam映射、真值验证、轨迹和运行参数 | 话题间隔不能直接作为纯ONNX推理耗时 |
| MATLAB终点附近持续跟踪 | 保存到点判定、零速切换及停止仿真的模型实现 | 脚本与模型的四元数接口差异需在复跑时核对 |

具体代码、参数、历史修订和原始证据见[项目经历](docs/project-journey.md)、[MATLAB实验](docs/matlab-experiments.md)及[版本说明](docs/version-history.md)。

## 实验数据

对保存的四份 episode CSV 按 `outcome` 全量计数，每组900回合：

| 方法/数据批次 | 成功 | 碰撞 | 超时 | 成功率 |
|---|---:|---:|---:|---:|
| ESA_ON | 814 | 73 | 13 | 90.44% |
| ESA_OFF | 266 | 591 | 43 | 29.56% |
| DWA原版 | 124 | 229 | 547 | 13.78% |
| 0414对标ESA设置的DWA | 189 | 344 | 367 | 21.00% |

仓库附[统计脚本](experiments/analysis/analyze_episodes.py)、原始 CSV、75份实车 summary 和4月28日8次运行记录。论文初稿中还有另一组到达率，尚未建立数据批次与筛选口径的对应关系；实车 `success` 字段也不等于无碰撞成功。两版DWA分开保存，详细口径见[实验说明](experiments/README.md)。

在仓库根目录运行以下命令即可复核四组统计，只需Python标准库：

```bash
python experiments/analysis/analyze_episodes.py
```

## 全部实验视频

52段有效视频按强化学习部署、SLAM与自主导航、MATLAB控制、答辩演示和诊断分类，原视频不转码，仅对SHA256完全相同的文件去重。

| 分类 | 视频数 | 观看与下载 |
|---|---:|---|
| 强化学习部署与场景切片 | 27 | [6卷下载与合并](media/README.md#rl-video-parts) |
| 自主导航与SLAM系统素材 | 4 | [下载ZIP](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-navigation_slam-01.zip) |
| MATLAB控制导航 | 2 | [下载ZIP](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-matlab_control-01.zip) |
| 毕业答辩演示 | 6 | 本页视频1 + [其余5段](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-defense_demo-01.zip) |
| 诊断与对比剪辑 | 13 | [下载ZIP](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-diagnostics-01.zip) |

逐文件来源、剪辑关系和哈希见[视频目录](media/README.md)。Release的10项资产均已核对GitHub端大小与SHA256，详见[校验记录](provenance/release-assets.json)。

## 仓库结构

| 目录 | 内容 |
|---|---|
| `ros1/python_nodes/rl_planner` | 早期多节点实车部署包 |
| `ros1/local_planner_plugin/rl_planner` | C++ move_base 局部规划器插件源码 |
| `ros1/runtime_reference` | 真机导航参数、TF与话题参考快照 |
| `models_and_export` | 模型、外置权重、导出脚本及观测接口 |
| `training/stage1` | 训练代码与配置、代表性检查点及训练日志 |
| `experiments` | 仿真/实车数据、诊断和统计脚本 |
| `slam`、`matlab` | SLAM基础、地图、MATLAB与Simulink实验 |
| `docs`、`documents` | 技术说明、问题处理记录及原始项目文档 |
| `literature` | 相关论文书目与阅读笔记 |
| `media` | 首页视频1、分类清单、分卷合并脚本及Release下载入口 |
| `archive` | 有实质差异的历史节点与插件版本 |

## 阅读与复现顺序

1. 先读[系统总览](docs/overview.md)和[项目经历](docs/project-journey.md)，区分实车部署与插件改造阶段。
2. 根据[模型与部署说明](docs/training-and-deployment.md)配对观测维度、normalizer、外置`.onnx.data`及动作参数。
3. 使用目标版本的ROS包、launch和真实硬件配置；归档中部分入口存在缺文件或版本差异，不能假定任意分支开箱即用。
4. 运行 `python experiments/analysis/analyze_episodes.py` 复核四组episode统计；这一检查无需ROS或模型推理。
5. 结合[视频清单](media/README.md)、原始运行日志和[验证边界](docs/limitations-and-validation.md)解释实验结果。

此次整理完成了文件复制哈希校验、模型接口核对、视频去重和数据重新计数，没有重新运行ROS、训练或驾驶小车。原有实验完成情况与本次复核范围分别记录。

相关代码保留原有版权和许可标识，第三方论文以[书目及阅读笔记](literature/README.md)引用。整理日期：2026-11-15。
