# 实车硬件平台与传感器接口

本项目整机产品型号为 **VSAI-FWD-B**，ROS底层功能包和机器人模型采用 **Zeus S2 / `zeus_s2`** 命名。两者分别是产品标识和软件平台标识。下表结合发货设备清单、项目论文和实车运行记录整理。

设备量程、标称输出率、实测ROS频率、工控机内存/存储/接口、电机编码器及电池参数集中见[完整参数表](hardware-parameters.md)，其中区分设备规格、配置声明和实测结果。

## 硬件配置

| 模块 | 型号或已确认配置 | 本项目中的作用与接口 | 依据与说明 |
|---|---|---|---|
| 移动底盘 | VSAI-FWD-B，四轮独立驱动、差速转向；ROS模型为Zeus S2 | 执行上层线速度、角速度指令，支持前进、后退及转向 | 发货设备清单；[项目论文](../documents/thesis-draft.docx)第3章平台介绍 |
| 上位主控计算机 | **GK4000，Intel工控机**；CPU具体型号未记录 | 运行ROS、传感器驱动、定位导航和ONNX策略推理 | 发货清单明确GK4000；[实车硬件快照](../experiments/real_robot_2026-04-28/9-传感器延时/04_system_end_to_end_delay_chain/01_sensor_delay_all_categories/hardware_runtime_snapshot_20260428_213011.txt)确认x86_64架构、约7.7GiB系统可见内存 |
| 二维激光雷达 | 随附清单标记**RPLIDAR A3M12** | `rplidar_ros`驱动，`/rplidarNode`发布`/scan`（`sensor_msgs/LaserScan`）；用于定位、避障及RL激光观测 | [A3启动配置](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/11_rplidar_launch_files.txt)包含`rplidar_a3.launch`、256000波特率及`laser_link`；[实际扫描字段](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/07_scan_key_fields.txt)可核对当次数据；厂商系列规格与版本差异见[参数详表](hardware-parameters.md) |
| 惯性测量单元 | **N100 IMU，带金属外壳**；内部芯片型号未记录 | `fdilink/launch_imu_node`接入`/dev/imu`，发布`/imu/data`（`sensor_msgs/Imu`）；提供姿态、角速度和线加速度 | 设备清单明确N100；[IMU启动配置](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/17_imu_launch_files.txt)、[实际接收记录](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/19_imu_python_receive.txt)保存`fdi_imu`坐标系数据 |
| 轮速与轮式里程计 | 电机内置光电编码器，具体器件型号未记录 | 平台按轮速及差速运动学估计运动；定位节点配置包含`/odom_wheel`输入 | [论文](../documents/thesis-draft.docx)说明电机内置光电编码器；[节点记录](../experiments/real_robot_2026-04-28/3-other-message/other-message/other-message/15_related_node_info.txt)显示`/relocalization`订阅`/odom_wheel`，但该快照未确认其消息类型和有效数据流 |
| 底盘驱动与下位控制 | `zeus_s2_driver`软件驱动；下位机板卡、MCU及电机驱动器的实车具体型号未明确 | 将控制源切换和平滑链路的速度指令接入底盘执行层 | [节点与连接记录](../experiments/real_robot_2026-04-28/3-other-message/other-message/other-message/15_related_node_info.txt)；驱动包名不能替代硬件板卡型号 |
| 电机、减速器与电源 | 资料列4×600W直流无刷电机、1:40减速器、48V/22Ah锂电池及整机外供电 | 电机内置光电编码器参与轮速测量；供电含24V/1A和12V/1A输出 | 厂家整机参数见[参数表与来源](hardware-parameters.md) |
| USB-CAN及串行桥 | 系统枚举USBCAN/CANalyst-II、FT232和CP210x USB-UART设备 | 提供底盘/外围通信连接；物理接线和CAN速率仍需核对 | [设备枚举记录](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/03_imu_serial_devices.txt) |

主控计算机的CPU完整SKU、下位机MCU和编码器型号尚未出现在已核实记录中，因此不以同系列产品的常见配置补写。

## 传感器与策略输入的分工

- **激光进入策略观测。** 原始`/scan`先修正180°前向偏差，再裁剪270°视场、重采样181束、按12米归一化并堆叠三帧。训练与部署尺度失配曾引发异常后退，其RViz排查与修正过程见[部署问题案例](deployment-problem-cases.md#normalization-mismatch)。
- **IMU主要服务状态估计。** 实车节点记录包含`/relocalization`订阅`/imu/data`。保存的RL部署分支使用固定重力投影`[0,0,-1]`，不能据平台安装IMU就把策略重力项写成实时IMU测量；具体观测契约见[训练与部署接口](training-and-deployment.md)。
- **`/odom`是估计结果话题。** [实际话题记录](../experiments/real_robot_2026-04-28/3-other-message/other-message/other-message/07_odom_info.txt)显示其类型为`nav_msgs/Odometry`、发布者为`/tf_pub_node`。它不对应一个独立的“里程计传感器型号”，也不能仅据名称认定为未经定位处理的编码器原始输出。
- **关节状态不等于编码器实测。** 当次节点快照中的`/joint_states`由`joint_state_publisher_gui`发布，用于模型关节显示；不能将它作为轮速编码器有效输出的证明。

平台资料另列超声波避障接口，但未明确物理设备型号；现有RL观测契约也未包含超声波输入。发货资料中的机械臂、三指夹爪、北斗和三维激光等扩展内容，不列为本项目已确认的RL实验配置。

## 实车系统与后续开发目标

| 阶段 | 已确认环境 | 解释 |
|---|---|---|
| 2026-04-28实车运行 | **Ubuntu 18.04系、ROS1 Melodic、x86_64** | 硬件快照的`uname`及`ROS_DISTRO=melodic`，并有`/opt/ros/melodic`运行进程；这是实车RL采集阶段的系统记录 |
| 后续局部规划器插件 | **ROS1 Noetic开发目标** | 研究讨论与插件说明采用Noetic；[工作总结](../ros1/local_planner_plugin/rl_planner/docs/2026-06-19_work_summary.md)明确尚未在目标ROS/catkin环境真实编译，不能反推4月实车已迁移Noetic |

不同阶段仍属于ROS1部署工作，复现时应选择对应代码、模型及运行环境，避免混用快照。

## 型号来源与追溯

原始知识库根目录为`D:\2-Knowledge`。设备型号依据为以下相对路径中的发货文档：

```text
02 Projects/D0-ROS-Robot-Car/智能小车/智能小车/智能小车/
  发货资料/v01.01.01.0000/01.开发文档/
    开源四轮差速智能移动机器人 -产品使用说明书.pdf
    开源四轮差速智能移动机器人-产品介绍.docx
```

使用说明书PDF第5—6页（正文页码2—3）设备清单与平台参数列出VSAI-FWD-B、GK4000、RPLIDAR A3M12及N100；产品介绍的“设备清单”表1提供相同型号。知识库中的`02 Projects/D0-Robotics-Experiments/1/4.28/0-问题回答/问题回复.docx`也记录了本次实车的上述配置。此页提炼项目硬件信息，不重复上传整份厂商手册。
