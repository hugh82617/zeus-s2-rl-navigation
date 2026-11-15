# 硬件参数、通信接口与实测频率

本页补充[硬件平台清单](hardware-platform.md)中的具体参数。**厂家资料规格、ROS接口声明、实测发布频率和策略预处理参数分别标注**：设备支持的最高输出率不等于本次ROS话题频率，驱动量程也不等于策略归一化尺度。

## 1 GK4000工控机：计算、存储与接口

下表以[2026-04-28实车硬件快照](../experiments/real_robot_2026-04-28/9-传感器延时/04_system_end_to_end_delay_chain/01_sensor_delay_all_categories/hardware_runtime_snapshot_20260428_213011.txt)为依据。

| 项目 | 已保存参数 | 参数性质 |
|---|---|---|
| 主机 | GK4000，Intel工控机，x86_64 | 型号见发货清单；CPU具体SKU、核数和主频未保存 |
| 内存 | 系统可见约7.7GiB；交换空间2.0G | `free -h`实测；未记录DDR代际、频率及内存条型号 |
| 系统存储 | `/dev/sda2`根分区117G，采集时已用23G、可用89G；EFI分区511M | `df -h`记录的是文件系统/分区容量，不能据此认定为128GB SSD或确定磁盘介质 |
| 外接存储 | `/dev/sdb1`挂载分区57G；USB枚举出现Kingston DataTraveler G4 | 采集时外设记录，与机内系统盘分别记录 |
| 网络 | 有线接口`enp2s0`、`enp3s0`；无线接口`wlp4s0` | 系统枚举；网卡型号、链路速率及完整物理插孔数量未保存 |
| USB | USB2.0/USB3.0总线；FT232及CP210x USB-UART桥、USB Hub | 总线及外设枚举，不据此推算主机物理USB孔数量 |
| 串口设备 | 系统枚举`ttyACM0`、`ttyUSB0`、`ttyUSB1`；ROS参数指定`/dev/imu`、`/dev/laser`、`/dev/odom`设备路径 | 设备枚举及软件配置；不是主机原生物理串口数量 |
| 系统 | Ubuntu 18.04系、ROS1 Melodic | 对应该次实车RL运行；后续Noetic插件开发目标另行保留 |

整机后面板资料另列**USB2.0、网口、SCI二次开发接口、充电接口和外供电接口**，车体有总闸、急停及快换电池仓。它们是整机接口，不全部归为GK4000主板端口；显示接口、原生串口数量与工控机完整配置仍需实物/装机清单确认。面板依据为产品使用说明书PDF第9页（正文第6页），来源见[硬件资料追溯](hardware-platform.md#型号来源与追溯)。

## 2 二维激光雷达：测距、采样和ROS输出

项目发货清单标记为**RPLIDAR A3M12**。官方A3系列页面用于参数参考，具体硬件修订版与本机清单标记的对应关系尚未由铭牌/序列号确认。

| 参数 | 官方A3系列参考规格 | 本项目实际ROS记录/配置 |
|---|---|---|
| 最大测距 | 增强模式：白色物体25米、深色物体10米；室外模式白色物体20米 | LaserScan声明`range_max=16米`，不是现场已测得最远16米 |
| 最小测距 | 0.2米 | LaserScan声明`range_min≈0.15米`，不据此重新定义物理盲区 |
| 测距采样率 | 增强模式16kHz；室外模式10kHz | 当前记录未直接测得硬件原始采样率 |
| 扫描频率 | 典型10Hz，可调5–15Hz | 单独`rostopic hz`窗口最终13.315Hz；8次运行窗口另见第5节 |
| 扫描角度/角间隔 | 360°；参考角分辨率0.225° | 角度补偿开启，数组1440束；`angle_increment≈0.0043542264rad≈0.24948°` |
| 单帧时间字段 | 随模式/转速变化 | 保存单帧`scan_time≈0.072929秒`；不以单帧倒数替代整段发布率统计 |
| 通信 | 3.3V TTL UART，256000bps | `/dev/laser`，256000bps，`rplidar_ros`，`scan_mode=Sensitivity` |
| 供电 | 5V，450–600mA，2.25–3W | 厂商参考规格，未保存本机电流实测 |
| 数据出口 | — | `/rplidarNode → /scan`，`sensor_msgs/LaserScan`，frame为`laser_link` |

官方规格来源：[SLAMTEC A3 Measurement Performance](https://www.slamtec.com/en/lidar/a3spec)。实际参数见[扫描关键字段](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/07_scan_key_fields.txt)、[驱动参数](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/12_lidar_rosparams.txt)及[独立频率记录](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/06_scan_hz.txt)。1440是驱动输出长度，不能用1440乘话题频率推断硬件原始采样率。

**随附资料差异。** 原产品资料及[早期实验记录表2](../documents/early-experiment-record.docx)还保存0.2–18米、2–8.01kHz及0.45°–1.35°的另一组表格，与官方A3页面不一致，作为原资料保留，不混为同一版设备的确定规格。

**进入策略的参数。** 方向修正后裁剪为−135°至135°，按1.5°重采样181束，以12米截断/归一化，再堆叠三帧543维。这是模型输入契约，独立于厂商最高测距和驱动16米声明量程。16→12归一化失配的排查见[异常后退案例](deployment-problem-cases.md#normalization-mismatch)。

## 3 N100 IMU与超声波

| N100参数 | 数值或配置 | 依据/性质 |
|---|---|---|
| IMU数据输出频率 | 标称400Hz | 随附产品资料表4，不代表本次ROS发布率 |
| 导航数据输出频率 | 标称200Hz | 同一设备资料中的另一类输出数据 |
| 本次`/imu/data`频率 | 100.002Hz | [独立实测窗口](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/08_imu_data_hz.txt)最后一次统计值 |
| 串口波特率 | 资料标称921600bps，可调 | [实车launch](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/17_imu_launch_files.txt)只设置设备路径，未显式记录实际波特率 |
| 通信与封装 | Type-C；资料列TTL/RS485/CAN预留能力 | 不等于本机已接装所有物理通信接口 |
| 电源 | 5V，最大功率400mW | 随附产品资料 |
| 实车数据 | `/dev/imu`，`fdilink/launch_imu_node`发布`/imu/data`，frame为`fdi_imu` | 四元数、三轴角速度及线加速度见[实际接收记录](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/19_imu_python_receive.txt) |
| 尚缺参数 | 陀螺仪/加速度计量程、噪声密度、零偏稳定性、姿态精度 | 本项目已核实资料未明示，不以其他产品或商家参数代填 |

| 超声波避障模块参数 | 随附产品资料值 |
|---|---|
| 接口与供电 | RS485，DC9–36V，工作电流小于36mA |
| 测量距离与方向角 | 28–450厘米，55° |
| 声学载波频率 | 40kHz；这是发射声波频率，不是测距更新率或ROS发布率 |
| 分辨率 | 原文`1+S*0.3 cm`，未说明S定义，不作为固定精度解释 |
| 尚缺参数 | 精确器件型号、装机数量、实际测距更新率及当次有效数据流未确认 |

IMU与超声波参数取自产品介绍表3、表4；[早期实验记录](../documents/early-experiment-record.docx)也保存相应参数表。实验指导书另有25–450厘米、1厘米分辨率等不同表述，按资料版本保留。现有RL观测不含超声波输入；IMU服务状态估计，部署策略的重力投影仍按保存代码固定为`[0,0,-1]`。

## 4 电机、编码器、底盘通信和供电

| 部件 | 资料参数与本项目记录 | 完成程度/作用 |
|---|---|---|
| 驱动电机及减速器 | 4×600W直流无刷电机，减速比1:40 | 厂家整机参数；电机具体型号、驱动器型号及驱动器数量未明确 |
| 光电编码器 | 每台驱动电机内置光电编码器，四台电机各有轮速测量部件 | [论文平台章节](../documents/thesis-draft.docx)明确记载；不能把编码器省略为一个抽象`odom`话题 |
| 编码器计数参数 | ROS存在`encoder_resolution_4wd`、`gear_reduction_4wd`、`wheel_diameter_4wd`、`wheel_track_4wd`键 | [参数名称列表](../experiments/real_robot_2026-04-28/1-robot-message/robot-message/robot-message/07_rosparam_list.txt)只保存名称；PPR/CPR、倍频和实际计数周期未保存 |
| 轮式里程计 | 轮速结合底盘运动学估计位移/转角，定位链配置含`/odom_wheel` | 当次`/odom`由`/tf_pub_node`发布；其约14Hz输出率不代表编码器硬件采样频率 |
| 底盘控制板/驱动器 | `zeus_s2_driver`为软件执行接口；下位机板卡、MCU及驱动器具体型号未确认 | 电气图中的机械臂主控板不作为本车底盘型号 |
| 底盘串行配置 | `/dev/odom`，`baud_rate=115200` | [实际ROS参数](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/15_imu_rosparams.txt)；与各USB设备的物理对应关系需按装机接线核对 |
| USB-CAN设备 | USB描述为Chuangxin Tech USBCAN/CANalyst-II，VID/PID `04d8:0053` | [内核枚举记录](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/03_imu_serial_devices.txt)还出现`zeus_s2_driver`访问该USB设备的记录；未保存CAN总线速率 |
| 底盘几何与运动能力 | 912×710×410毫米，轴距500毫米，轮距555毫米，最小离地间隙105毫米；最大车速1米/秒，可原地转向 | 厂家平台规格，不等于本次策略速度限幅，也不直接替代训练模型几何参数 |
| 电池与外供电 | 48V、22Ah锂电池；外供24V/1A、12V/1A；独立AC220V充电器 | 厂家整机参数；车体配有急停、总闸与充电接口 |
| 遥控与采集外设 | 实车USB枚举有Xbox360手柄、USB-UART桥、USB Hub及Kingston U盘 | 手柄用于人工控制/接管，外设列表对应采集时系统状态 |

电机、底盘尺寸及电源规格见[早期实验记录](../documents/early-experiment-record.docx)和[原始硬件资料来源](hardware-platform.md#型号来源与追溯)。`/joint_states`当次由GUI发布，不能据其约10Hz推定编码器更新率。

## 5 各话题的实测频率与采集口径

独立检查窗口最终统计：`/scan=13.315Hz`、`/imu/data=100.002Hz`、`/odom=14.070Hz`，见[雷达](../experiments/real_robot_2026-04-28/2-lidar-message/lidar-message/lidar-message/06_scan_hz.txt)、[IMU](../experiments/real_robot_2026-04-28/3-other-message/imu-check-message/imu-check-message/08_imu_data_hz.txt)、[里程计](../experiments/real_robot_2026-04-28/3-other-message/other-message/other-message/07_odom_hz.txt)原始日志。

以下另统计8次运行各自保存的`avg_hz`，表中给出**试次发布率的中位数及最小—最大值**，不合并消息流；与独立窗口分别记录。

| 话题 | 中位数Hz | 试次范围Hz | 解释 |
|---|---:|---:|---|
| `/scan` | 14.286 | 14.119–14.374 | 原始雷达扫描话题 |
| `/scan_aligned` | 14.280 | 14.066–14.398 | 前向修正后的扫描 |
| `/rl/lidar_181` | 14.286 | 14.273–14.380 | 单帧观测发布率 |
| `/rl/lidar_543` | 14.285 | 14.274–14.407 | 三帧堆叠输出率，不是三倍扫描率 |
| `/rl/obs_556` | 13.691 | 12.188–14.343 | 策略输入组装/发布率 |
| `/rl_cmd_vel` | 13.710 | 12.219–14.368 | 策略命令输出率，不是纯ONNX计算速度 |
| `/odom` | 14.277 | 14.220–14.407 | 定位/里程计输出率 |
| `/zeus_speed` | 19.978 | 19.458–20.004 | 该快照涉及速度平滑命令，不能直接称硬件反馈率 |
| `/joint_states` | 10.001 | 9.994–10.068 | GUI模型关节状态，不能作为编码器实测 |

8次频率CSV未给出`/imu/data`、`/odom_wheel`和`/motor_driverFeedback`的有效发布率；IMU另有上述100Hz独立日志，其他项不补推数值。超声波、编码器内部采样和CAN总线报文率也尚无对应实测值。

统计来自[原始8次频率记录](../experiments/real_robot_2026-04-28/4-hz-message/01_frequency_information/)，逐试次数值与来源见[JSON汇总](../experiments/analysis/hardware-runtime-summary.json)。在仓库根目录运行即可复核：

```bash
python experiments/analysis/summarize_hardware_runtime.py
```

## 6 尚需实车补录的原始参数

缺失项集中为CPU完整型号/核数/主频、内存条规格、系统盘介质与型号、完整物理端口清单、编码器PPR/CPR及倍频、底盘MCU/电机驱动器型号、IMU量程/精度，以及超声波实际更新率。这些没有在现有材料中得到可靠数值。

后续可在实车只读采集`lscpu`、`lsblk -d -o NAME,MODEL,SIZE,ROTA,TRAN`、`lspci -nn`、`lsusb`和`rosparam get /zeus_s2_driver`，并核对电机/编码器铭牌与装机清单。ROS参数的编码器分辨率还需结合计数倍频、减速比和单位解释，不能直接等同于器件PPR。
