# MATLAB与Simulink实车控制实验

本阶段已完成MATLAB连接ROS1小车、激光与地图数据读取、Pure Pursuit路径跟踪，以及Simulink与ROS的闭环控制联调。工作总结和实验记录保存了程序设计过程、运行附件、地图导出问题和终点绕行分析。本目录保留这些已做实验对应的脚本与模型；此次仓库整理读取源码和模型XML，没有重新启动MATLAB或驱动小车。

| 文件 | 已完成的工作 |
|---|---|
| [scripts/test1.m](scripts/test1.m) | MATLAB建立ROS连接，订阅/odom、计算Pure Pursuit速度、发布/raw_cmd_vel并显示路径与车体姿态 |
| [scripts/test2.m](scripts/test2.m) | 订阅/scan，将测距和角度构成lidarScan并绘制扫描 |
| [scripts/test3.m](scripts/test3.m) | 订阅/map，读取OccupancyGrid并显示地图 |
| [simulink/exp_6_2020b.slx](simulink/exp_6_2020b.slx) | ROS消息订阅、tf_pose姿态转换、Pure Pursuit、到达判定、零速门控和Twist发布组成的实车控制模型 |
| [simulink/build_ros_model.sh](simulink/build_ros_model.sh) | 保留的MathWorks模型生成节点构建脚本，面向导出的tgz和catkin工作空间 |
| [simulink/model-inspection.json](simulink/model-inspection.json) | 直接从保存模型XML读取的版本、接口和关键参数，不是一次新运行日志 |

MATLAB脚本路径为`[0,0; -2,0; -2,1]`；Simulink模型路径为`[0,0; -3,0; -3,2.8]`。两者保存的期望线速度均为0.2m/s、最大角速度0.3rad/s、前瞻距离0.3m，到点距离阈值0.2m。脚本期望循环间隔和模型固定步长均为0.02s；这不代表实车话题实测频率已经达到50Hz。

早期实验还尝试过0.01m前瞻，记录了目标附近持续绕行与小前瞻引起的抖动。后续保存模型将到达判定单独接入两路速度Switch和Stop Simulation，明确区分路径跟踪与终点停车。

```text
/odom → 位姿和四元数 → [x,y,yaw] → Pure Pursuit
预设waypoints ──────────────────↗
到点距离判定 → 零速门控 → geometry_msgs/Twist → /raw_cmd_vel
```

test1的位姿更新实际使用/odom消息，差速模型积分代码处于注释状态。Simulink由/odom的IsNew触发控制子系统。这一阶段完成的消息、坐标、闭环控制和终点处理，构成后续RL策略真实部署的工程基础。

模型虽名为exp_6_2020b，保存元数据为**R2022b**，目标配置仍含`/opt/ros/melodic`，属于早期ROS1快照。后续RL资料使用Noetic；复用时应按实际环境配置，不能仅凭文件名判断兼容性。

完整过程、问题处理、接口差异及依赖说明见[MATLAB实验详述](../docs/matlab-experiments.md)。
