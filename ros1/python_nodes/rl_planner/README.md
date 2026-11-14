# Python多节点部署快照

链路为 `/scan → laser_switch → lidar_obs_single_frame → lidar_stack_543 → rl_planner_node → /rl_cmd_vel`。目标来自 `/move_base_simple/goal`，通过TF转换成车体系4维目标；速度、固定重力、goal4和543维激光拼为556维输入。

激光主配置为181束、[-135°,135°]、三帧按[t0,t1,t2]排列，距离按12m归一化。实际参数仍须同时检查源码默认和launch覆盖。

`models/model_1200_actor.onnx`与 `.onnx.data` 为配套文件。本目录保持历史源码，未在本次整理中执行ROS或ONNX推理。已发现 `launch/lidar_obs_single_frame.launch` 引用的 `config/lidar_obs_single_frame.yaml` 和 `lidar_obs_single_frame(2).py` 在此快照中缺失；不要据此宣称开箱可运行。需要在对应机器人环境核对入口、依赖和源文件。

模型normalizer已随导出图保存，部署动作另有clip/线性映射。`/zeus_speed`在历史运行参数中可能映射为平滑控制命令，不能未经核查当作独立硬件实测反馈。
