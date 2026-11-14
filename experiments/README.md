# 实验资料

这里保存与Zeus S2小车导航项目直接相关的实验数据、ROS运行记录和历史诊断材料。原始文件保留其字段、数值及目录命名；新增的说明和统计将不同实验批次分开，未重新训练策略或复现实车运行。

| 目录 | 内容 |
|---|---|
| [datasets](datasets/) | 900回合仿真对照CSV、实车summary与obs_556、22张实验截图 |
| [real_robot_2026-04-28](real_robot_2026-04-28/) | 8次trial、话题/频率/TF/参数/延时记录、控制源与部署状态、实物和链路图 |
| [diagnostics_2026-05-10](diagnostics_2026-05-10/) | 21张历史诊断界面截图 |
| [diagnostics_2026-05-11](diagnostics_2026-05-11/) | 离线诊断脚本、原始报告、JSON和绘图；使用前应阅读其限制 |
| [dwa_navigation_2026-06-22](dwa_navigation_2026-06-22/) | Navfn全局路径、DWA局部/全局参考路径导出的CSV |
| [analysis](analysis/) | 从原始资料提取的汇总和可以重新运行的episode统计脚本 |

## 仿真对照

每份episode文件均为900条、episode_id唯一0至899。下表按outcome全量计数，未移除碰撞或超时行。

| 原始文件 | success | collision | timeout | 成功率 |
|---|---:|---:|---:|---:|
| rollout_episode_esa_on.csv | 814 | 73 | 13 | 90.44% |
| rollout_episode_esa_off.csv | 266 | 591 | 43 | 29.56% |
| rollout_episode_dwa.csv | 124 | 229 | 547 | 13.78% |
| 0414对标ESA设置的rollout_episode_dwa.csv | 189 | 344 | 367 | 21.00% |

两份DWA是不同批次/设置的结果，不能合并成同一基线的1800回合。四份episode文件按episode_id逐一核对时，scenario_name、speed_tier、start_dist_m一致；这不代表其他感知、速度约束或终止配置也已经全部核对一致。每份还配有step级CSV。

重新统计（仅Python标准库）：

```bash
python experiments/analysis/analyze_episodes.py
```

脚本默认输出到终端；需要保存时可传 `--output experiments/analysis/episode_statistics_recomputed.json`。路径从脚本所在目录推导，可以在仓库任意工作目录调用。

## 实车记录的解释

保存的summary共75份，其中RL39份、DWA36份，start_time_sec互不相同但experiment_id存在复用。所有summary都写success=1，同时有12个所在目录名含pengzhuang。该字段主要反映记录器的到达终态，不能作为“75次均无碰撞成功”的证据。原目录拼写如diagnol、yvle、pengzhuang予以保留，便于追溯。

2026年4月28日8次trial均记录到运动，6次出现goal_reached。其自动记录停止条件是运动后速度稳定为零；6/8是到达状态出现比例，不能直接改称标准化的导航成功率。话题间延时还受异步匹配、控制源切换与等待影响，不能当作纯ONNX算子耗时；/zeus_speed在保存配置中存在速度平滑器输出映射，不应未经核实称为独立底盘测量反馈。

论文初稿另记DWA/ESA_OFF/ESA_ON到达率0.746/0.343/0.919，碰撞率均0。它们尚未与上述900回合CSV建立批次和过滤口径对应，不能拼接为同一组结果。

## 资料来源与边界

源目录相对于原始知识库根：`07 Assets/D0-Datasets`、`02 Projects/D0-Robotics-Experiments/1/{4.28,5.10,5.11}`和`02 Projects/D0-Navigation-Research`。重复ZIP、演示视频和大型bag不放入这个实验目录。正文日志中的原始ROS机器路径和运行命令保留，用于解释当次环境；它们不是当前安装步骤。

新增JSON中的path使用本仓库相对路径。以source:开头的值表示原始知识库相对来源、未在该目录复制的资源。历史诊断中的“发现”是原报告提出的假设，不能自动当作已验证的策略错误。
