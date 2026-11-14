# 智能小车导航相关研究与阅读笔记

整理日期：2026-11-15。本页保留51份论文归档对应的题名、方法摘要与主题关系，按研究主题归并43项。仅发布书目和阅读笔记，不附第三方PDF或提取全文。

## 1. 覆盖与阅读边界

- 共 51 份 PDF、663 页；51 份均提取到正文，没有整份扫描无文字层的文件；逐份核读题名、摘要或首面概述，对关键论文补读方法与结论。
- 按 SHA256 检出 5 组完全相同副本，合并后为 46 个不同PDF文件；另有2个中文版本和1组同题预印本/期刊版，按研究主题归并为43项（不表示版本内容完全相同）。
- 本地整理保留文件SHA256与页数，公开书目见 [bibliography.csv](bibliography.csv)。副本不重复计为独立研究。
- 文字提取成功不等于每个公式/表格完整。DWA原文字符分散；提取器曾报告少数字体缺少fontTools及个别form内容跳过，公式、图表和数值引用应回原PDF核对。本报告主要提炼正文方法，不据文字层声称验证全部图表。
- 下文“可借鉴”是结合小车导航任务的分析，不等于当前项目已采用该方法；论文所报告的效果也不等于本地小车实验结果。

## 2. 与现有小车导航最接近的研究链

| 研究环节 | 优先资料 | 已核实内容与可用线索 |
|---|---|---|
| 激光到连续速度、仿真到实车 | PDF10 Virtual-to-real、PDF11 CrowdMove | 稀疏激光/相对目标、自身速度；连续控制；多机器人、多场景、多阶段训练。可核对激光观测与目标坐标约定，但不能直接把论文输入维度套到556维策略。 |
| 激光时序与动态障碍 | PDF15 iTD3-CLN、PDF45 PL-TD3 | CLN按通道组织激光、N步回报/PER；LSTM与PER结合TD3。适合与多帧激光堆叠比较，区分时间记忆和纯单帧感知。 |
| 稠密奖励与运动平滑 | PDF07 HDRS | 用APF先验构造训练奖励，包含目标前进/朝向、障碍距离/方向、角速度变化；DDPG实验与受控噪声增强。它是训练端改动，外层限速/停车不等同于HDRS。 |
| 动态碰撞预见 | PDF14 DRL-VO、PDF40 DWA-RL | 前者需要行人运动学并引入VO奖励，后者在动作/观测中兼顾动力学可行性与障碍运动。仅有距离激光不代表已经具备这些运动信息。 |
| 全局路线+RL局部策略 | PDF20 Arena-Rosnav、PDF22 Hybrid Indoor Social Navigation | 传统全局规划提供路径/航点，RL负责局部跟随避障；可用于解释全局路线、局部目标与RL策略分工。 |
| DWA与RL组合 | PDF05/16 Hybrid Classical/RL、PDF06 Improved DWA | 一种是在完整规划器之间切换，另一种是让TD3补充候选轨迹并由DWA统一评价；两者不能混为同一算法。 |
| 里程计与定位可靠性 | PDF41 Localisation-Safe、PDF35 Holistic Fusion | 一个通过奖励和历史轨迹减少定位失败，另一个通过因子图多传感融合改善状态估计；后者本身不训练RL策略。 |
| 公平评测与泛化 | PDF24/27/29 Arena系列、PDF25 DynaBARN、PDF26 RL Benchmark、PDF28 BARN竞赛 | 统一场景、障碍运动模式、未知环境、真实机器人部署及失败统计。适合建立传统方法/RL方法对比与消融。 |

### HDRS奖励笔记与原论文的关系

PDF07第4.2节（式8-10）以 `r_t = U_att + U_rep` 描述奖励势场：目标在前方时按相邻时刻目标距离变化给奖励，目标在后方时引入朝向和目标距离；障碍在安全范围内时按距离及方向罚分，范围外则惩罚角速度突变。第4.3节明确采用DDPG展示HDRS，声称该奖励思想可用于其他Actor-Critic算法。第6节总结了训练效率、传感器噪声增强与实车部署。

目录内另有 `启发式密集奖励塑造_4.2_HDRS奖励函数笔记.md`，其总体“APF先验→奖励→DRL”解释与原文相符。但系数正负号、角度β定义、分段边界和终止奖励不能只照抄笔记；尤其“正前方/侧后方”必须结合论文角度定义及代码核验。论文的密集奖励也不自动构成全局安全证明。

### 实验与实现时应明确区分的内容

1. **无地图局部控制与完整系统无地图不同。** 局部RL策略可以只用激光、目标和速度；整套长期导航仍可由地图/全局规划器提供路线或子目标。
2. **RL模型与部署包裹不同。** 动作限幅、速度平滑、安全停车、控制源切换、ROS插件封装，是部署系统的一部分；训练奖励、网络结构、数据采样才决定学习方法本体。
3. **动态环境证据不同。** 只在静态障碍附近转向，不足以证明理解动态障碍速度。可借DynaBARN改变速度、加减速和运动平滑度，并留出未见运动分布。
4. **基线需要同任务条件。** 目标、起终点、速度和加速度限制、机器人尺寸、感知范围及终止规则应一致，再比较到达率、碰撞、时长、路长、障碍间距、速度平滑和计算耗时。

这些是本次根据资料提出的对照维度，未在此执行新实验。

## 3. 数字命名1.pdf至12.pdf的真实内容

全部位于 `05 Papers\D0-References\参考论文`，以下题名来自PDF正文，而非文件名猜测。

| 原文件 | 核实题名 | 定位 |
|---|---|---|
| PDF 20：1.pdf（原归档：`05 Papers/D0-References/参考论文/1.pdf`） | Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems | 核心：RL接入导航栈；将DRL局部避障规划器接入传统全局导航，解决纯局部策略长期导航/局部极小值问题；提供训练测试与传统方法比较框架。 |
| PDF 24：2.pdf（原归档：`05 Papers/D0-References/参考论文/2.pdf`） | Arena-Bench: A Benchmarking Suite for Obstacle Avoidance Approaches in Highly Dynamic Environments | 核心：实验平台；ROS集成的3D动态障碍训练、测试、评估套件，可比较传统规划与学习规划、多机器人平台并进行真实部署。 |
| PDF 25：3.pdf（原归档：`05 Papers/D0-References/参考论文/3.pdf`） | DynaBARN: Benchmarking Metric Ground Navigation in Dynamic Environments | 核心：评测基准；系统改变动态障碍运动速度、平滑度和加减速等motion profiles；难度指标生成场景，提供人类示范采集与传统/学习基线。 |
| PDF 26：4.pdf（原归档：`05 Papers/D0-References/参考论文/4.pdf`） | Benchmarking Reinforcement Learning Techniques for Autonomous Navigation | 核心：实验设计；导航RL四目标：不确定性推理、安全、有限试错数据、未知环境泛化；比较记忆网络、安全RL、模型RL、域随机化，包含大规模基准与真实环境研究。 |
| PDF 27：5.pdf（原归档：`05 Papers/D0-References/参考论文/5.pdf`） | Arena-Rosnav 2.0: A Development and Benchmarking Platform for Robot Navigation in Highly Dynamic Environments | 核心：实验平台；模块化重构Arena，统一规划算法/仿真器/评价API，更真实行人及多个仿真器，降低开发训练评测接入难度。 |
| PDF 28：6.pdf（原归档：`05 Papers/D0-References/参考论文/6.pdf`） | Autonomous Ground Navigation in Highly Constrained Spaces: Lessons learned from The 3rd BARN Challenge at ICRA 2024 | 导航竞赛经验；ICRA2024 BARN狭窄密集静态障碍赛事报告；Jackal差速底盘与2D激光，仿真晋级与实物赛，适合实车鲁棒性/失败案例参考，不是单独RL新算法。 |
| PDF 29：7.pdf（原归档：`05 Papers/D0-References/参考论文/7.pdf`） | Arena 3.0: Advancing Social Navigation in Collaborative and Highly Dynamic Environments | 核心：社会导航平台；扩展Arena系列，增强人-人、人-机器人交互与社会力仿真、多任务/动态环境生成，跨三类仿真器支持开发和基准。 |
| PDF 30：8.pdf（原归档：`05 Papers/D0-References/参考论文/8.pdf`） | SICNav: Safe and Interactive Crowd Navigation using Model Predictive Control and Bilevel Optimization | 模型规划对照；双层非线性MPC联合求机器人运动和人群预测；ORCA人类模型嵌入约束并用KKT转化求解，显式考虑交互与安全。非RL。 |
| PDF 31：9.pdf（原归档：`05 Papers/D0-References/参考论文/9.pdf`） | SICNav-Diffusion: Safe and Interactive Crowd Navigation with Diffusion Trajectory Predictions | 学习预测与优化对照；扩散模型产生多人联合多模态轨迹预测，双层MPC同时安全过滤预测与优化机器人轨迹；ETH/UCY开放环及真实机器人闭环测试，非RL控制策略。 |
| PDF 21：10.pdf（原归档：`05 Papers/D0-References/参考论文/10.pdf`） | Dynamic Gap: Safe Gap-based Navigation in Dynamic Environments | 传统动态避障对照；跟踪自由空间间隙而非全局坐标系障碍物；分析间隙穿越/闭合可行性，动态环境安全局部规划。非RL。 |
| PDF 22：11.pdf（原归档：`05 Papers/D0-References/参考论文/11.pdf`） | A Hybrid Approach to Indoor Social Navigation: Integrating Reactive Local Planning and Proactive Global Planning | 核心：全局与局部协同；预测行人位置附近赋全局软代价，生成航点，DRL局部规划跟随航点并避障；面向建筑尺度室内社会导航，附2D基准。 |
| PDF 23：12.pdf（原归档：`05 Papers/D0-References/参考论文/12.pdf`） | Learning to Tune Like an Expert: Interpretable and Scene-Aware Navigation via MLLM Reasoning and CVAE-Based Adaptation | 参数自适应拓展；LE-Nav：MLLM场景理解/语言推理与CVAE将指令映射为导航规划器超参数；智能轮椅验证。方法本体不是常规RL策略训练。 |

## 4. 逐文件提取清单（51份，含副本与译本）

各项方法概述均根据正文摘要/概述核读；“核心”表示任务相关度，不代表论文质量排名。归档名称仅用于来源追溯；可用的DOI链接来自原文件元数据，未重新联网核验。

### PDF 01 · DWAS-RL: A safety-efficiency balanced reinforcement learning approach for path planning of Unmanned Surface Vehicles in complex marine environments

- DOI：[10.1016/j.oceaneng.2024.119641](https://doi.org/10.1016/j.oceaneng.2024.119641)。

- 分类：跨载体RL方法参考。
- 内容：无人水面艇；约束MDP、安全RL、DWA预测危险与安全动作、HER提高样本效率。可借鉴安全动作筛选，不能直接算作小车实验。
- 来源：PDF 01：1-DWAS-RL.pdf（原归档：`05 Papers/D0-References/参考文献/1-DWAS-RL.pdf`）；11页。

### PDF 02 · Deep reinforcement learning with dynamic window approach based collision avoidance path planning for maritime autonomous surface ships

- DOI：[10.1016/j.oceaneng.2023.115208](https://doi.org/10.1016/j.oceaneng.2023.115208)。

- 分类：跨载体RL方法参考。
- 内容：船舶；按DWA的线速度/角速度构造PPO动作空间，将距离、速度、朝向评价引入奖励。适合研究DWA先验如何进入RL。
- 来源：PDF 02：2-Deep reinforcement learning with dynamic window approach based collision.pdf（原归档：`05 Papers/D0-References/参考文献/2-Deep reinforcement learning with dynamic window approach based collision.pdf`）；16页。

### PDF 03 · E3MoP: Efficient Motion Planning Based on Heuristic-Guided Motion Primitives Pruning and Path Optimization With Sparse-Banded Structure

- 分类：传统导航背景。
- 内容：三层框架：状态格全局规划、稀疏带状结构局部路径优化、时间最优速度规划；非RL，可作平滑性/效率对照。
- 来源：PDF 03：2012.08892v3.pdf（原归档：`05 Papers/D0-References/参考文献/2012.08892v3.pdf`）；14页。

### PDF 04 · Escaping Offline Pessimism: Vector-Field Reward Shaping for Safe Frontier Exploration

- DOI：[10.48550/arXiv.2603.18326](https://doi.org/10.48550/arXiv.2603.18326)。

- 分类：RL奖励与安全探索。
- 内容：离线数据训练不确定性oracle；梯度对齐吸引到目标不确定度、旋转流沿不确定性边界运动，避免到边界后停车。SAC二维连续导航实验；不能据此声称实车验证。
- 来源：PDF 04：2026arxiv Escaping Offline Pessimism Vector Field Reward1.pdf（原归档：`05 Papers/D0-References/参考文献/2026arxiv Escaping Offline Pessimism Vector Field Reward1.pdf`）；16页。

### PDF 05 · Hybrid Classical/RL Local Planner for Ground Robot Navigation

- 分类：核心：混合局部规划。
- 内容：传统速度空间规划器负责路径跟踪和平滑，RL规划器擅长动态避障；用环境驱动的元推理切换。与RL/DWA切换和外层部署结构直接相关。
- 来源：PDF 05：2410.03066v1.pdf（原归档：`05 Papers/D0-References/参考文献/2410.03066v1.pdf`）；8页。

### PDF 06 · An Improved Dynamic Window Approach Based on Reinforcement Learning for the Trajectory Planning of Automated Guided Vehicles

- 分类：核心：DWA与RL融合。
- 内容：TD3生成可变采样辅助候选轨迹，与传统DWA候选轨迹共同评价竞争；由DWA保留规划机制，并非简单让策略直接输出最终控制。
- 来源：PDF 06：3-An_Improved_Dynamic_Window_Approach_Based_on_Reinforcement_Learning_for_the_Trajectory_Planning_of_Automated_Guided_Vehicles.pdf（原归档：`05 Papers/D0-References/参考文献/3-An_Improved_Dynamic_Window_Approach_Based_on_Reinforcement_Learning_for_the_Trajectory_Planning_of_Automated_Guided_Vehicles.pdf`）；10页。

### PDF 07 · Heuristic dense reward shaping for learning-based map-free navigation of industrial automatic mobile robots

- DOI：[10.1016/j.isatra.2024.10.026](https://doi.org/10.1016/j.isatra.2024.10.026)。

- 分类：核心：奖励设计与迁移。
- 内容：HDRS把APF目标吸引/障碍排斥先验写成密集奖励，兼顾目标进展、目标朝向、障碍距离方向与角速度变化；DDPG验证，训练加受控传感器噪声，仿真及真实AMR部署。
- 来源：PDF 07：4-Heuristic dense reward shaping for learning-based map-free navigation of industrial automatic mobile robots.pdf（原归档：`05 Papers/D0-References/参考文献/4-Heuristic dense reward shaping for learning-based map-free navigation of industrial automatic mobile robots.pdf`）；18页。

### PDF 08 · Learning to Navigate in Complex Environments

- 分类：RL导航基础。
- 内容：以原始多模态输入学习目标导航，联合深度预测和回环分类辅助任务以提升数据效率；3D迷宫导航，偏视觉/记忆基础。
- 来源：PDF 08：5-LEARNING TO NAVIGATE IN COMPLEX ENVIRONMENTS.pdf（原归档：`05 Papers/D0-References/参考文献/5-LEARNING TO NAVIGATE IN COMPLEX ENVIRONMENTS.pdf`）；16页。

### PDF 09 · Target-driven Visual Navigation in Indoor Scenes using Deep Reinforcement Learning

- 分类：视觉导航拓展。
- 内容：目标条件Actor-Critic，把当前观察和目标同时输入；AI2-THOR收集训练样本，研究跨目标/跨场景泛化及少量微调后的真实机器人迁移。
- 来源：PDF 09：6-Target-driven Visual Navigation in Indoor Scenes using Deep Reinforcement Learning∗.pdf（原归档：`05 Papers/D0-References/参考文献/6-Target-driven Visual Navigation in Indoor Scenes using Deep Reinforcement Learning∗.pdf`）；8页。

### PDF 10 · Virtual-to-real Deep Reinforcement Learning: Continuous Control of Mobile Robots for Mapless Navigation

- 分类：核心：无地图连续控制。
- 内容：10维稀疏测距与机器人坐标系目标位置作为输入，连续转向控制输出；异步深度RL从仿真迁移到非完整约束移动机器人。
- 来源：PDF 10：7-Virtual-to-real Deep Reinforcement Learning：Continuous Control of Mobile Robots for Mapless Navigation.pdf（原归档：`05 Papers/D0-References/参考文献/7-Virtual-to-real Deep Reinforcement Learning：Continuous Control of Mobile Robots for Mapless Navigation.pdf`）；6页。

### PDF 11 · CrowdMove: Autonomous Mapless Navigation in Crowded Scenarios

- 分类：核心：动态避障与训练。
- 内容：3M训练：多机器人、多场景、多阶段；PPO优化无地图导航策略，激光、相对目标、自身速度输入，多种移动平台动态人群测试。
- 来源：PDF 11：8-CrowdMove Autonomous Mapless Navigation in Crowded Scenarios.pdf（原归档：`05 Papers/D0-References/参考文献/8-CrowdMove Autonomous Mapless Navigation in Crowded Scenarios.pdf`）；6页。

### PDF 12 · Benchmarking local motion planners for navigation of mobile manipulators

- 分类：传统导航对照。
- 内容：比较移动机械臂底盘局部轨迹规划器，包括DWA/TEB；适合作为传统控制指标/测试场景参考，非RL算法论文。
- 来源：PDF 12：DWA与TEB.pdf（原归档：`05 Papers/D0-References/参考文献/DWA与TEB.pdf`）；9页。

### PDF 13 · The Dynamic Window Approach to Collision Avoidance

- 分类：传统导航基础。
- 内容：DWA经典原文；速度空间、运动约束与可制动的避障速度筛选，是与RL比较或融合的基础。文字层存在明显字符分散，公式应对照PDF。
- 来源：PDF 13：DWA原文.pdf（原归档：`05 Papers/D0-References/参考文献/DWA原文.pdf`）；23页。

### PDF 14 · DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles

- 分类：核心：动态避障与奖励。
- 内容：短时激光历史、附近行人运动学、子目标作为输入，以速度障碍VO奖励引导避让。包含密集动态仿真和真实机器人测试；输入要求比仅激光的现有策略更丰富。
- 来源：PDF 14：IEEE T-RO DRL-VO.pdf（原归档：`05 Papers/D0-References/参考文献/IEEE T-RO DRL-VO.pdf`）；20页。

### PDF 15 · iTD3-CLN: Learn to navigate in dynamic scene through Deep Reinforcement Learning

- 分类：核心：算法与时序观测。
- 内容：TD3结合N步回报、优先经验回放和按通道组织激光的卷积网络CLN；无地图动态导航及无需进一步微调的实车迁移。
- 来源：PDF 15：NeurocomputingiTD3 CLN Learn to navigate in dynamic scene through Deep.pdf（原归档：`05 Papers/D0-References/参考文献/NeurocomputingiTD3 CLN Learn to navigate in dynamic scene through Deep.pdf`）；11页。

### PDF 16 · Hybrid Classical/RL Local Planner for Ground Robot Navigation

- 分类：核心：混合局部规划。
- 内容：与2410.03066v1同题的2025 Reinforcement Learning Journal版本；应作为同一研究的不同版本，不重复算两种方法。
- 来源：PDF 16：RLJ_RLC_2025_214.pdf（原归档：`05 Papers/D0-References/参考文献/RLJ_RLC_2025_214.pdf`）；15页。

### PDF 17 · ROS Navigation Tuning Guide

- 分类：ROS1工程基础。
- 内容：Kaiyu Zheng，2016；ROS navigation stack参数调优、里程计、传感器、代价地图及局部规划。文件名ROS_nav2.pdf不代表ROS2 Nav2文档。
- 来源：PDF 17：ROS_nav2.pdf（原归档：`05 Papers/D0-References/参考文献/ROS_nav2.pdf`）；23页。

### PDF 18 · Deep reinforcement learning for path planning of autonomous mobile robots in complicated environments

- DOI：[10.1007/s40747-025-01906-9](https://doi.org/10.1007/s40747-025-01906-9)。

- 分类：核心：SAC改进。
- 内容：GAP_SAC：扩充状态空间、动态启发式奖励、PER与门控注意力，针对动态和狭窄环境；与TD3/SAC变体比较，适合算法及消融思路。
- 来源：PDF 18：s40747-025-01906-9.pdf（原归档：`05 Papers/D0-References/参考文献/s40747-025-01906-9.pdf`）；15页。

### PDF 19 · SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation

- DOI：[10.48550/arXiv.2606.19555](https://doi.org/10.48550/arXiv.2606.19555)。

- 分类：邻近领域参考。
- 内容：四足机器人3D导航；偏航感知双圆柱足迹、3D占据查询、投影A*与滑动地图，非小车RL，适合研究几何碰撞建模。
- 来源：PDF 19：空间碰撞感知规划器（上海交通大学）.pdf（原归档：`05 Papers/D0-References/参考文献/空间碰撞感知规划器（上海交通大学）.pdf`）；8页。

### PDF 20 · Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems

- 分类：核心：RL接入导航栈。
- 内容：将DRL局部避障规划器接入传统全局导航，解决纯局部策略长期导航/局部极小值问题；提供训练测试与传统方法比较框架。
- 来源：PDF 20：1.pdf（原归档：`05 Papers/D0-References/参考论文/1.pdf`）；8页。

### PDF 21 · Dynamic Gap: Safe Gap-based Navigation in Dynamic Environments

- 分类：传统动态避障对照。
- 内容：跟踪自由空间间隙而非全局坐标系障碍物；分析间隙穿越/闭合可行性，动态环境安全局部规划。非RL。
- 来源：PDF 21：10.pdf（原归档：`05 Papers/D0-References/参考论文/10.pdf`）；7页。

### PDF 22 · A Hybrid Approach to Indoor Social Navigation: Integrating Reactive Local Planning and Proactive Global Planning

- DOI：[10.48550/arXiv.2506.02593](https://doi.org/10.48550/arXiv.2506.02593)。

- 分类：核心：全局与局部协同。
- 内容：预测行人位置附近赋全局软代价，生成航点，DRL局部规划跟随航点并避障；面向建筑尺度室内社会导航，附2D基准。
- 来源：PDF 22：11.pdf（原归档：`05 Papers/D0-References/参考论文/11.pdf`）；7页。

### PDF 23 · Learning to Tune Like an Expert: Interpretable and Scene-Aware Navigation via MLLM Reasoning and CVAE-Based Adaptation

- DOI：[10.48550/arXiv.2507.11001](https://doi.org/10.48550/arXiv.2507.11001)。

- 分类：参数自适应拓展。
- 内容：LE-Nav：MLLM场景理解/语言推理与CVAE将指令映射为导航规划器超参数；智能轮椅验证。方法本体不是常规RL策略训练。
- 来源：PDF 23：12.pdf（原归档：`05 Papers/D0-References/参考论文/12.pdf`）；13页。

### PDF 24 · Arena-Bench: A Benchmarking Suite for Obstacle Avoidance Approaches in Highly Dynamic Environments

- 分类：核心：实验平台。
- 内容：ROS集成的3D动态障碍训练、测试、评估套件，可比较传统规划与学习规划、多机器人平台并进行真实部署。
- 来源：PDF 24：2.pdf（原归档：`05 Papers/D0-References/参考论文/2.pdf`）；8页。

### PDF 25 · DynaBARN: Benchmarking Metric Ground Navigation in Dynamic Environments

- 分类：核心：评测基准。
- 内容：系统改变动态障碍运动速度、平滑度和加减速等motion profiles；难度指标生成场景，提供人类示范采集与传统/学习基线。
- 来源：PDF 25：3.pdf（原归档：`05 Papers/D0-References/参考论文/3.pdf`）；6页。

### PDF 26 · Benchmarking Reinforcement Learning Techniques for Autonomous Navigation

- 分类：核心：实验设计。
- 内容：导航RL四目标：不确定性推理、安全、有限试错数据、未知环境泛化；比较记忆网络、安全RL、模型RL、域随机化，包含大规模基准与真实环境研究。
- 来源：PDF 26：4.pdf（原归档：`05 Papers/D0-References/参考论文/4.pdf`）；7页。

### PDF 27 · Arena-Rosnav 2.0: A Development and Benchmarking Platform for Robot Navigation in Highly Dynamic Environments

- 分类：核心：实验平台。
- 内容：模块化重构Arena，统一规划算法/仿真器/评价API，更真实行人及多个仿真器，降低开发训练评测接入难度。
- 来源：PDF 27：5.pdf（原归档：`05 Papers/D0-References/参考论文/5.pdf`）；8页。

### PDF 28 · Autonomous Ground Navigation in Highly Constrained Spaces: Lessons learned from The 3rd BARN Challenge at ICRA 2024

- 分类：导航竞赛经验。
- 内容：ICRA2024 BARN狭窄密集静态障碍赛事报告；Jackal差速底盘与2D激光，仿真晋级与实物赛，适合实车鲁棒性/失败案例参考，不是单独RL新算法。
- 来源：PDF 28：6.pdf（原归档：`05 Papers/D0-References/参考论文/6.pdf`）；9页。

### PDF 29 · Arena 3.0: Advancing Social Navigation in Collaborative and Highly Dynamic Environments

- 分类：核心：社会导航平台。
- 内容：扩展Arena系列，增强人-人、人-机器人交互与社会力仿真、多任务/动态环境生成，跨三类仿真器支持开发和基准。
- 来源：PDF 29：7.pdf（原归档：`05 Papers/D0-References/参考论文/7.pdf`）；11页。

### PDF 30 · SICNav: Safe and Interactive Crowd Navigation using Model Predictive Control and Bilevel Optimization

- 分类：模型规划对照。
- 内容：双层非线性MPC联合求机器人运动和人群预测；ORCA人类模型嵌入约束并用KKT转化求解，显式考虑交互与安全。非RL。
- 来源：PDF 30：8.pdf（原归档：`05 Papers/D0-References/参考论文/8.pdf`）；18页。

### PDF 31 · SICNav-Diffusion: Safe and Interactive Crowd Navigation with Diffusion Trajectory Predictions

- DOI：[10.48550/arXiv.2503.08858](https://doi.org/10.48550/arXiv.2503.08858)。

- 分类：学习预测与优化对照。
- 内容：扩散模型产生多人联合多模态轨迹预测，双层MPC同时安全过滤预测与优化机器人轨迹；ETH/UCY开放环及真实机器人闭环测试，非RL控制策略。
- 来源：PDF 31：9.pdf（原归档：`05 Papers/D0-References/参考论文/9.pdf`）；8页。

### PDF 32 · DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles

- 分类：重复副本。
- 内容：与ID14 SHA256一致。
- 来源：PDF 32：IEEE T-RO DRL-VO(1).pdf（原归档：`05 Papers/D0-References/参考论文/IEEE T-RO DRL-VO(1).pdf`）；20页。

### PDF 33 · DRL-VO：学习使用速度障碍物在拥挤的动态场景中导航

- 分类：中文版本。
- 内容：ID14的中文/压缩版本，同一论文；技术结论优先核对英文原文。
- 来源：PDF 33：IEEE T-RO DRL-VO_compressed-mono.pdf（原归档：`05 Papers/D0-References/参考论文/IEEE T-RO DRL-VO_compressed-mono.pdf`）；20页。

### PDF 34 · iTD3-CLN: Learn to navigate in dynamic scene through Deep Reinforcement Learning

- 分类：重复副本。
- 内容：与ID15 SHA256一致。
- 来源：PDF 34：NeurocomputingiTD3 CLN Learn to navigate in dynamic scene through Deep(1).pdf（原归档：`05 Papers/D0-References/参考论文/NeurocomputingiTD3 CLN Learn to navigate in dynamic scene through Deep(1).pdf`）；11页。

### PDF 35 · Holistic Fusion: Task- and Setup-Agnostic Robot Localization and State Estimation with Factor Graphs

- 分类：定位状态估计支撑。
- 内容：多模态因子图联合估计局部/全局状态与参考系对齐，低时延且平滑的在线融合；非RL导航，但与里程计、定位与坐标系稳定性相关。
- 来源：PDF 35：Task-andSetup-Agnostic Robot.pdf（原归档：`05 Papers/D0-References/参考论文/Task-andSetup-Agnostic Robot.pdf`）；21页。

### PDF 36 · 用于工业自动移动机器人的基于学习的无地图导航的启发式密集奖励塑造

- DOI：[10.1016/j.isatra.2024.10.026](https://doi.org/10.1016/j.isatra.2024.10.026)。

- 分类：中文版本。
- 内容：ID7的中文版本，技术公式/符号以英文原文为准。
- 来源：PDF 36：启发式密集奖励塑造（中南大学）（中文版）.pdf（原归档：`05 Papers/D0-References/参考论文/启发式密集奖励塑造（中南大学）（中文版）.pdf`）；18页。

### PDF 37 · Heuristic dense reward shaping for learning-based map-free navigation of industrial automatic mobile robots

- DOI：[10.1016/j.isatra.2024.10.026](https://doi.org/10.1016/j.isatra.2024.10.026)。

- 分类：重复副本。
- 内容：与ID7 SHA256一致。
- 来源：PDF 37：启发式密集奖励塑造（中南大学）（英文版）.pdf（原归档：`05 Papers/D0-References/参考论文/启发式密集奖励塑造（中南大学）（英文版）.pdf`）；18页。

### PDF 38 · Virtual-to-real Deep Reinforcement Learning: Continuous Control of Mobile Robots for Mapless Navigation

- 分类：重复副本。
- 内容：与ID10 SHA256一致。
- 来源：PDF 38：2017IROSVirtual-to-real Deep Reinforcement Learning.pdf（原归档：`05 Papers/RL论文/2017IROSVirtual-to-real Deep Reinforcement Learning.pdf`）；6页。

### PDF 39 · CrowdMove: Autonomous Mapless Navigation in Crowded Scenarios

- 分类：重复副本。
- 内容：与ID11 SHA256一致。
- 来源：PDF 39：CrowdMove Autonomous Mapless Navigation in Crowded Scenarioshttpsar5iv.labs.arxiv.orghtml1807.07870.pdf（原归档：`05 Papers/RL论文/CrowdMove Autonomous Mapless Navigation in Crowded Scenarioshttpsar5iv.labs.arxiv.orghtml1807.07870.pdf`）；6页。

### PDF 40 · DWA-RL: Dynamically Feasible Deep Reinforcement Learning Policy for Robot Navigation among Mobile Obstacles

- 分类：核心：动力学可行RL。
- 内容：融合DWA动力学约束和DRL动态避障；低维观测编码障碍运动，奖励鼓励避开障碍朝向，3D仿真及真实差速机器人测试。PDF内正式题名与文件名不同。
- 来源：PDF 40：Dynamically Feasible Deep Reinforcement Learning Policy for Robot Navigation in Dense Mobile Crowds(1).pdf（原归档：`05 Papers/RL论文/Dynamically Feasible Deep Reinforcement Learning Policy for Robot Navigation in Dense Mobile Crowds(1).pdf`）；9页。

### PDF 41 · Localisation-Safe Reinforcement Learning for Mapless Navigation

- 分类：核心：定位与历史信息。
- 内容：不再假设真实位姿，使用激光/视觉里程计；定位失败惩罚及当前观测+历史轨迹表示，避免使定位失效的行为和局部极小值。
- 来源：PDF 41：Localisation-Safe Reinforcement Learning for Mapless Navigation.pdf（原归档：`05 Papers/RL论文/Localisation-Safe Reinforcement Learning for Mapless Navigation.pdf`）；8页。

### PDF 42 · Robot Navigation with Map-Based Deep Reinforcement Learning

- 分类：核心：地图观测表示。
- 内容：自车中心局部占据地图输入CNN预测转向动作；容纳多传感器及融合，训练于仿真、部署真实机器人，关注传感器噪声鲁棒性。
- 来源：PDF 42：Robot Navigation with Map-Based Deep Reinforcement Learning.pdf（原归档：`05 Papers/RL论文/Robot Navigation with Map-Based Deep Reinforcement Learning.pdf`）；6页。

### PDF 43 · A comprehensive review of path planning algorithms for autonomous navigation

- DOI：[10.1016/j.rineng.2025.107750](https://doi.org/10.1016/j.rineng.2025.107750)。

- 分类：导航综述。
- 内容：覆盖经典、元启发式、AI/RL规划以及动态环境、非完整约束和未知环境；适合相关工作结构与方法分类。
- 来源：PDF 43：A comprehensive review of path planning algorithms for autonomous navigation.pdf（原归档：`工作总结-2026.1.18/7-文献整理/A comprehensive review of path planning algorithms for autonomous navigation.pdf`）；22页。

### PDF 44 · A hybrid RRT-DWA path planning framework for UAVs in dynamic environments

- DOI：[10.1038/s41598-025-32993-w](https://doi.org/10.1038/s41598-025-32993-w)。

- 分类：跨载体传统规划参考。
- 内容：无人机RRT全局+DWA局部，以关键子目标引导避障/逃离局部最优；非RL，可参考全局路径到局部目标的接口。
- 来源：PDF 44：A hybrid RRT-DWA path planning framework for UAVs in dynamic environments.pdf（原归档：`工作总结-2026.1.18/7-文献整理/A hybrid RRT-DWA path planning framework for UAVs in dynamic environments.pdf`）；17页。

### PDF 45 · Efficient TD3 based path planning of mobile robot in dynamic environments using prioritized experience replay and LSTM

- DOI：[10.1038/s41598-025-02244-z](https://doi.org/10.1038/s41598-025-02244-z)。

- 分类：核心：TD3与记忆。
- 内容：PL-TD3：PER提高样本使用效率，LSTM处理时间依赖；静态/动态障碍、泛化与实物实验。与三帧激光堆叠对比学习时序编码直接相关。
- 来源：PDF 45：Efficient TD3 based path planning of mobile robot in dynamic environments using prioritized experience replay and LSTM.pdf（原归档：`工作总结-2026.1.18/7-文献整理/Efficient TD3 based path planning of mobile robot in dynamic environments using prioritized experience replay and LSTM.pdf`）；16页。

### PDF 46 · Enhancing autonomous exploration for robotics via real time map optimization and improved frontier costs

- DOI：[10.1038/s41598-025-97231-9](https://doi.org/10.1038/s41598-025-97231-9)。

- 分类：建图探索支撑。
- 内容：双边滤波/膨胀优化栅格、前沿成本融合路径长度/传感器量程/信息增益，结合MRTSP提高探索覆盖效率；非RL。
- 来源：PDF 46：Enhancing autonomous exploration for robotics via real time map optimization and improved frontier costs.pdf（原归档：`工作总结-2026.1.18/7-文献整理/Enhancing autonomous exploration for robotics via real time map optimization and improved frontier costs.pdf`）；12页。

### PDF 47 · Global and local path planning of robots combining ACO and dynamic window algorithm

- DOI：[10.1038/s41598-025-93571-8](https://doi.org/10.1038/s41598-025-93571-8)。

- 分类：传统全局局部对照。
- 内容：改进蚁群全局搜索+DWA局部避障；锥形信息素初始化、自适应启发因子、分工，路径方向角评价与动态速度采样。非RL。
- 来源：PDF 47：Global and local path planning of robots combining improved ant colony algorithm and dynamic window algorithm.pdf（原归档：`工作总结-2026.1.18/7-文献整理/Global and local path planning of robots combining improved ant colony algorithm and dynamic window algorithm.pdf`）；19页。

### PDF 48 · Robot movement planning for obstacle avoidance using reinforcement learning

- DOI：[10.1038/s41598-025-17740-5](https://doi.org/10.1038/s41598-025-17740-5)。

- 分类：机械臂方法参考。
- 内容：多自由度机械臂三维避障，改进APF+DDPG、方向补偿奖励、末端轨迹和全身碰撞；题名虽泛称robot，不能当作智能小车论文。
- 来源：PDF 48：Robot movement planning for obstacle avoidance using reinforcement learning.pdf（原归档：`工作总结-2026.1.18/7-文献整理/Robot movement planning for obstacle avoidance using reinforcement learning.pdf`）；11页。

### PDF 49 · An energy aware Q-learning framework for comprehensive coverage path planning in unknown complex environments

- DOI：[10.1038/s41598-025-28477-6](https://doi.org/10.1038/s41598-025-28477-6)。

- 分类：RL覆盖任务拓展。
- 内容：PERM-QN：动态权重奖励、优先经验回放、记忆网络，兼顾全覆盖、路长、能耗；仿真灾后环境探索，任务区别于点到点局部导航。
- 来源：PDF 49：一种面向未知复杂环境的全面覆盖路径规划的节能型Q学习框架.pdf（原归档：`工作总结-2026.1.18/7-文献整理/一种面向未知复杂环境的全面覆盖路径规划的节能型Q学习框架.pdf`）；18页。

### PDF 50 · A Comprehensive Study of Recent Path-Planning Techniques in Dynamic Environments for Autonomous Robots

- 分类：动态导航综述。
- 内容：动态环境路径规划综述，涉及障碍检测、路径规划、多机器人编队和通信/决策机制；适合动态环境相关工作。Sensors 2024, 24, 8089。
- 来源：PDF 50：动态环境下自主机器人路径规划技术的最新研究进展.pdf（原归档：`工作总结-2026.1.18/7-文献整理/动态环境下自主机器人路径规划技术的最新研究进展.pdf`）；31页。

### PDF 51 · Self-adaptive search algorithm for path planning based on the A* algorithm

- DOI：[10.1038/s41598-025-28847-0](https://doi.org/10.1038/s41598-025-28847-0)。

- 分类：传统全局规划参考。
- 内容：ODDEA*依障碍密度自适应调启发函数权重并加局部动态惩罚，减少扩展节点和搜索耗时；适合全局规划比较，非RL。
- 来源：PDF 51：基于A算法的路径规划自适应搜索算法.pdf（原归档：`工作总结-2026.1.18/7-文献整理/基于A算法的路径规划自适应搜索算法.pdf`）；11页。

## 5. 副本、版本和容易误读的文件

| 类型 | 文件关联 | 处理方式 |
|---|---|---|
| SHA256完全重复 | PDF14=PDF32；PDF15=PDF34；PDF11=PDF39；PDF10=PDF38；PDF07=PDF37 | 保留源文件路径，阅读时合并，不重复算独立研究。 |
| 中文版本 | PDF33对应DRL-VO；PDF36对应HDRS | 可辅助理解；公式、术语和量化结果优先核对英文原文。 |
| 同题不同版本 | PDF05 arXiv:2410.03066v1 与 PDF16 RLJ_RLC_2025_214 | 同一Hybrid Classical/RL研究，版本内容可能有扩充；不按两个独立算法计数。 |
| 文件名误导 | PDF17 ROS_nav2.pdf | 正文为2016年的ROS Navigation Tuning Guide，并非ROS2 Nav2论文。 |
| 载体不符 | PDF01/02水面艇、PDF44无人机、PDF19四足机器人、PDF48机械臂 | 归为方法/邻域参考，不用于证明小车RL导航效果。 |
| 任务不同 | PDF49全覆盖、PDF46探索建图、PDF35定位融合 | 可构成系统支撑或拓展；不等同点到点局部导航。 |

## 6. 未完成的验证边界

本次已完成论文目录的文件级全文提取、题名核对和主题摘要；没有执行论文代码、模型训练、实车复现、外链检查或数学证明审查。本地提取页码可能与印刷页码不同。任何用于正式论文的量化表格、方程或参考文献信息，仍须按原PDF对应页面核对。
