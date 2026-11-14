# RL推理节点 v22 更新说明

## 版本信息
- **版本号**: v22
- **基于**: v21-13-10
- **更新日期**: 2026-04-21
- **主要目标**: 解决"快到目标点时绕圈"问题

---

## 核心问题诊断

从实验0001的数据分析发现：
1. **长期单侧转向**：avg_angular_z = -0.444, max = 0.0, min = -0.5
   - 整场实验角速度几乎没有正向输出
   - 中后段持续 `(0.5, -0.5)` 组合（最大线速度 + 饱和角速度）
   - 形成大弧线绕行而非直接收束到目标点

2. **缺乏近目标控制逻辑**：
   - 只有"进入tolerance就停"的终止条件
   - 没有"接近目标前如何稳定收进去"的过渡控制
   - 现有角度门控只减速不纠偏

3. **实验结果**：
   - 成功标志：0（失败）
   - 结束原因：mode_switched_to_teleop（人工接管）
   - 最终距离目标：0.849m

---

## v22三大核心改进

### 改进1：近目标收束控制器
分三级梯度减速和角速度压制：

```
距离 > 1.0m:  正常网络控制
距离 < 1.0m:  v_cap = 0.20, w_scale = 0.5  (一级收束)
距离 < 0.7m:  v_cap = 0.10, w_scale = 0.3  (二级收束)
距离 < 0.6m:  进入精确对准模式
  且 |dyaw| > 0.25rad:  v = 0, w = P控制直接对准
```

**关键特性**：
- 防止"高速前进 + 大角速度"组合
- 距离越近，控制越保守
- 三级模式之间平滑过渡

### 改进2：绕圈保护器
检测并终止绕圈行为：

**触发条件**（同时满足）：
1. 距离 < 1.2m（近目标区域）
2. 最近12帧中，单侧转向比例 > 0.9
3. 角速度均值 > 0.35 rad/s
4. 距离下降 < 0.05m

**响应动作**：
- 立即停止前进（v = 0）
- 切换到P控制对准模式
- 复位滤波器历史状态

### 改进3：防大弧线组合
禁止近目标区的"快速绕圈"组合：

```
IF dist < 1.0m AND |w| > 0.25 rad/s:
    v_cap = 0.12 m/s
```

即使网络输出高速+大角速度，也会被强制压制。

---

## 新增参数对照表

| 参数名 | 默认值 | 作用 |
|--------|--------|------|
| `enable_near_goal_controller` | true | 总开关 |
| `near_goal_dist_1` | 1.0 m | 一级收束区距离 |
| `near_goal_dist_2` | 0.7 m | 二级收束区距离 |
| `near_goal_dist_3` | 0.6 m | 对准模式距离 |
| `near_goal_v_cap_1` | 0.20 m/s | 一级速度上限 |
| `near_goal_v_cap_2` | 0.10 m/s | 二级速度上限 |
| `near_goal_w_scale_1` | 0.5 | 一级角速度缩放 |
| `near_goal_w_scale_2` | 0.3 | 二级角速度缩放 |
| `near_goal_align_yaw_thresh` | 0.25 rad | 对准角度阈值 |
| `near_goal_align_k` | 1.2 | P控制增益 |
| `near_goal_align_w_cap` | 0.20 rad/s | 对准角速度上限 |
| `enable_orbit_guard` | true | 绕圈保护开关 |
| `orbit_guard_dist` | 1.2 m | 保护启用距离 |
| `orbit_guard_window` | 12 帧 | 检测窗口 |
| `orbit_guard_same_sign_ratio` | 0.9 | 单侧比例阈值 |
| `orbit_guard_abs_w_thresh` | 0.35 rad/s | 角速度阈值 |
| `orbit_guard_progress_thresh` | 0.05 m | 距离进展阈值 |

---

## 代码修改对比

### v21 → v22 修改点：

#### 1. `__init__` 新增参数（第148-179行）
```python
# ===== 版本22新增：近目标收束控制参数 =====
self.enable_near_goal_controller = bool(...)
self.near_goal_dist_1 = float(...)
# ... 共17个新参数
self._recent_cmd_w = []
self._recent_dist = []
```

#### 2. 新增两个辅助函数（第424-475行）
```python
def _push_control_history(self, dist_now, cmd_w):
    """记录控制历史用于绕圈检测"""
    
def _orbit_guard_triggered(self):
    """检测是否触发绕圈保护"""
```

#### 3. 推理循环核心逻辑替换（第937-1017行）
替换原有简单角度门控为：
- A. 原有粗门控（兼容保留）
- B. 近目标三级收束控制
- C. 绕圈保护器
- D. 最终保险clip

#### 4. 日志增强
新增三个状态标签：
- `angle_gate`: "turn_in_place" / "slowdown" / "none"
- `near_goal_mode`: "stage_1" / "stage_2" / "align" / "no_arc" / "none"
- `orbit_guard`: "orbit_guard_align" / "none"

---

## 部署步骤

### 步骤1：替换节点文件
```bash
# 备份旧版本
cp ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py \
   ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4_v21_backup.py

# 部署v22
cp rl_planner_node_v22.py \
   ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py

chmod +x ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py
```

### 步骤2：更新launch文件参数
在你的launch文件中添加：
```xml
<!-- v22新增参数 -->
<param name="enable_near_goal_controller" value="true"/>
<param name="near_goal_dist_1" value="1.0"/>
<param name="near_goal_dist_2" value="0.7"/>
<param name="near_goal_dist_3" value="0.6"/>
<!-- ... 或直接加载yaml文件 -->
<rosparam command="load" file="$(find rl_planner)/config/v22_params.yaml"/>
```

### 步骤3：验证参数加载
```bash
roslaunch your_package your_launch_file.launch

# 在另一个终端检查参数
rosparam list | grep near_goal
rosparam list | grep orbit_guard
```

---

## 预期效果验证

### 实验对照清单

**v21问题复现**（作为对照组）：
- [ ] 中后段出现持续 `(0.5, -0.5)` 组合
- [ ] avg_angular_z 严重负偏（如 -0.44）
- [ ] max_angular_z = 0.0（无正向纠偏）
- [ ] 最终距离目标 > 0.5m
- [ ] finish_reason = mode_switched_to_teleop

**v22期望表现**：
- [ ] 距离 < 1.0m 时，日志显示 `near_goal_stage_1`
- [ ] 距离 < 0.7m 时，日志显示 `near_goal_stage_2`
- [ ] 距离 < 0.6m 且角度偏大时，显示 `near_goal_align`
- [ ] 如触发绕圈，日志显示 `orbit_guard_align` + "reset action filter"
- [ ] avg_angular_z 接近0（双向纠偏）
- [ ] 成功到达 goal_tolerance 范围内（0.5m）
- [ ] finish_reason = goal_reached 或本地日志"已到达目标点"

### 关键日志格式
```
[RL] goal dx=+0.123 dy=-0.456 dyaw=+0.123 dist=0.789m | 
     status=goal_tracking | 
     gate=slowdown | 
     near=near_goal_stage_2 | 
     orbit=none | 
     raw[+0.456,+0.321] clip[+0.456,+0.321] | 
     cmd v=+0.100 w=-0.123
```

---

## 调试建议

### 如果仍然绕圈：
1. 检查 `near_goal_dist_1` 是否过小（建议1.0m起步）
2. 调大 `near_goal_w_scale_1/2`（当前0.5/0.3）
3. 降低 `near_goal_align_k`（当前1.2，可降到0.8）

### 如果提前停止/过度保守：
1. 放宽 `near_goal_align_yaw_thresh`（从0.25增到0.35）
2. 调小 `orbit_guard_same_sign_ratio`（从0.9降到0.85）
3. 检查 `orbit_guard_abs_w_thresh` 是否过严

### 如果对准震荡：
1. 降低 `near_goal_align_k`（P增益过大）
2. 检查 `cmd_w_filter_alpha`（滤波是否合适）
3. 增加 `orbit_guard_window`（检测窗口过小）

---

## 下一步优化方向

如果v22解决了绕圈问题，后续可考虑：
1. **自适应距离阈值**：根据目标方向动态调整收束区大小
2. **速度-角度联合门控**：替代现有分段式控制
3. **轨迹平滑预测**：提前检测绕圈趋势
4. **终点朝向控制**：不仅到达位置，还要满足终点姿态

---

## 参数调优参考

### 保守配置（高成功率，速度慢）
```yaml
near_goal_dist_1: 1.5
near_goal_v_cap_1: 0.15
near_goal_w_scale_1: 0.3
orbit_guard_same_sign_ratio: 0.85
```

### 激进配置（速度快，需稳定环境）
```yaml
near_goal_dist_1: 0.8
near_goal_v_cap_1: 0.25
near_goal_w_scale_1: 0.7
orbit_guard_same_sign_ratio: 0.95
```

### 当前默认（平衡型）
适合Zeus-S2底盘，激光雷达543点，0.5m到达阈值场景。

---

## 版权与支持
- 基于实验0001数据诊断开发
- 符合现有obs_556架构和PPO训练范式
- 兼容ROS + Ubuntu 18.04 + ONNX运行时
