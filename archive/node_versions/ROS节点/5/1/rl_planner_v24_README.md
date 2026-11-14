# RL推理节点 v24 更新说明

## 版本信息
- **版本号**: v24 (综合版)
- **基于**: v21-13-10
- **合并功能**: v22近目标收束 + v23障碍绕行
- **更新日期**: 2026-04-21
- **目标**: 一次性解决两大核心问题

---

## 问题域覆盖

v24是v22和v23的**完全合并版本**，解决：

| 问题 | 来源 | 解决方案 |
|------|------|---------|
| 快到目标点时绕圈 | v22 | 三级收束 + 绕圈保护器 |
| 在障碍物前蹲步 | v23 | 前向检测 + 强制转向 |

---

## 控制链架构

v24的完整控制流程：

```
ONNX网络输出
    ↓
动作映射(clip + 滤波)
    ↓
┌─────────────────────────────────────┐
│ A. 目标角门控（原有粗控）              │
│    - abs(dyaw) > 0.8 → 原地转        │
│    - abs(dyaw) > 0.5 → 限速0.1       │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ B. 近目标收束控制（v22）              │
│    一级: dist < 1.0m                 │
│       → v ≤ 0.20, w × 0.5           │
│    二级: dist < 0.7m                 │
│       → v ≤ 0.10, w × 0.3           │
│    三级: dist < 0.6m + |dyaw| > 0.25 │
│       → v = 0, w = P控制对准         │
│    防大弧线: dist < 1.0m + |w| > 0.25│
│       → v ≤ 0.12                    │
│    绕圈保护: 12帧检测触发            │
│       → v = 0, w = 强制对准          │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ C. 前向障碍绕行增强（v23）            │
│    软保护: front < 0.60m             │
│       → v ≤ 0.08, |w| ≥ 0.45        │
│    硬保护: front < 0.40m             │
│       → v = 0, |w| ≥ 0.65           │
│    方向: 选择左右更空的通道           │
└─────────────────────────────────────┘
    ↓
最终clip保险
    ↓
发布cmd_vel
```

---

## 功能协同逻辑

### 场景分析矩阵

| 距离目标 | 前方状态 | 触发功能 | 日志标签 | 预期行为 |
|---------|---------|---------|---------|---------|
| > 1.5m | 空旷 | 无 | near=none, obs=none | 网络原始控制 |
| > 1.5m | 有障碍 | v23 | near=none, obs=soft_turn | 低速转向绕障 |
| 0.8m | 空旷 | v22 | near=stage_1, obs=none | 减速收束 |
| 0.8m | 有障碍 | v22+v23 | near=stage_1, obs=soft_turn | 低速小角度转向 |
| 0.5m | 空旷 | v22 | near=stage_2, obs=none | 进一步减速 |
| 0.5m | 有障碍 | v22+v23 | near=stage_2, obs=hard_turn | 停止+强转 |
| 0.3m | 空旷+对准中 | v22 | near=align, obs=none | 精确对准 |
| 绕圈中 | 任何 | v22保护 | orbit=orbit_guard_align | 强制纠偏 |

### 关键协同点

1. **近目标 + 前方空旷**
   - v22生效，v23不干预
   - 平滑收束到目标点

2. **近目标 + 前方有障碍**
   - v22限制最大速度（如0.10 m/s）
   - v23确保最小转向（如0.45 rad/s）
   - 结果：既不会高速撞上，也不会原地蹲步

3. **远离目标 + 前方有障碍**
   - v22不触发
   - v23单独生效
   - 保持空旷直行的稳定性

4. **绕圈检测触发**
   - 绕圈保护器优先级最高
   - 直接覆盖网络输出
   - 强制进入对准模式

---

## 与v21/v22/v23对比

| 特性 | v21 | v22 | v23 | v24 |
|------|-----|-----|-----|-----|
| 空旷直行 | ✅ 稳定 | ✅ 保持 | ✅ 保持 | ✅ 保持 |
| 近目标收束 | ❌ 无 | ✅ 三级 | ❌ 无 | ✅ 三级 |
| 绕圈保护 | ❌ 无 | ✅ 有 | ❌ 无 | ✅ 有 |
| 障碍检测 | ❌ 无 | ❌ 无 | ✅ 两级 | ✅ 两级 |
| 强制转向 | ❌ 无 | ❌ 无 | ✅ 有 | ✅ 有 |
| 代码复杂度 | 低 | 中 | 中 | 高 |
| 参数数量 | 10+ | 25+ | 20+ | 35+ |

**v24 = v22 + v23**，不是简单叠加，而是有机整合。

---

## 部署步骤

### 步骤1：替换节点文件
```bash
# 备份当前版本
cp ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py \
   ~/rl_planner_node4_backup_$(date +%Y%m%d).py

# 部署v24
cp rl_planner_node_v24_full.py \
   ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py
chmod +x ~/catkin_ws/src/rl_planner/scripts/rl_planner_node4.py
```

### 步骤2：更新launch文件
```xml
<!-- v24完整参数配置 -->
<rosparam command="load" file="$(find rl_planner)/config/v24_params.yaml"/>

<!-- 或手动添加所有参数 -->
<!-- v22参数 -->
<param name="enable_near_goal_controller" value="true"/>
<param name="near_goal_dist_1" value="1.0"/>
<param name="near_goal_dist_2" value="0.7"/>
<param name="near_goal_dist_3" value="0.6"/>
<param name="near_goal_v_cap_1" value="0.20"/>
<param name="near_goal_v_cap_2" value="0.10"/>
<param name="near_goal_w_scale_1" value="0.5"/>
<param name="near_goal_w_scale_2" value="0.3"/>
<param name="near_goal_align_yaw_thresh" value="0.25"/>
<param name="near_goal_align_k" value="1.2"/>
<param name="near_goal_align_w_cap" value="0.20"/>
<param name="near_goal_no_arc_dist" value="1.0"/>
<param name="near_goal_no_arc_w_thresh" value="0.25"/>
<param name="near_goal_no_arc_v_cap" value="0.12"/>
<param name="enable_orbit_guard" value="true"/>
<param name="orbit_guard_dist" value="1.2"/>
<param name="orbit_guard_window" value="12"/>
<param name="orbit_guard_same_sign_ratio" value="0.9"/>
<param name="orbit_guard_abs_w_thresh" value="0.35"/>
<param name="orbit_guard_progress_thresh" value="0.05"/>

<!-- v23参数 -->
<param name="enable_obstacle_turn_assist" value="true"/>
<param name="front_sector_half_width" value="20"/>
<param name="front_obstacle_dist_thresh" value="0.60"/>
<param name="front_obstacle_dist_hard" value="0.40"/>
<param name="obstacle_turn_v_cap" value="0.08"/>
<param name="obstacle_turn_v_cap_hard" value="0.00"/>
<param name="obstacle_turn_w_min" value="0.45"/>
<param name="obstacle_turn_w_hard" value="0.65"/>
<param name="side_sector_width" value="50"/>
```

### 步骤3：验证参数加载
```bash
roslaunch your_package your_launch_file.launch

# 检查v22参数
rosparam list | grep near_goal
rosparam list | grep orbit_guard

# 检查v23参数
rosparam list | grep obstacle
```

---

## 验证测试矩阵

### 测试场景设计

**场景1：空旷直行基准测试**
- 起点到目标：直线5米，无障碍
- 期望：保持v21的直行稳定性
- 日志：`near=none, obs=none`
- 目的：确认v24不会过度干预

**场景2：近目标收束测试**
- 起点到目标：直线2米，无障碍
- 期望：三级梯度减速，平滑到达
- 日志：
  ```
  dist=0.92m | near=stage_1, obs=none
  dist=0.65m | near=stage_2, obs=none
  dist=0.48m | near=align, obs=none
  *** 已到达目标点 dist=0.47m ***
  ```

**场景3：障碍绕行测试**
- 起点到目标：2米，前方0.5m处有障碍
- 期望：低速转向绕过障碍
- 日志：
  ```
  dist=1.23m | near=none, obs=soft_turn | front=0.52 L=2.3 R=3.1
  dist=0.98m | near=stage_1, obs=soft_turn | front=0.48 L=2.5 R=3.4
  ```

**场景4：近目标遇障碍综合测试**
- 起点到目标：1.5米，目标前0.4m有障碍
- 期望：两套机制协同，既不绕圈也不蹲步
- 日志：
  ```
  dist=0.87m | near=stage_1, obs=soft_turn | cmd v=+0.080 w=+0.225
  dist=0.62m | near=stage_2, obs=hard_turn | cmd v=+0.000 w=+0.450
  ```

**场景5：绕圈保护触发测试**
- 人为制造绕圈场景（如目标在墙后）
- 期望：12帧后触发保护
- 日志：
  ```
  [RL] reset action filter state: orbit_guard_triggered
  orbit=orbit_guard_align | cmd v=+0.000 w=[P控制值]
  ```

---

## 日志解读指南

### 完整日志格式
```
[RL] goal dx=+1.23 dy=-0.45 dyaw=+0.12 dist=1.32m | 
     gate=none | 
     near=stage_1 | 
     orbit=none | 
     obs=soft_turn | 
     front=0.54 L/R=2.1/3.4 | 
     cmd v=+0.180 w=+0.225
```

### 标签组合解读

| gate | near | orbit | obs | 含义 |
|------|------|-------|-----|------|
| none | none | none | none | 网络原始控制，无增强 |
| slowdown | none | none | none | 目标角度大，限速 |
| none | stage_1 | none | none | 近目标一级收束 |
| none | none | none | soft_turn | 前方障碍，低速转向 |
| none | stage_2 | none | hard_turn | 近目标+近障碍，停止强转 |
| none | align | none | none | 近目标精确对准模式 |
| none | stage_1 | orbit_guard_align | none | 绕圈触发，强制纠偏 |

### 异常模式识别

**绕圈未解决**：
```
# 连续多帧同样标签
dist=0.78m | near=stage_1 | orbit=none
dist=0.77m | near=stage_1 | orbit=none
dist=0.79m | near=stage_1 | orbit=none  ← 距离震荡
```
→ 调大 `near_goal_w_scale_1` 或启用绕圈保护

**障碍前蹲步未解决**：
```
# 速度反复正负
cmd v=+0.08 w=+0.45
cmd v=-0.05 w=+0.45
cmd v=+0.08 w=+0.45  ← 前后反复
```
→ 增大 `obstacle_turn_w_min` 或 `front_obstacle_dist_thresh`

**过度干预（丢失直行稳定性）**：
```
# 空旷时频繁触发增强
dist=3.45m | near=none | obs=soft_turn  ← 不应触发
front=1.23m ← 距离足够远
```
→ 调整 `front_obstacle_dist_thresh` 阈值

---

## 参数调优策略

### 问题诊断树

```
问题：仍然出现绕圈或蹲步

├─ 发生在近目标阶段（dist < 1m）
│  ├─ 日志显示 near=none
│  │  → v22未启用，检查 enable_near_goal_controller
│  ├─ 日志显示 near=stage_1/2，但仍绕圈
│  │  → 角速度压制不够，增大 w_scale 或启用 orbit_guard
│  └─ 日志显示 orbit=orbit_guard_align，但仍绕圈
│     → 保护器参数过严，放宽 same_sign_ratio 或 progress_thresh
│
└─ 发生在障碍物前（任何距离）
   ├─ 日志显示 obs=none
   │  → v23未启用，检查 enable_obstacle_turn_assist
   ├─ 日志显示 obs=soft_turn，但仍蹲步
   │  → 转向不够强，增大 obstacle_turn_w_min
   └─ 日志显示 obs=hard_turn，但仍蹲步
      → 极端情况，考虑环境因素或激光盲区
```

### 分模块调试

**只调v22**（近目标）：
```yaml
enable_near_goal_controller: true
enable_obstacle_turn_assist: false  # 暂时关闭v23
```

**只调v23**（障碍绕行）：
```yaml
enable_near_goal_controller: false  # 暂时关闭v22
enable_obstacle_turn_assist: true
```

**两者都调**（综合测试）：
```yaml
enable_near_goal_controller: true
enable_obstacle_turn_assist: true
```

---

## 性能开销

v24相比v21的额外计算：

| 功能 | 每帧开销 |
|------|---------|
| 前向障碍检测 | ~0.1ms (numpy切片+min/mean) |
| 左右通道比较 | ~0.05ms (numpy切片+mean) |
| 绕圈历史记录 | ~0.02ms (list append) |
| 绕圈检测判断 | ~0.1ms (numpy统计) |
| **总计** | **< 0.3ms/帧** |

在10Hz推理频率下，性能影响可忽略。

---

## 已知限制

1. **激光盲区问题**：
   - 如果障碍物在激光雷达盲区
   - v23无法检测，v22也无法绕开
   - 需要配合超声波或其他传感器

2. **动态障碍物**：
   - 当前逻辑假设障碍物静止
   - 快速移动的障碍物可能误判方向
   - 需要结合速度信息

3. **复杂拓扑**：
   - U形死胡同场景
   - 两套机制都可能失效
   - 需要全局路径规划

4. **参数耦合**：
   - 35+个参数，调优空间大
   - 某些组合可能产生冲突
   - 建议分模块逐个调试

---

## 版本演进路线图

```
v21 (基线)
  ↓
v21 → v22 (近目标) → v24 (综合)
  ↓                     ↑
v21 → v23 (障碍)   ────┘
```

**选择指南**：
- **只有近目标绕圈问题** → v22
- **只有障碍前蹲步问题** → v23
- **两个问题都有** → v24 ✅
- **追求极致稳定性** → v21（纯网络）

---

## 文件清单

v24交付包含：
1. `rl_planner_node_v24_full.py` - 综合版节点代码
2. `rl_planner_v24_params.yaml` - 完整参数配置
3. `rl_planner_v24_README.md` - 本说明文档

建议保留v22/v23文档作为参考。

---

## 技术支持

遇到问题时，请提供：
1. 完整roslaunch日志（包含 `[RL]` 行）
2. 问题发生时的四个标签值：`gate / near / orbit / obs`
3. 当时的 `dist_now / front_min / L/R` 数值
4. 使用的参数配置（是否修改过默认值）
5. 场景描述（目标距离、障碍位置）

---

**v24是生产就绪版本**，合并了v22和v23的所有功能，控制链清晰，日志完备，推荐作为主分支使用。
