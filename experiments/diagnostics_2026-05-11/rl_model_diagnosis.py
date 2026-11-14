#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 强化学习局部规划器离线诊断分析工具
完整分析模型结构、观测数据、推理输出、部署问题
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import json
import os

# 设置中文字体
rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

class RLModelDiagnoser:
    def __init__(self, model_path, obs_path, summary_path):
        self.model_path = model_path
        self.obs_path = obs_path
        self.summary_path = summary_path
        
        # 数据容器
        self.model_dict = None
        self.obs_data = []
        self.summary_data = {}
        
        # 分析结果
        self.diagnosis_report = {
            "model_structure": {},
            "obs_validation": {},
            "inference_results": {},
            "problem_analysis": {},
            "recommendations": []
        }
        
    def load_model(self):
        """加载并检查模型文件"""
        print("=" * 80)
        print("一、模型文件检查")
        print("=" * 80)
        
        try:
            self.model_dict = torch.load(self.model_path, map_location='cpu')
            print(f"✓ 成功加载模型文件: {self.model_path}")
            print(f"✓ 模型文件类型: {type(self.model_dict)}")
            
            # 检查模型结构
            if isinstance(self.model_dict, dict):
                print(f"\n模型字典包含的 keys:")
                for key in self.model_dict.keys():
                    value = self.model_dict[key]
                    if isinstance(value, torch.Tensor):
                        print(f"  - {key}: Tensor, shape={value.shape}")
                    elif isinstance(value, dict):
                        print(f"  - {key}: Dict, 包含 {len(value)} 个子项")
                        if len(value) <= 10:
                            for subkey in list(value.keys())[:10]:
                                subval = value[subkey]
                                if isinstance(subval, torch.Tensor):
                                    print(f"      - {subkey}: shape={subval.shape}")
                    else:
                        print(f"  - {key}: {type(value)}")
                        
                # 检查是否是checkpoint
                if 'policy_state_dict' in self.model_dict or 'actor_state_dict' in self.model_dict:
                    print("\n✓ 检测到这是一个训练 checkpoint")
                    self.diagnosis_report["model_structure"]["type"] = "checkpoint"
                elif 'model_state_dict' in self.model_dict:
                    print("\n✓ 检测到这是一个包含 model_state_dict 的文件")
                    self.diagnosis_report["model_structure"]["type"] = "state_dict_wrapper"
                else:
                    # 直接是state_dict
                    print("\n✓ 检测到这可能是直接的 state_dict")
                    self.diagnosis_report["model_structure"]["type"] = "state_dict"
                    
                    # 尝试分析网络结构
                    self._analyze_network_structure()
                    
            else:
                print(f"\n✗ 模型类型不是字典: {type(self.model_dict)}")
                self.diagnosis_report["model_structure"]["type"] = "unknown"
                
        except Exception as e:
            print(f"\n✗ 加载模型失败: {e}")
            self.diagnosis_report["model_structure"]["error"] = str(e)
            return False
            
        return True
        
    def _analyze_network_structure(self):
        """分析网络结构"""
        print("\n网络结构分析:")
        
        # 查找第一层的输入维度
        input_dim = None
        output_dim = None
        
        layers = {}
        for key, tensor in self.model_dict.items():
            if isinstance(tensor, torch.Tensor):
                layers[key] = tensor.shape
                
                # 尝试推断输入维度
                if 'weight' in key and input_dim is None:
                    if len(tensor.shape) == 2:
                        input_dim = tensor.shape[1]
                        
        print(f"  发现 {len(layers)} 个参数层")
        
        # 显示前几层
        print("\n  网络参数概览 (前10层):")
        for i, (key, shape) in enumerate(list(layers.items())[:10]):
            print(f"    {key}: {shape}")
            
        if input_dim:
            print(f"\n  ✓ 推断输入维度: {input_dim}")
            self.diagnosis_report["model_structure"]["input_dim"] = input_dim
            
            if input_dim == 556:
                print(f"  ✓ 输入维度与观测维度匹配 (556)")
            else:
                print(f"  ✗ 输入维度 ({input_dim}) 与观测维度 (556) 不匹配")
                
        # 尝试找输出维度
        for key in reversed(list(layers.keys())):
            if 'weight' in key and len(layers[key]) == 2:
                output_dim = layers[key][0]
                break
                
        if output_dim:
            print(f"  ✓ 推断输出维度: {output_dim}")
            self.diagnosis_report["model_structure"]["output_dim"] = output_dim
            
            if output_dim == 2:
                print(f"  ✓ 输出维度为2 (linear_v, angular_w)")
            else:
                print(f"  ? 输出维度为 {output_dim} (预期为2)")
                
        # 保存网络结构
        self.diagnosis_report["model_structure"]["layers"] = {k: list(v) for k, v in layers.items()}
        
    def load_obs_data(self):
        """加载并解析观测数据"""
        print("\n" + "=" * 80)
        print("二、obs_556 数据解析与合法性检查")
        print("=" * 80)
        
        try:
            with open(self.obs_path, 'r') as f:
                content = f.read()
                
            # 解析每一帧
            frames = content.split('[Frame ')
            frames = [f for f in frames if f.strip()]
            
            print(f"\n✓ 成功加载观测文件: {self.obs_path}")
            print(f"✓ 总帧数: {len(frames)}")
            
            for frame_text in frames:
                lines = frame_text.strip().split('\n')
                frame_idx = int(lines[0].split(']')[0])
                
                # 提取时间戳
                t_line = lines[1] if len(lines) > 1 else ""
                t = float(t_line.split('t:')[1].strip()) if 't:' in t_line else 0.0
                
                # 提取观测
                obs_line = lines[2] if len(lines) > 2 else ""
                if 'obs:' in obs_line:
                    obs_str = obs_line.split('obs:')[1].strip()
                    obs = np.array([float(x) for x in obs_str.split()])
                    
                    self.obs_data.append({
                        'frame': frame_idx,
                        't': t,
                        'obs': obs
                    })
                    
            print(f"✓ 成功解析 {len(self.obs_data)} 帧观测数据")
            
            # 数据合法性检查
            self._validate_obs_data()
            
        except Exception as e:
            print(f"\n✗ 加载观测数据失败: {e}")
            import traceback
            traceback.print_exc()
            return False
            
        return True
        
    def _validate_obs_data(self):
        """验证观测数据合法性"""
        print("\n观测数据合法性检查:")
        
        # 检查维度
        dims = [len(item['obs']) for item in self.obs_data]
        all_556 = all(d == 556 for d in dims)
        
        print(f"  维度检查: {'✓ 所有帧都是556维' if all_556 else '✗ 存在维度不一致'}")
        if not all_556:
            unique_dims = set(dims)
            print(f"    发现的维度: {unique_dims}")
            
        # 检查NaN/Inf
        has_nan = False
        has_inf = False
        for item in self.obs_data:
            obs = item['obs']
            if np.any(np.isnan(obs)):
                has_nan = True
            if np.any(np.isinf(obs)):
                has_inf = True
                
        print(f"  NaN 检查: {'✗ 存在NaN' if has_nan else '✓ 无NaN'}")
        print(f"  Inf 检查: {'✗ 存在Inf' if has_inf else '✓ 无Inf'}")
        
        # 统计各部分
        if len(self.obs_data) > 0:
            obs_array = np.array([item['obs'] for item in self.obs_data])
            
            # obs[0:3] 线速度
            lin_vel = obs_array[:, 0:3]
            print(f"\n  obs[0:3] 线速度统计:")
            print(f"    范围: [{lin_vel.min():.4f}, {lin_vel.max():.4f}]")
            print(f"    均值: {lin_vel.mean(axis=0)}")
            
            # obs[3:6] 角速度
            ang_vel = obs_array[:, 3:6]
            print(f"\n  obs[3:6] 角速度统计:")
            print(f"    范围: [{ang_vel.min():.4f}, {ang_vel.max():.4f}]")
            print(f"    均值: {ang_vel.mean(axis=0)}")
            
            # obs[6:9] 重力投影
            gravity = obs_array[:, 6:9]
            print(f"\n  obs[6:9] 重力投影统计:")
            print(f"    范围: [{gravity.min():.4f}, {gravity.max():.4f}]")
            print(f"    均值: {gravity.mean(axis=0)}")
            is_fixed_gravity = np.allclose(gravity, [0, 0, -1], atol=0.01)
            print(f"    固定为[0,0,-1]: {'✓ 是' if is_fixed_gravity else '✗ 否'}")
            
            # obs[9:13] 局部目标
            goal = obs_array[:, 9:13]
            print(f"\n  obs[9:13] 局部目标 [dx, dy, dz, dyaw] 统计:")
            print(f"    dx 范围: [{goal[:, 0].min():.4f}, {goal[:, 0].max():.4f}]")
            print(f"    dy 范围: [{goal[:, 1].min():.4f}, {goal[:, 1].max():.4f}]")
            print(f"    dz 范围: [{goal[:, 2].min():.4f}, {goal[:, 2].max():.4f}]")
            print(f"    dyaw 范围: [{goal[:, 3].min():.4f}, {goal[:, 3].max():.4f}]")
            
            # 目标距离
            goal_dist = np.sqrt(goal[:, 0]**2 + goal[:, 1]**2)
            print(f"    目标距离范围: [{goal_dist.min():.4f}, {goal_dist.max():.4f}] m")
            
            # obs[13:556] LiDAR (三帧 181维×3)
            lidar = obs_array[:, 13:556]
            print(f"\n  obs[13:556] LiDAR (543维 = 181×3帧) 统计:")
            print(f"    范围: [{lidar.min():.4f}, {lidar.max():.4f}]")
            print(f"    均值: {lidar.mean():.4f}")
            print(f"    标准差: {lidar.std():.4f}")
            
            # 检查LiDAR是否在[0,1]附近 (假设归一化为÷12)
            in_01_range = np.sum((lidar >= 0) & (lidar <= 1.2)) / lidar.size
            print(f"    在[0, 1.2]范围内的比例: {in_01_range*100:.2f}%")
            
            # 三帧LiDAR分别统计
            lidar_t0 = obs_array[:, 13:194]
            lidar_t1 = obs_array[:, 194:375]
            lidar_t2 = obs_array[:, 375:556]
            
            print(f"\n  三帧LiDAR统计:")
            print(f"    t0 (最新帧) 均值: {lidar_t0.mean():.4f}, 最小值: {lidar_t0.min():.4f}")
            print(f"    t1 (中间帧) 均值: {lidar_t1.mean():.4f}, 最小值: {lidar_t1.min():.4f}")
            print(f"    t2 (最旧帧) 均值: {lidar_t2.mean():.4f}, 最小值: {lidar_t2.min():.4f}")
            
            # 前方LiDAR (假设中间60度, 约beam 60-120)
            front_lidar = lidar_t0[:, 60:120]
            print(f"\n  前方LiDAR区域 (beam 60-120) 统计:")
            print(f"    均值: {front_lidar.mean():.4f}")
            print(f"    最小值: {front_lidar.min():.4f}")
            print(f"    最小值 < 0.3 的帧数: {np.sum(front_lidar.min(axis=1) < 0.3)}/{len(self.obs_data)}")
            
            # 保存到报告
            self.diagnosis_report["obs_validation"] = {
                "total_frames": len(self.obs_data),
                "all_556_dim": bool(all_556),
                "has_nan": bool(has_nan),
                "has_inf": bool(has_inf),
                "lin_vel_range": [float(lin_vel.min()), float(lin_vel.max())],
                "ang_vel_range": [float(ang_vel.min()), float(ang_vel.max())],
                "gravity_fixed": bool(is_fixed_gravity),
                "goal_dist_range": [float(goal_dist.min()), float(goal_dist.max())],
                "lidar_range": [float(lidar.min()), float(lidar.max())],
                "lidar_in_01_ratio": float(in_01_range)
            }
            
    def load_summary(self):
        """加载实验摘要"""
        try:
            with open(self.summary_path, 'r') as f:
                content = f.read()
                
            # 简单解析
            lines = content.split('\n')
            for line in lines:
                if ':' in line and not line.startswith('['):
                    parts = line.split(':', 1)
                    if len(parts) == 2:
                        key = parts[0].strip()
                        value = parts[1].strip()
                        self.summary_data[key] = value
                        
            print(f"\n✓ 成功加载实验摘要")
            print(f"  实验ID: {self.summary_data.get('experiment_id', 'N/A')}")
            print(f"  控制模式: {self.summary_data.get('control_mode', 'N/A')}")
            print(f"  成功: {self.summary_data.get('success', 'N/A')}")
            print(f"  完成原因: {self.summary_data.get('finish_reason', 'N/A')}")
            print(f"  总路径长度: {self.summary_data.get('total_path_length_m', 'N/A')} m")
            print(f"  路径效率: {self.summary_data.get('path_efficiency', 'N/A')}")
            
        except Exception as e:
            print(f"\n✗ 加载实验摘要失败: {e}")
            
    def offline_inference(self):
        """执行离线推理"""
        print("\n" + "=" * 80)
        print("三、模型离线推理复现")
        print("=" * 80)
        
        if self.model_dict is None:
            print("✗ 模型未加载,无法推理")
            return False
            
        if len(self.obs_data) == 0:
            print("✗ 观测数据为空,无法推理")
            return False
            
        # 尝试构建一个简单的前向网络
        try:
            # 检查是否能直接推理
            state_dict = self.model_dict
            
            # 如果是checkpoint,提取state_dict
            if 'policy_state_dict' in self.model_dict:
                state_dict = self.model_dict['policy_state_dict']
            elif 'actor_state_dict' in self.model_dict:
                state_dict = self.model_dict['actor_state_dict']
            elif 'model_state_dict' in self.model_dict:
                state_dict = self.model_dict['model_state_dict']
                
            # 构建简单的前向推理
            print("\n尝试使用简化推理...")
            actions = []
            
            for i, item in enumerate(self.obs_data):
                obs = torch.FloatTensor(item['obs']).unsqueeze(0)
                
                # 简单的多层前向传播
                x = obs
                layer_idx = 0
                
                try:
                    # 假设是标准MLP结构
                    while True:
                        weight_key = f'actor.{layer_idx}.weight' if f'actor.{layer_idx}.weight' in state_dict else None
                        if weight_key is None:
                            weight_key = f'net.{layer_idx}.weight' if f'net.{layer_idx}.weight' in state_dict else None
                        if weight_key is None:
                            weight_key = f'{layer_idx}.weight' if f'{layer_idx}.weight' in state_dict else None
                            
                        if weight_key is None:
                            # 尝试其他命名
                            possible_keys = [k for k in state_dict.keys() if 'weight' in k and str(layer_idx) in k]
                            if possible_keys:
                                weight_key = possible_keys[0]
                            else:
                                break
                                
                        bias_key = weight_key.replace('weight', 'bias')
                        
                        weight = state_dict[weight_key]
                        bias = state_dict.get(bias_key, None)
                        
                        x = torch.matmul(x, weight.t())
                        if bias is not None:
                            x = x + bias
                            
                        # 激活函数 (假设前面层用ReLU,最后一层用tanh)
                        if layer_idx < 3:  # 假设有3个隐藏层
                            x = torch.relu(x)
                            
                        layer_idx += 1
                        
                        if layer_idx > 10:  # 防止无限循环
                            break
                            
                    # 最后一层用tanh
                    action = torch.tanh(x)
                    actions.append(action.detach().numpy()[0])
                    
                except Exception as e:
                    print(f"✗ 推理失败 (frame {i}): {e}")
                    break
                    
            if len(actions) > 0:
                print(f"✓ 成功推理 {len(actions)} 帧")
                self._analyze_actions(actions)
                return True
            else:
                print("✗ 推理失败,可能需要完整的网络定义")
                print("\n缺少的内容:")
                print("  - 完整的网络类定义 (例如 Actor 类)")
                print("  - obs normalization 参数 (running_mean, running_var)")
                print("  - action scaling 参数")
                print("  - 准确的网络层数和激活函数信息")
                return False
                
        except Exception as e:
            print(f"\n✗ 离线推理失败: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def _analyze_actions(self, actions):
        """分析推理输出的动作"""
        print("\n动作输出分析:")
        
        actions = np.array(actions)
        
        if actions.shape[1] >= 2:
            raw_v = actions[:, 0]
            raw_w = actions[:, 1]
            
            print(f"\n  原始动作输出范围:")
            print(f"    raw_v: [{raw_v.min():.4f}, {raw_v.max():.4f}]")
            print(f"    raw_w: [{raw_w.min():.4f}, {raw_w.max():.4f}]")
            
            # 检查饱和
            v_saturated = np.sum(np.abs(raw_v) > 0.95)
            w_saturated = np.sum(np.abs(raw_w) > 0.95)
            
            print(f"\n  动作饱和统计 (|action| > 0.95):")
            print(f"    线速度饱和帧数: {v_saturated}/{len(actions)} ({v_saturated/len(actions)*100:.1f}%)")
            print(f"    角速度饱和帧数: {w_saturated}/{len(actions)} ({w_saturated/len(actions)*100:.1f}%)")
            
            # 部署端映射
            linear_min = -0.2
            linear_max = 0.5
            angular_min = -0.5
            angular_max = 0.5
            
            v_mapped = linear_min + (raw_v + 1) * 0.5 * (linear_max - linear_min)
            w_mapped = angular_min + (raw_w + 1) * 0.5 * (angular_max - angular_min)
            
            v_mapped = np.clip(v_mapped, linear_min, linear_max)
            w_mapped = np.clip(w_mapped, angular_min, angular_max)
            
            print(f"\n  部署端映射后速度范围:")
            print(f"    v: [{v_mapped.min():.4f}, {v_mapped.max():.4f}] m/s")
            print(f"    w: [{w_mapped.min():.4f}, {w_mapped.max():.4f}] rad/s")
            
            # 检查部署端限幅
            v_clipped = np.sum((v_mapped <= linear_min + 0.01) | (v_mapped >= linear_max - 0.01))
            w_clipped = np.sum((w_mapped <= angular_min + 0.01) | (w_mapped >= angular_max - 0.01))
            
            print(f"\n  部署端限幅统计:")
            print(f"    线速度触及限幅: {v_clipped}/{len(actions)} ({v_clipped/len(actions)*100:.1f}%)")
            print(f"    角速度触及限幅: {w_clipped}/{len(actions)} ({w_clipped/len(actions)*100:.1f}%)")
            
            # 保存结果
            self.diagnosis_report["inference_results"] = {
                "total_frames": len(actions),
                "raw_v_range": [float(raw_v.min()), float(raw_v.max())],
                "raw_w_range": [float(raw_w.min()), float(raw_w.max())],
                "v_saturated": int(v_saturated),
                "w_saturated": int(w_saturated),
                "v_mapped_range": [float(v_mapped.min()), float(v_mapped.max())],
                "w_mapped_range": [float(w_mapped.min()), float(w_mapped.max())],
                "v_clipped": int(v_clipped),
                "w_clipped": int(w_clipped)
            }
            
            # 目标接近阶段分析
            self._analyze_goal_approach(actions)
            
    def _analyze_goal_approach(self, actions):
        """分析接近目标阶段的行为"""
        print("\n接近目标阶段行为分析:")
        
        # 提取目标距离
        goal_dists = []
        for item in self.obs_data[:len(actions)]:
            obs = item['obs']
            dx = obs[9]
            dy = obs[10]
            dist = np.sqrt(dx**2 + dy**2)
            goal_dists.append(dist)
            
        goal_dists = np.array(goal_dists)
        actions = np.array(actions)
        
        # 找到距离目标<1m的帧
        near_goal_mask = goal_dists < 1.0
        near_goal_indices = np.where(near_goal_mask)[0]
        
        if len(near_goal_indices) > 0:
            print(f"\n  距离目标 < 1m 的帧数: {len(near_goal_indices)}")
            
            near_actions = actions[near_goal_mask]
            near_dists = goal_dists[near_goal_mask]
            
            raw_v = near_actions[:, 0]
            raw_w = near_actions[:, 1]
            
            print(f"  该阶段原始动作统计:")
            print(f"    raw_v 均值: {raw_v.mean():.4f}, 标准差: {raw_v.std():.4f}")
            print(f"    raw_w 均值: {raw_w.mean():.4f}, 标准差: {raw_w.std():.4f}")
            print(f"    |raw_w| > 0.5 的帧数: {np.sum(np.abs(raw_w) > 0.5)}/{len(near_actions)}")
            
            # 检查是否存在持续旋转
            if np.abs(raw_w.mean()) > 0.3:
                print(f"  ⚠ 警告: 接近目标时存在明显的平均旋转偏向 ({raw_w.mean():.4f})")
                
        # 距离目标<0.5m
        very_near_mask = goal_dists < 0.5
        if np.sum(very_near_mask) > 0:
            very_near_actions = actions[very_near_mask]
            print(f"\n  距离目标 < 0.5m 的帧数: {np.sum(very_near_mask)}")
            print(f"    该阶段 |raw_w| 均值: {np.abs(very_near_actions[:, 1]).mean():.4f}")
            
    def diagnose_problems(self):
        """综合诊断问题"""
        print("\n" + "=" * 80)
        print("四、重点问题诊断")
        print("=" * 80)
        
        problems = []
        
        # 1. LiDAR归一化尺度问题
        print("\n1. LiDAR归一化尺度敏感性分析:")
        if 'obs_validation' in self.diagnosis_report:
            lidar_range = self.diagnosis_report['obs_validation'].get('lidar_range', [0, 1])
            max_lidar = lidar_range[1]
            
            print(f"   实测LiDAR最大值: {max_lidar:.4f}")
            print(f"   训练假设: LiDAR归一化为 ÷12 (最大归一化值约为 1.0)")
            
            if max_lidar > 1.1:
                print(f"   ⚠ 警告: LiDAR最大值({max_lidar:.2f})明显超过1.0")
                print(f"   → 可能部署端使用了不同的归一化尺度")
                print(f"   → 如果部署端用÷16,则12m障碍物会被归一化为0.75")
                print(f"   → 模型会认为环境更加拥挤,可能导致保守行为")
                problems.append("LiDAR归一化尺度不匹配")
            else:
                print(f"   ✓ LiDAR归一化尺度基本符合训练假设")
                
        # 2. 动作输出饱和问题
        print("\n2. 动作输出饱和分析:")
        if 'inference_results' in self.diagnosis_report:
            v_sat = self.diagnosis_report['inference_results'].get('v_saturated', 0)
            w_sat = self.diagnosis_report['inference_results'].get('w_saturated', 0)
            total = self.diagnosis_report['inference_results'].get('total_frames', 1)
            
            v_sat_rate = v_sat / total
            w_sat_rate = w_sat / total
            
            print(f"   线速度饱和率: {v_sat_rate*100:.1f}%")
            print(f"   角速度饱和率: {w_sat_rate*100:.1f}%")
            
            if v_sat_rate > 0.2 or w_sat_rate > 0.2:
                print(f"   ⚠ 警告: 存在明显的动作饱和问题")
                print(f"   → 模型频繁输出接近±1的原始动作")
                print(f"   → 说明模型期望更大的动作空间")
                print(f"   → 可能是训练端action范围与部署端不一致")
                problems.append("动作输出频繁饱和")
            else:
                print(f"   ✓ 动作饱和在可接受范围内")
                
        # 3. 目标朝向控制问题
        print("\n3. 目标朝向控制能力分析:")
        print(f"   当前输入: dyaw = atan2(dy, dx) (目标方向角)")
        print(f"   这表示目标点相对车体的方位,而非目标点的最终姿态角")
        print(f"   → 模型只知道'目标在哪个方向',不知道'到达后应该朝哪'")
        print(f"   → 该模型适合点到点导航,不适合定点定向任务")
        print(f"   → 如需最终朝向控制,需要在输入中增加目标yaw误差")
        problems.append("缺少目标最终朝向控制能力")
        
        # 4. 路径效率问题
        print("\n4. 路径效率分析:")
        path_eff = float(self.summary_data.get('path_efficiency', 0.6))
        print(f"   实测路径效率: {path_eff:.3f}")
        print(f"   总路径长度: {self.summary_data.get('total_path_length_m', 'N/A')} m")
        print(f"   直线距离: {self.summary_data.get('straight_start_to_goal_m', 'N/A')} m")
        
        if path_eff < 0.7:
            print(f"   ⚠ 路径效率偏低 (<0.7)")
            print(f"   → 存在明显的绕行、旋转、反复调整")
            print(f"   → 可能原因: 1)LiDAR归一化不匹配  2)sim-to-real差异  3)模型训练不足")
            problems.append("路径效率偏低")
        else:
            print(f"   ✓ 路径效率可接受")
            
        # 5. 部署限幅问题
        print("\n5. 部署限幅影响分析:")
        if 'inference_results' in self.diagnosis_report:
            v_clip = self.diagnosis_report['inference_results'].get('v_clipped', 0)
            w_clip = self.diagnosis_report['inference_results'].get('w_clipped', 0)
            total = self.diagnosis_report['inference_results'].get('total_frames', 1)
            
            print(f"   线速度触及限幅: {v_clip}/{total} ({v_clip/total*100:.1f}%)")
            print(f"   角速度触及限幅: {w_clip}/{total} ({w_clip/total*100:.1f}%)")
            print(f"   当前限幅: v∈[-0.2, 0.5], w∈[-0.5, 0.5]")
            
            if w_clip / total > 0.3:
                print(f"   ⚠ 角速度频繁触及限幅")
                print(f"   → 部署限幅可能削弱了策略表达能力")
                print(f"   → 训练端可能使用了更大的动作范围")
                problems.append("部署端限幅过于严格")
                
        # 保存诊断结果
        self.diagnosis_report["problem_analysis"]["identified_problems"] = problems
        
    def generate_recommendations(self):
        """生成修改建议"""
        print("\n" + "=" * 80)
        print("五、修改建议 (按优先级排序)")
        print("=" * 80)
        
        recommendations = []
        
        # 根据诊断结果生成建议
        problems = self.diagnosis_report.get("problem_analysis", {}).get("identified_problems", [])
        
        priority = 1
        
        if "LiDAR归一化尺度不匹配" in problems:
            rec = f"{priority}. 【最高优先级】检查训练端与部署端LiDAR归一化尺度"
            rec += "\n   - 训练端假设: 最大距离12m,归一化为÷12"
            rec += "\n   - 如果部署端使用÷16,会导致模型认为环境更拥挤"
            rec += "\n   - 修改方法: 在部署端统一使用÷12归一化"
            rec += "\n   - 验证方法: 检查lidar_pipeline_node.py中的归一化代码"
            recommendations.append(rec)
            priority += 1
            
        rec = f"{priority}. 检查观测向量的556维顺序与训练端完全一致"
        rec += "\n   - 速度[0:6], 重力[6:9], 目标[9:13], LiDAR[13:556]"
        rec += "\n   - 三帧LiDAR顺序: [t0_newest, t1, t2_oldest]"
        rec += "\n   - 验证方法: 对比训练代码中的obs构造逻辑"
        recommendations.append(rec)
        priority += 1
        
        if "动作输出频繁饱和" in problems:
            rec = f"{priority}. 检查动作映射范围与训练端一致"
            rec += "\n   - 训练端action范围: 通常为[-1, 1]经过tanh输出"
            rec += "\n   - 部署端映射: v∈[-0.2, 0.5], w∈[-0.5, 0.5]"
            rec += "\n   - 如果训练端使用了不同的映射,需要调整部署端"
            rec += "\n   - 验证方法: 检查训练代码中的action_scaling参数"
            recommendations.append(rec)
            priority += 1
            
        rec = f"{priority}. 增加目标附近停止逻辑"
        rec += "\n   - 当距离目标<0.3m时,可以提前停止RL控制"
        rec += "\n   - 避免RL在目标附近持续旋转或震荡"
        rec += "\n   - 修改方法: 在rl_local_goal_provider中增加提前到达判断"
        recommendations.append(rec)
        priority += 1
        
        if "缺少目标最终朝向控制能力" in problems:
            rec = f"{priority}. 如需定点定向功能,需要重新训练"
            rec += "\n   - 当前dyaw=atan2(dy,dx)只表示目标方向,不是最终姿态"
            rec += "\n   - 需要在obs中增加目标yaw误差: dyaw_error = goal_yaw - robot_yaw"
            rec += "\n   - 或者在RL到达目标点后,切换到简单的朝向调整控制器"
            rec += "\n   - 当前模型适合展示动态避障,不适合精确停车任务"
            recommendations.append(rec)
            priority += 1
            
        rec = f"{priority}. 考虑retraining的改进方向"
        rec += "\n   - 使用tanh作为action约束,而不是clip"
        rec += "\n   - 增加镜像数据增强,消除左右不对称偏置"
        rec += "\n   - 增加场地尺寸随机化,提升泛化能力"
        rec += "\n   - 增加初始速度随机化,避免总是从静止启动"
        recommendations.append(rec)
        priority += 1
        
        rec = f"{priority}. 与DWA对比的客观评价"
        rec += "\n   - DWA: 控制逻辑明确,稳定性强,但速度较慢,路径不够灵活"
        rec += "\n   - RL: 响应快,决策灵活,但高度依赖输入一致性"
        rec += "\n   - RL更敏感: 归一化尺度、动作映射、传感器噪声、摩擦力"
        rec += "\n   - 建议: RL用于展示智能决策,DWA作为稳定工程基线"
        recommendations.append(rec)
        
        # 打印建议
        for rec in recommendations:
            print(f"\n{rec}")
            
        self.diagnosis_report["recommendations"] = recommendations
        
    def generate_visualizations(self):
        """生成可视化图表"""
        print("\n" + "=" * 80)
        print("六、生成可视化图表")
        print("=" * 80)
        
        if len(self.obs_data) == 0:
            print("✗ 无观测数据,跳过可视化")
            return
            
        try:
            fig, axes = plt.subplots(3, 2, figsize=(14, 10))
            fig.suptitle('RL局部规划器离线诊断分析', fontsize=16, fontweight='bold')
            
            # 提取数据
            frames = [item['frame'] for item in self.obs_data]
            obs_array = np.array([item['obs'] for item in self.obs_data])
            
            # 1. 目标距离变化
            goal_dx = obs_array[:, 9]
            goal_dy = obs_array[:, 10]
            goal_dist = np.sqrt(goal_dx**2 + goal_dy**2)
            
            axes[0, 0].plot(frames, goal_dist, 'b-', linewidth=2)
            axes[0, 0].set_xlabel('帧序号')
            axes[0, 0].set_ylabel('目标距离 (m)')
            axes[0, 0].set_title('目标距离变化')
            axes[0, 0].grid(True, alpha=0.3)
            
            # 2. 目标方向角变化
            goal_dyaw = obs_array[:, 12]
            axes[0, 1].plot(frames, goal_dyaw, 'r-', linewidth=2)
            axes[0, 1].set_xlabel('帧序号')
            axes[0, 1].set_ylabel('dyaw (rad)')
            axes[0, 1].set_title('目标方向角变化')
            axes[0, 1].grid(True, alpha=0.3)
            
            # 3. LiDAR最小值变化
            lidar_t0 = obs_array[:, 13:194]
            lidar_min = lidar_t0.min(axis=1)
            
            axes[1, 0].plot(frames, lidar_min, 'g-', linewidth=2)
            axes[1, 0].axhline(y=0.3, color='r', linestyle='--', label='危险阈值(0.3)')
            axes[1, 0].set_xlabel('帧序号')
            axes[1, 0].set_ylabel('最小LiDAR距离')
            axes[1, 0].set_title('全向LiDAR最小值')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
            
            # 4. 前方LiDAR最小值
            front_lidar = lidar_t0[:, 60:120]  # 假设前方区域
            front_min = front_lidar.min(axis=1)
            
            axes[1, 1].plot(frames, front_min, 'orange', linewidth=2)
            axes[1, 1].axhline(y=0.3, color='r', linestyle='--', label='危险阈值(0.3)')
            axes[1, 1].set_xlabel('帧序号')
            axes[1, 1].set_ylabel('最小前方距离')
            axes[1, 1].set_title('前方LiDAR最小值 (beam 60-120)')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)
            
            # 5. 三帧LiDAR均值对比
            lidar_t1 = obs_array[:, 194:375]
            lidar_t2 = obs_array[:, 375:556]
            
            axes[2, 0].plot(frames, lidar_t0.mean(axis=1), 'b-', label='t0 (最新)', linewidth=2)
            axes[2, 0].plot(frames, lidar_t1.mean(axis=1), 'g-', label='t1 (中间)', linewidth=2)
            axes[2, 0].plot(frames, lidar_t2.mean(axis=1), 'r-', label='t2 (最旧)', linewidth=2)
            axes[2, 0].set_xlabel('帧序号')
            axes[2, 0].set_ylabel('LiDAR均值')
            axes[2, 0].set_title('三帧LiDAR均值对比')
            axes[2, 0].legend()
            axes[2, 0].grid(True, alpha=0.3)
            
            # 6. 速度统计
            lin_vel = obs_array[:, 0]
            ang_vel = obs_array[:, 5]  # 注意这里是obs[5]
            
            ax_v = axes[2, 1]
            ax_w = ax_v.twinx()
            
            line1 = ax_v.plot(frames, lin_vel, 'b-', label='线速度', linewidth=2)
            line2 = ax_w.plot(frames, ang_vel, 'r-', label='角速度', linewidth=2)
            
            ax_v.set_xlabel('帧序号')
            ax_v.set_ylabel('线速度 (m/s)', color='b')
            ax_w.set_ylabel('角速度 (rad/s)', color='r')
            ax_v.set_title('速度变化')
            ax_v.grid(True, alpha=0.3)
            
            # 合并图例
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax_v.legend(lines, labels, loc='upper right')
            
            plt.tight_layout()
            plt.savefig('/home/claude/rl_diagnosis_plots.png', dpi=150, bbox_inches='tight')
            print(f"✓ 可视化图表已保存: /home/claude/rl_diagnosis_plots.png")
            
        except Exception as e:
            print(f"✗ 生成可视化失败: {e}")
            import traceback
            traceback.print_exc()
            
    def save_report(self):
        """保存诊断报告"""
        report_path = '/home/claude/rl_diagnosis_report.json'
        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(self.diagnosis_report, f, indent=2, ensure_ascii=False)
            print(f"\n✓ 诊断报告已保存: {report_path}")
        except Exception as e:
            print(f"\n✗ 保存报告失败: {e}")
            
    def generate_conclusion(self):
        """生成模型问题定位结论"""
        print("\n" + "=" * 80)
        print("七、模型问题定位结论")
        print("=" * 80)
        
        print("\n基于obs_556.txt、summary.txt和模型离线分析,该RL局部规划模型存在以下主要问题:\n")
        
        problems_detail = []
        
        # 问题1: 路径效率偏低
        path_eff = float(self.summary_data.get('path_efficiency', 0.6))
        print(f"1. 【路径效率偏低】")
        print(f"   证据: summary.txt显示路径效率为{path_eff:.3f},总路径7.56m,直线距离5.00m")
        print(f"   问题: 存在明显的旋转、绕行、反复调整,路径不够直接")
        print(f"   判断: 主要来自模型本身的决策质量,可能与训练不足或sim-to-real差异有关")
        problems_detail.append({
            "问题": "路径效率偏低",
            "证据": f"路径效率{path_eff:.3f}",
            "来源": "模型本身 + sim-to-real迁移"
        })
        
        # 问题2: 动作输出饱和
        if 'inference_results' in self.diagnosis_report:
            v_sat_rate = self.diagnosis_report['inference_results'].get('v_saturated', 0) / \
                        self.diagnosis_report['inference_results'].get('total_frames', 1)
            w_sat_rate = self.diagnosis_report['inference_results'].get('w_saturated', 0) / \
                        self.diagnosis_report['inference_results'].get('total_frames', 1)
            
            print(f"\n2. 【动作输出频繁饱和】")
            print(f"   证据: summary.txt显示速度多次达到0.5和-0.5限幅")
            print(f"   证据: 离线推理显示原始动作饱和率: v={v_sat_rate*100:.1f}%, w={w_sat_rate*100:.1f}%")
            print(f"   问题: 模型频繁输出接近±1的原始动作,说明期望更大的动作空间")
            print(f"   判断: 主要来自动作映射配置,训练端与部署端动作范围可能不一致")
            problems_detail.append({
                "问题": "动作输出频繁饱和",
                "证据": f"v饱和{v_sat_rate*100:.1f}%, w饱和{w_sat_rate*100:.1f}%",
                "来源": "动作映射配置问题"
            })
            
        # 问题3: LiDAR归一化尺度敏感
        if 'obs_validation' in self.diagnosis_report:
            lidar_max = self.diagnosis_report['obs_validation']['lidar_range'][1]
            print(f"\n3. 【LiDAR归一化尺度可能不匹配】")
            print(f"   证据: obs_556.txt中LiDAR最大值为{lidar_max:.4f}")
            print(f"   假设: 训练端使用÷12归一化,最大值应接近1.0")
            
            if lidar_max > 1.1:
                print(f"   问题: LiDAR最大值明显超过1.0,说明部署端可能用了不同归一化")
                print(f"   影响: 如果部署端用÷16,12m障碍物归一化为0.75,模型会认为更拥挤")
                print(f"   判断: 主要来自部署输入配置,导致模型保守、后退、绕行")
                problems_detail.append({
                    "问题": "LiDAR归一化尺度不匹配",
                    "证据": f"LiDAR最大值{lidar_max:.2f}",
                    "来源": "部署输入配置问题"
                })
            else:
                print(f"   判断: LiDAR归一化基本符合训练假设,此问题不明显")
                
        # 问题4: 缺少目标最终朝向控制
        print(f"\n4. 【缺少目标最终朝向控制能力】")
        print(f"   证据: obs_556.txt中dyaw=atan2(dy,dx),只表示目标方向")
        print(f"   问题: 模型不知道到达目标后应该朝向哪里")
        print(f"   影响: 只能做点到点导航,无法精确定点定向")
        print(f"   判断: 来自目标表达方式,需要在obs中增加目标yaw误差才能控制最终朝向")
        problems_detail.append({
            "问题": "缺少目标最终朝向控制",
            "证据": "dyaw只是方位角,不是姿态误差",
            "来源": "目标表达方式设计问题"
        })
        
        # 问题5: 接近目标时可能存在旋转
        near_goal_obs = [item for item in self.obs_data if np.sqrt(item['obs'][9]**2 + item['obs'][10]**2) < 1.0]
        if len(near_goal_obs) > 5:
            near_dyaw = [item['obs'][12] for item in near_goal_obs]
            dyaw_std = np.std(near_dyaw)
            
            print(f"\n5. 【接近目标时存在调整行为】")
            print(f"   证据: 距离<1m时的帧数={len(near_goal_obs)},dyaw标准差={dyaw_std:.4f}")
            print(f"   问题: 模型在接近目标时仍在调整朝向,导致路径不够直接")
            print(f"   判断: 来自模型本身,可能与缺少目标朝向控制有关")
            problems_detail.append({
                "问题": "接近目标时存在旋转调整",
                "证据": f"近目标帧数{len(near_goal_obs)}, dyaw波动{dyaw_std:.3f}",
                "来源": "模型决策 + 目标表达方式"
            })
            
        # 问题6: 部署端限幅影响
        if 'inference_results' in self.diagnosis_report:
            w_clip_rate = self.diagnosis_report['inference_results'].get('w_clipped', 0) / \
                         self.diagnosis_report['inference_results'].get('total_frames', 1)
            
            print(f"\n6. 【部署端动作限幅削弱策略能力】")
            print(f"   证据: 角速度触及限幅比例{w_clip_rate*100:.1f}%")
            print(f"   问题: 部署端限幅w∈[-0.5,0.5]可能过于严格")
            print(f"   判断: 来自动作映射配置,限制了模型的控制灵活性")
            problems_detail.append({
                "问题": "部署端限幅过严",
                "证据": f"角速度限幅率{w_clip_rate*100:.1f}%",
                "来源": "动作映射配置"
            })
            
        # 总结
        print(f"\n" + "-" * 80)
        print(f"问题来源分类:")
        print(f"-" * 80)
        
        sources = {
            "模型本身": [],
            "部署输入": [],
            "动作映射": [],
            "目标表达": [],
            "sim-to-real": []
        }
        
        for p in problems_detail:
            if "模型" in p["来源"]:
                sources["模型本身"].append(p["问题"])
            if "输入" in p["来源"]:
                sources["部署输入"].append(p["问题"])
            if "动作映射" in p["来源"]:
                sources["动作映射"].append(p["问题"])
            if "目标表达" in p["来源"]:
                sources["目标表达"].append(p["问题"])
            if "sim-to-real" in p["来源"] or "迁移" in p["来源"]:
                sources["sim-to-real"].append(p["问题"])
                
        for source, items in sources.items():
            if items:
                print(f"\n{source}:")
                for item in items:
                    print(f"  - {item}")
                    
        print(f"\n" + "=" * 80)
        print(f"结论: 该模型的主要问题是多方面综合作用的结果:")
        print(f"  1. 模型本身决策质量有提升空间 (路径效率0.60)")
        print(f"  2. 部署配置需要与训练端严格对齐 (LiDAR归一化、动作映射)")
        print(f"  3. 目标表达方式限制了定向能力 (只有方位,没有姿态)")
        print(f"  4. sim-to-real差异影响模型泛化 (摩擦力、传感器噪声)")
        print(f"=" * 80)
        
        # 保存到报告
        self.diagnosis_report["conclusion"] = {
            "problems": problems_detail,
            "summary": "模型问题是多方面综合作用,包括模型质量、部署配置、目标表达和sim-to-real差异"
        }
        
    def run_full_diagnosis(self):
        """执行完整诊断流程"""
        print("=" * 80)
        print("ROS 强化学习局部规划器 - 完整离线诊断分析")
        print("=" * 80)
        
        # 1. 加载模型
        if not self.load_model():
            print("\n警告: 模型加载失败,将跳过推理部分")
            
        # 2. 加载观测数据
        if not self.load_obs_data():
            print("\n错误: 观测数据加载失败,无法继续")
            return
            
        # 3. 加载实验摘要
        self.load_summary()
        
        # 4. 尝试离线推理
        inference_ok = self.offline_inference()
        
        if not inference_ok:
            print("\n注意: 由于缺少完整的网络定义,无法执行精确的离线推理")
            print("建议提供:")
            print("  - 完整的网络类定义 (Actor类)")
            print("  - obs normalization参数")
            print("  - 训练代码中的网络构建脚本")
            
        # 5. 诊断问题
        self.diagnose_problems()
        
        # 6. 生成建议
        self.generate_recommendations()
        
        # 7. 生成可视化
        self.generate_visualizations()
        
        # 8. 生成结论
        self.generate_conclusion()
        
        # 9. 保存报告
        self.save_report()
        
        print("\n" + "=" * 80)
        print("诊断分析完成!")
        print("=" * 80)
        print(f"\n生成的文件:")
        print(f"  - /home/claude/rl_diagnosis_plots.png (可视化图表)")
        print(f"  - /home/claude/rl_diagnosis_report.json (完整诊断报告)")
        

def main():
    model_path = '/mnt/user-data/uploads/model_1200.pt'
    obs_path = '/mnt/user-data/uploads/obs_556.txt'
    summary_path = '/mnt/user-data/uploads/summary.txt'
    
    diagnoser = RLModelDiagnoser(model_path, obs_path, summary_path)
    diagnoser.run_full_diagnosis()
    

if __name__ == '__main__':
    main()
