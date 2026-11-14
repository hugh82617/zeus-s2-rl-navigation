import torch
import torch.nn as nn


# ----------------------------
# 1) 定义观测归一化模块
# ----------------------------
class ObsNormalizer(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.register_buffer("_mean", torch.zeros(1, obs_dim))
        self.register_buffer("_var", torch.ones(1, obs_dim))
        self.register_buffer("_std", torch.ones(1, obs_dim))
        self.register_buffer("count", torch.tensor(1.0))

    def forward(self, x):
        return (x - self._mean) / self._std


# ----------------------------
# 2) 定义 Actor 网络
# ----------------------------
class ActorNet(nn.Module):
    def __init__(self, obs_dim=556, act_dim=2):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, act_dim),
        )

    def forward(self, obs):
        return self.actor(obs)


# ----------------------------
# 3) 定义最终导出的策略
#    = normalizer + actor
# ----------------------------
class ExportedPolicy(nn.Module):
    def __init__(self, obs_dim=556, act_dim=2):
        super().__init__()
        self.actor_obs_normalizer = ObsNormalizer(obs_dim)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, act_dim),
        )

    def forward(self, obs):
        obs = (obs - self.actor_obs_normalizer._mean) / self.actor_obs_normalizer._std
        action = self.actor(obs)
        return action


# ----------------------------
# 4) 加载 checkpoint
# ----------------------------
pt_path = "model_1200.pt"
onnx_path = "model_1200_actor.onnx"

ckpt = torch.load(pt_path, map_location="cpu")
sd = ckpt["model_state_dict"]

model = ExportedPolicy(obs_dim=556, act_dim=2)

# strict=True 最稳，如果结构不匹配会直接报错
model.load_state_dict(
    {
        "actor.0.weight": sd["actor.0.weight"],
        "actor.0.bias": sd["actor.0.bias"],
        "actor.2.weight": sd["actor.2.weight"],
        "actor.2.bias": sd["actor.2.bias"],
        "actor.4.weight": sd["actor.4.weight"],
        "actor.4.bias": sd["actor.4.bias"],
        "actor.6.weight": sd["actor.6.weight"],
        "actor.6.bias": sd["actor.6.bias"],
        "actor_obs_normalizer._mean": sd["actor_obs_normalizer._mean"],
        "actor_obs_normalizer._var": sd["actor_obs_normalizer._var"],
        "actor_obs_normalizer._std": sd["actor_obs_normalizer._std"],
        "actor_obs_normalizer.count": sd["actor_obs_normalizer.count"],
    },
    strict=True,
)

model.eval()

print("模型加载成功。")


# ----------------------------
# 5) 先做一次前向测试
# ----------------------------
dummy_input = torch.randn(1, 556)

with torch.no_grad():
    out = model(dummy_input)

print("前向输出 shape:", out.shape)
print("前向输出:", out)


# ----------------------------
# 6) 导出 ONNX
# ----------------------------
torch.onnx.export(
    model,
    dummy_input,
    onnx_path,
    export_params=True,
    opset_version=11,
    do_constant_folding=True,
    input_names=["obs"],
    output_names=["action"],
    dynamic_axes={
        "obs": {0: "batch_size"},
        "action": {0: "batch_size"},
    },
)

print(f"ONNX 导出完成: {onnx_path}")