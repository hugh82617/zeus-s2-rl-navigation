import os
import torch
import torch.nn as nn
import onnx
import onnxruntime as ort

CKPT = '/mnt/data/unz/#U6700#U65b0#U6a21#U578b#U53ca#U89c2#U6d4b#U8981#U6c42/model_1200.pt'
OUT = '/mnt/data/model_1200_actor.onnx'

class RunningNorm(nn.Module):
    def __init__(self, mean: torch.Tensor, std: torch.Tensor):
        super().__init__()
        self.register_buffer('mean', mean.clone().float())
        self.register_buffer('std', std.clone().float())

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return (x - self.mean) / self.std

class ActorOnly(nn.Module):
    def __init__(self, state_dict):
        super().__init__()
        mean = state_dict['actor_obs_normalizer._mean']
        std = state_dict['actor_obs_normalizer._std']
        self.norm = RunningNorm(mean, std)
        self.actor = nn.Sequential(
            nn.Linear(556, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, 2),
        )
        self._load_from_ckpt(state_dict)

    def _load_from_ckpt(self, sd):
        own = self.state_dict()
        mapping = {
            'actor.0.weight': 'actor.0.weight',
            'actor.0.bias': 'actor.0.bias',
            'actor.2.weight': 'actor.2.weight',
            'actor.2.bias': 'actor.2.bias',
            'actor.4.weight': 'actor.4.weight',
            'actor.4.bias': 'actor.4.bias',
            'actor.6.weight': 'actor.6.weight',
            'actor.6.bias': 'actor.6.bias',
        }
        for src, dst in mapping.items():
            own[dst].copy_(sd[src])
        self.norm.mean.copy_(sd['actor_obs_normalizer._mean'])
        self.norm.std.copy_(sd['actor_obs_normalizer._std'])

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        obs = self.norm(obs)
        return self.actor(obs)

ckpt = torch.load(CKPT, map_location='cpu')
sd = ckpt['model_state_dict']
model = ActorOnly(sd).eval()

dummy = torch.zeros(1, 556, dtype=torch.float32)

with torch.no_grad():
    torch_out = model(dummy)

# Export ONNX
with torch.no_grad():
    torch.onnx.export(
        model,
        dummy,
        OUT,
        input_names=['obs'],
        output_names=['action'],
        dynamic_axes={'obs': {0: 'batch_size'}, 'action': {0: 'batch_size'}},
        opset_version=13,
        do_constant_folding=True,
        export_params=True,
        dynamo=False,
    )

# Validate ONNX
onnx_model = onnx.load(OUT)
onnx.checker.check_model(onnx_model)

sess = ort.InferenceSession(OUT, providers=['CPUExecutionProvider'])
ort_out = sess.run(['action'], {'obs': dummy.numpy()})[0]
max_abs = float(torch.max(torch.abs(torch_out - torch.from_numpy(ort_out))))

print(f'Exported to: {OUT}')
print(f'iter: {ckpt.get("iter")}')
print(f'ONNX size: {os.path.getsize(OUT)} bytes')
print(f'Max abs diff (dummy input): {max_abs:.8f}')
