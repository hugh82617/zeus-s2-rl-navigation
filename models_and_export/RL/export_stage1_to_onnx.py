import torch
import torch.nn as nn
import onnx

CHECKPOINT_PATH = 'model_1999.pt'
OUTPUT_ONNX_PATH = 'zeus_stage1_model_1999.onnx'
OBS_DIM = 194
ACT_DIM = 2

class EmpNorm(nn.Module):
    def __init__(self, mean, std, eps=1e-8):
        super().__init__()
        self.register_buffer('mean', mean.clone().float())
        self.register_buffer('std', std.clone().float())
        self.eps = eps

    def forward(self, x):
        return (x - self.mean) / (self.std + self.eps)

class PolicyExport(nn.Module):
    def __init__(self, model_state_dict):
        super().__init__()
        self.norm = EmpNorm(
            model_state_dict['actor_obs_normalizer._mean'],
            model_state_dict['actor_obs_normalizer._std']
        )
        self.actor = nn.Sequential(
            nn.Linear(OBS_DIM, 256), nn.ELU(),
            nn.Linear(256, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, ACT_DIM)
        )
        with torch.no_grad():
            self.actor[0].weight.copy_(model_state_dict['actor.0.weight'])
            self.actor[0].bias.copy_(model_state_dict['actor.0.bias'])
            self.actor[2].weight.copy_(model_state_dict['actor.2.weight'])
            self.actor[2].bias.copy_(model_state_dict['actor.2.bias'])
            self.actor[4].weight.copy_(model_state_dict['actor.4.weight'])
            self.actor[4].bias.copy_(model_state_dict['actor.4.bias'])
            self.actor[6].weight.copy_(model_state_dict['actor.6.weight'])
            self.actor[6].bias.copy_(model_state_dict['actor.6.bias'])

    def forward(self, obs):
        obs = self.norm(obs)
        action = self.actor(obs)
        return action


def main():
    ckpt = torch.load(CHECKPOINT_PATH, map_location='cpu')
    model = PolicyExport(ckpt['model_state_dict']).eval()
    dummy_input = torch.zeros(1, OBS_DIM, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        OUTPUT_ONNX_PATH,
        input_names=['obs'],
        output_names=['action'],
        dynamic_axes={
            'obs': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        },
        opset_version=17,
        dynamo=False,
    )

    onnx_model = onnx.load(OUTPUT_ONNX_PATH)
    onnx.checker.check_model(onnx_model)
    print(f'导出成功: {OUTPUT_ONNX_PATH}')

if __name__ == '__main__':
    main()
