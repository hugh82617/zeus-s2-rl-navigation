import torch

pt_path = "model_1200.pt"

ckpt = torch.load(pt_path, map_location="cpu")

print("对象类型:", type(ckpt))

if isinstance(ckpt, dict):
    print("这是一个 checkpoint / dict，包含键：")
    for k in ckpt.keys():
        print(" -", k, type(ckpt[k]))
else:
    print("这不是 dict，可能是直接模型")