import torch

x = torch.load("model_1999.pt", map_location="cpu")

print("对象类型:", type(x))

if isinstance(x, dict):
    print("顶层 keys:")
    for k, v in x.items():
        print(f"{k}: {type(v)}")
else:
    print(x)