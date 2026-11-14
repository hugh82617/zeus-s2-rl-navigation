import torch

ckpt = torch.load("model_1200.pt", map_location="cpu")
sd = ckpt["model_state_dict"]

print("state_dict 参数个数:", len(sd))
print("=" * 60)

for i, (k, v) in enumerate(sd.items()):
    if hasattr(v, "shape"):
        print(f"{i:03d} | {k:50s} | shape={tuple(v.shape)}")
    else:
        print(f"{i:03d} | {k:50s} | type={type(v)}")