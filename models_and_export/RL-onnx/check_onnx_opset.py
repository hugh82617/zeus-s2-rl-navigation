import onnx

model = onnx.load("model_1200_actor.onnx")

for opset in model.opset_import:
    print("domain:", opset.domain, "version:", opset.version)