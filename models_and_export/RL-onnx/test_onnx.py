import onnxruntime as ort
import numpy as np

sess = ort.InferenceSession("model_1200_actor.onnx")

x = np.random.randn(1, 556).astype(np.float32)
y = sess.run(None, {"obs": x})[0]

print("输出 shape:", y.shape)
print("输出:", y)