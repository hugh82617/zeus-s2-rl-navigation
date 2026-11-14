import onnxruntime as ort
import numpy as np

sess = ort.InferenceSession("model_1200_actor.onnx")

print("inputs:")
for x in sess.get_inputs():
    print(x.name, x.shape, x.type)

print("outputs:")
for y in sess.get_outputs():
    print(y.name, y.shape, y.type)

x = np.random.randn(1, 556).astype(np.float32)
y = sess.run(None, {"obs": x})[0]

print("output shape:", y.shape)
print("output:", y)
