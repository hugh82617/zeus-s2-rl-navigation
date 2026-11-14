# ONNX Runtime C++ 安装清单

## 推荐版本

在没有目标机器人现场环境信息时，建议先使用 CPU 版 ONNX Runtime C++：

- 首选版本：`1.18.1`
- 架构为普通工控机/笔记本/NUC：下载 `onnxruntime-linux-x64-1.18.1.tgz`
- 架构为 ARM64/Jetson：下载 `onnxruntime-linux-aarch64-1.18.1.tgz`

选择 `1.18.1` 的原因：

- C++ API 已经稳定，适合 ROS1 插件链接。
- CPU 版不依赖 CUDA、cuDNN、TensorRT，部署风险最低。
- 对常见 PyTorch/Isaac 导出的 ONNX actor 网络通常足够。
- 比很新的版本更适合老一点的 Ubuntu/ROS 环境。

如果机器人上的 Python `onnxruntime` 版本已经确定，最理想的做法是让 C++ 版和 Python 版保持同一版本：

```bash
/home/ros/miniconda3/envs/onnx_env/bin/python -c "import onnxruntime as ort; print(ort.__version__)"
```

如果输出不是 `1.18.1`，也可以下载对应版本的 C++ 包，例如 Python 是 `1.16.3`，就下载 `onnxruntime-linux-x64-1.16.3.tgz`。

## 需要安装的文件

C++ 单插件真正需要的是：

```text
/opt/onnxruntime/include/onnxruntime_cxx_api.h
/opt/onnxruntime/include/onnxruntime_c_api.h
/opt/onnxruntime/lib/libonnxruntime.so
```

ONNX Runtime 官方 C++ 压缩包里通常还会包含其他头文件和库文件，全部保留即可，不要只复制单个文件。

## x86_64 机器人安装命令

```bash
export ORT_VERSION=1.18.1

cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz

sudo mkdir -p /opt/onnxruntime-${ORT_VERSION}
sudo tar -xzf onnxruntime-linux-x64-${ORT_VERSION}.tgz \
  --strip-components=1 \
  -C /opt/onnxruntime-${ORT_VERSION}

sudo ln -sfn /opt/onnxruntime-${ORT_VERSION} /opt/onnxruntime

echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
```

## ARM64/Jetson CPU 版安装命令

```bash
export ORT_VERSION=1.18.1

cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-aarch64-${ORT_VERSION}.tgz

sudo mkdir -p /opt/onnxruntime-${ORT_VERSION}
sudo tar -xzf onnxruntime-linux-aarch64-${ORT_VERSION}.tgz \
  --strip-components=1 \
  -C /opt/onnxruntime-${ORT_VERSION}

sudo ln -sfn /opt/onnxruntime-${ORT_VERSION} /opt/onnxruntime

echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
```

## 验证安装

```bash
ls /opt/onnxruntime/include/onnxruntime_cxx_api.h
ls /opt/onnxruntime/lib/libonnxruntime.so*
ldconfig -p | grep onnxruntime
```

期望能看到：

```text
/opt/onnxruntime/include/onnxruntime_cxx_api.h
/opt/onnxruntime/lib/libonnxruntime.so
libonnxruntime.so => /opt/onnxruntime/lib/libonnxruntime.so
```

## ROS 编译时需要的 CMake 参数

后续把 `OnnxPolicy` 加进插件后，推荐 CMake 使用：

```bash
catkin_make \
  -DONNXRUNTIME_INCLUDE_DIR=/opt/onnxruntime/include \
  -DONNXRUNTIME_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so
```

或者写进包的 `CMakeLists.txt` 默认路径：

```cmake
set(ONNXRUNTIME_INCLUDE_DIR "/opt/onnxruntime/include" CACHE PATH "ONNX Runtime include directory")
set(ONNXRUNTIME_LIBRARY "/opt/onnxruntime/lib/libonnxruntime.so" CACHE FILEPATH "ONNX Runtime shared library")
```

## 不建议优先使用 GPU 版

第一版单插件建议只使用 CPU 版 ONNX Runtime。

GPU 版会额外依赖 CUDA、cuDNN、TensorRT、显卡驱动版本匹配，尤其在 Jetson 上更容易因为版本组合不一致导致编译或运行失败。当前 actor 网络通常较小，CPU 推理更容易稳定上线。

## 离线安装方式

如果机器人不能联网，可以在其他电脑下载对应 `.tgz`，拷贝到机器人 `/tmp` 后执行：

```bash
export ORT_VERSION=1.18.1

sudo mkdir -p /opt/onnxruntime-${ORT_VERSION}
sudo tar -xzf /tmp/onnxruntime-linux-x64-${ORT_VERSION}.tgz \
  --strip-components=1 \
  -C /opt/onnxruntime-${ORT_VERSION}

sudo ln -sfn /opt/onnxruntime-${ORT_VERSION} /opt/onnxruntime
echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig
```

ARM64 时把文件名换成：

```text
onnxruntime-linux-aarch64-1.18.1.tgz
```
