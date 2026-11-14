# 小车项目视频

[观看或下载指定的答辩视频1](video1.mp4)

视频1来自毕业答辩5.19目录，原文件完整复制，未剪辑或转码。它与RL目录中的 `3_clip_1m48s_2m56s.mp4` 字节相同，因此只保留仓库这一份。视频不作为后续C++强化学习插件已完成实车验证的证据。

共登记67个MP4原路径；SHA256去重后保留52个非空独立文件，14条重复路径仅保留索引，1个0字节文件不作为可播放素材上传。保留原始、压缩、加速和剪辑版本，只对字节完全相同文件去重，不把不同剪辑当作完全重复。

**Release链接说明**：下列目标文件名用于 `v1.0.0` Release。此清单在本地准备阶段生成；链接只有在对应资产上传完成后才可下载。仓库发布步骤应核查全部资产存在。

## 分类下载

| 分类 | 唯一非空视频数 | 原始字节合计 | 下载 |
|---|---:|---:|---|
| 强化学习部署与场景切片 | 27 | 1148.2 MiB | [video-rl_deployment-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip) |
| DWA动态避障 | 0 | 0.0 MiB | 未发现可独立归入此类的MP4；不据此认定未做该实验 |
| 自主导航与SLAM系统素材 | 4 | 436.1 MiB | [video-navigation_slam-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-navigation_slam-01.zip) |
| MATLAB控制导航实验 | 2 | 7.6 MiB | [video-matlab_control-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-matlab_control-01.zip) |
| 毕业答辩演示与加速剪辑 | 6 | 38.5 MiB | [视频1](video1.mp4)<br>[video-defense_demo-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-defense_demo-01.zip) |
| 诊断与对比剪辑素材 | 13 | 157.8 MiB | [video-diagnostics-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-diagnostics-01.zip) |

ZIP使用不重压缩方式保存MP4，保留中文原名与原资料相对路径。解压后可用SHA256核对；大视频放Release，避免把全部原始视频写入Git历史。

## 场景与证据说明

- RL分类包含2026年4月22日原始录制及crossing、diagnol（原目录拼写）、headon、wenti的场景切片。分类依据来源目录，不据视频文件名推导成功率、碰撞次数或控制器性能。
- 5.11分类保留完整录制、压缩版、截取版、2倍速无声版和对比剪辑等；没有逐帧审阅，不能仅凭“左右对比”认定左右分别使用哪个算法。
- MATLAB分类包含前瞻距离0.3与实物运行素材。导航分类包含定点导航、配置讲解和自主导航实物运行，作为导航系统材料；分类名中的SLAM不表示每段都直接展示了建图。
- 答辩分类包含视频1/2/3及各自15秒加速版。视频2/3与5.11对比视频相同，5.24媒体1/2与15秒版相同，只保留一份实体。
- 未在独立DWA目录中发现MP4。DWA对照数据与技术记录仍在项目其他部分；不能将RL目录中的动态避障视频标成DWA。

## 逐文件追溯

- [catalog.csv](catalog.csv)：67个原始路径，记录分类、规范副本、SHA256、文件大小和下载位置。
- [catalog.json](catalog.json)：独立视频、全部来源别名、MP4容器mvhd时长、Release包及去重记录。时长仅从容器元数据读取，未解码画面。
- 空文件 `5.11/5.11/左右同步对比视频_带间隔.mp4` 为0字节，已在catalog标明，未上传无效视频。
