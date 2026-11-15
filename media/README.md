# 小车项目视频

[观看或下载指定的答辩视频1](video1.mp4)

视频1已嵌入[仓库首页](../README.md)，可直接播放。用于首页的GitHub视频附件与仓库MP4对应关系记录在 [video1-homepage.json](video1-homepage.json)；它是同一段视频的展示副本，不增加独立视频数量。

视频1来自毕业答辩5.19目录，原文件完整复制，未剪辑或转码。它与RL目录中的 `3_clip_1m48s_2m56s.mp4` 字节相同，规范副本保留在仓库，RL视频包不重复收录。视频不作为后续C++强化学习插件已完成实车验证的证据。

共登记67个MP4原路径；SHA256去重后保留52个非空独立文件，14条重复路径仅保留索引，1个0字节文件不作为可播放素材上传。保留原始、压缩、加速和剪辑版本，只对字节完全相同文件去重，不把不同剪辑当作完全重复。

全部视频通过仓库与 [v1.0.0 Release](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/tag/v1.0.0) 提供。强化学习包使用6个二进制分卷，其余4类各提供一个完整ZIP；逐卷和完整包的SHA256见清单。

## 分类下载

| 分类 | 唯一非空视频数 | 原始字节合计 | 下载 |
|---|---:|---:|---|
| 强化学习部署与场景切片 | 27 | 1148.2 MiB | [6个分卷与合并方法](#rl-video-parts) |
| DWA动态避障 | 0 | 0.0 MiB | 未发现可独立归入此类的MP4；不据此认定未做该实验 |
| 自主导航与SLAM系统素材 | 4 | 436.1 MiB | [video-navigation_slam-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-navigation_slam-01.zip) |
| MATLAB控制导航实验 | 2 | 7.6 MiB | [video-matlab_control-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-matlab_control-01.zip) |
| 毕业答辩演示与加速剪辑 | 6 | 38.5 MiB | [视频1](video1.mp4)<br>[video-defense_demo-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-defense_demo-01.zip) |
| 诊断与对比剪辑素材 | 13 | 157.8 MiB | [video-diagnostics-01.zip](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-diagnostics-01.zip) |

ZIP使用不重压缩方式保存MP4，保留中文原名与原资料相对路径。解压后可用SHA256核对；大视频放Release，避免把全部原始视频写入Git历史。

<a id="rl-video-parts"></a>
## 强化学习视频分卷

下载下面6个文件到同一个目录，保持文件名不变。分卷仅为稳定传输，合并后是原始ZIP，27段视频字节不变。

| 分卷 | 大小 | 下载 |
|---|---:|---|
| 001 | 200 MiB | [part001](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part001) |
| 002 | 200 MiB | [part002](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part002) |
| 003 | 200 MiB | [part003](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part003) |
| 004 | 200 MiB | [part004](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part004) |
| 005 | 200 MiB | [part005](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part005) |
| 006 | 148.2 MiB | [part006](https://github.com/hugh82617/zeus-s2-rl-navigation/releases/download/v1.0.0/video-rl_deployment-01.zip.part006) |

克隆仓库后，在仓库根目录运行以下命令，将示例目录替换为保存分卷的目录。只需Python 3标准库，无需安装额外包：

```powershell
python media/assemble_rl_videos.py --parts-dir "D:\Downloads\zeus-videos"
```

也可单独下载 [合并脚本](assemble_rl_videos.py) 和 [分卷清单](rl-video-parts.json)，放在同一目录后运行 `python assemble_rl_videos.py --parts-dir "分卷目录"`。脚本逐卷校验，再核对完整ZIP；成功后在分卷目录生成 `video-rl_deployment-01.zip`，使用常规解压工具打开即可。已有内容不同的输出文件会被保留并报错。

完整ZIP大小为1,204,021,051字节，SHA256为 `48dbc41c7c1e188f3aa3dc06e3bb97b2f451468436f8178501dc8e217e711ca4`。分卷校验清单见 [rl-video-parts.json](rl-video-parts.json)；GitHub端资产大小与SHA256的核验记录见 [release-assets.json](../provenance/release-assets.json)。

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
