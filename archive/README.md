# 历史代码与修订

- [node_versions](node_versions)：原节点1至12、now、备份目录和各控制规则版本，保留来源相对结构。
- [plugin_fixed](plugin_fixed)：另一份C++插件修订，原目录名含fixed；名称不证明其更晚或更完善。
- [plugin_zip_original](plugin_zip_original)：原始rl_planner.zip独有的四份历史文本，与展开目录不同。

源码逻辑保持原样；这些是开发快照，不是互相可以直接替换的发布版本。未复制IDE缓存、pyc或重复压缩包。同内容历史模型二进制已去重，见 `binary_duplicates.json`；主Python/C++包和模型导出目录保留完整权重配套。

v22/v24包含近目标门控、限速、障碍规则和转向修正，属于部署外层控制，并不等于PPO网络被重新训练。历史README中“生产就绪”等措辞为原文表述，未在本次整理中验证。
