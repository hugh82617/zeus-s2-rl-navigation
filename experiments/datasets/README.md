# 原始实验数据

保留原文件名和分层。`900批次8个csv文件(2)/900批次csv文件`包含4份episode CSV和4份step CSV；`数据/ESA1/ESA`与`数据/dwa-1/dwa-1`分别保存RL与DWA实车summary/观测。`exp_data`保存22张实验截图。

统计入口：[实验说明](../README.md)、[可重算脚本](../analysis/analyze_episodes.py)。CSV/TXT没有按成功结果过滤、重新标注或改写。success、outcome、finish_reason等字段应按各自记录器解释，不能互相替代。
