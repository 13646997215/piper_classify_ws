# 文档与证据索引

当前工程共 11 个 ROS 包。启动和构建以 [根 README](../README.md) 为准，接口以当前 `src/` 为准；带日期的记录用于追溯，不代表当前版本仍使用当时全部参数。

## 当前说明

| 文档 | 用途 |
| --- | --- |
| [2026-10-28 发布整理](release_20261028/README.md) | 新工程差异、清理、修复、实测与未覆盖范围 |
| [交接说明](05_交接说明.md) | 常用操作，附各阶段历史数据 |
| [节点与通信报告](07_系统架构与通信报告.md) | 启动链、话题、服务、QoS、TF 与参数 |
| [语言演示](agent_demo_20261019/README.md) | 文本/语音入口和 RobotCommand |
| [doctor 说明](../doctor_使用与交付说明.md) | 图形会话、DDS、在线诊断与恢复 |

## 历史设计与验收

| 路径 | 内容 |
| --- | --- |
| [规划目标](01_规划目标.md)、[任务背景](03_任务要求与背景.md) | 原始任务和验收目标 |
| [开发进程](02_开发进程记录.md)、[输出记录](4_输出.md)、[接管审查](06_接管审查.md) | 开发/审查历史 |
| [audit_20260930](audit_20260930/README.md) | 早期几何、料盒与姿态审查 |
| [perception_motion_20260930](perception_motion_20260930/README.md) | 运动时视觉、TF、新鲜度 |
| [task_20261001](task_20261001/README.md) | 三块自动分拣、位移测试与截图 |
| [gemini335_20261001](gemini335_20261001/README.md) | 光学参数、手眼矩阵、视图验证 |
| [gemini335_fov_20261001](gemini335_fov_20261001/README.md) | 真机参考视场核对 |
| [reset_20261001](reset_20261001/README.md) | 生命周期与重置修复 |
| [robustness_20261002](robustness_20261002/README.md) | 障碍、扰动、遮挡、取消和深度压力 |
| [multi_object_20261002](multi_object_20261002/README.md) | 同色多块 ID、yaw、分槽和容量 |
| [closeout_20261002](closeout_20261002/README.md) | 原阶段收尾清单 |
| [github_20261002](github_20261002/README.md) | 首次 GitHub 清理与构建验证 |
| [remote_deployment_20261010](remote_deployment_20261010/README.md) | 另一台机器上的 SSH / Fast DDS 故障对照 |

历史目录同时保留成功、失败及调参试验。请以各目录 README 标注的最终接受记录为准；`before`、`trial`、`reference`、`runs` 等文件不属于当前运行源码。历史绝对路径和旧源文件 hash 是当时的记录，不应替换为当前机器路径。

原始 RGB-D `.npz`、ASR 权重、编辑器索引、编译结果和运行日志不作为新源码交付内容；选定的报告日志和 JSON 会保留在日期目录。队友提供的节点图位于 [images/rosgraph.png](images/rosgraph.png)，原终端草稿保存为 [terminal_notes_20261021.txt](agent_demo_20261019/terminal_notes_20261021.txt)，草稿中的命令以当前说明为准。
