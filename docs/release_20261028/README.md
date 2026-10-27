# 新工程审查、整理与发布 · 2026-10-28

本次整理对象为队友扩展后的 `piper_classify_ws`，原项目已更名为 `piper_classify_ws_old`，旧目录未做源码修改。以 GitHub 原 `main` 的 `a7c6ba4` 为基线，保留此前未推送的 SSH/DDS 修复和本次语言功能。

## 工程现状与审查范围

工程为 11 个 ROS 包：原有 9 包加 `classify_agent`、`classify_voice`，清单见 [packages.json](packages.json)。重点审查了新增 Python 包、RobotCommand 服务、C++ 任务状态/选择/槽位/持物路径、启动环境与生命周期脚本、包依赖、RViz 配置和现有测试；对旧工程做了逐文件内容对照。原相机模型、网格、核心感知和场景跟踪实现保留。

新增能力：文本/可选语音 → 规则或 LLM 解析 → `/classify/command` → C++ MTC 执行；支持颜色、数量、选择器、跨色料盒、单件持物和 HOME。旧语音节点使用批量 Trigger 接口，属于独立兼容入口，不与新代理混为一套协议。

本次执行的是本机仿真验收，没有 SSH 修改其他电脑，没有操作真机或调用付费 LLM。当前环境版本及未覆盖项目见 [environment.json](environment.json)。

## 文件整理

- 工作区外建立私有备份：`../piper_classify_ws_release_backup_20261028_012507/`。包含整理前源码快照、原 Git bundle、未提交差异、原构建/安装/日志、编辑器设置/索引，以及不完整模型目录。
- 原 `build/ install/ log/` 含队友机器的 `/home/wujian/...` 路径，移出后从头构建；本机新构建结果保留在忽略目录，便于继续运行，不上传。
- 移出约 1.6 GB VS Code 浏览索引，将本机索引路径改到用户缓存目录，并整理本机 include 路径。`.vscode/` 仍不入 Git。
- 根目录 `faster-whisper-base` 缺少 `model.bin`，只是 tokenizer/config 和下载缓存，不能当作完整模型。已保存在备份中；README 提供官方完整下载方法，ASR 权重与缓存不入 Git。
- 队友节点图移到 [docs/images/rosgraph.png](../images/rosgraph.png)；原 `rosdaima.sh` 是多终端命令笔记，不是顺序可执行脚本，归档为 [终端草稿](../agent_demo_20261019/terminal_notes_20261021.txt)。
- 补充模型/下载缓存/egg-info 等忽略规则；保留历史设计、失败试验与最终验收记录，通过 [文档索引](../README.md) 区分，不移动破坏已有相对引用。
- 重写根 README，更新交接、语言说明、通信报告、doctor 指南和第三方声明；补齐旧语音包的描述、MIT 声明及所带 Apache 测试模板许可证。

## 发布前修复

### 失败/取消后的会话锁定回退

队友版 `start_blocked_locked()` 只检查持物状态，允许失败后重新进入动作。独立仿真用例请求基线中不存在的第二个红块，触发 `FAILED` 后再次发 HOME；原代码错误返回 accepted=True。复现见 [guard_before.json](guard_before.json)。

修复后 `FAILED`、`CANCELLED`、`HOLDING` 或实际持物均要求重置；成功语言放置后仍允许新指令。旧批量入口利用持久的已放置记录拒绝重复启动，即使中间执行过 HOME 也不会丢失保护。增加 [服务会话验证工具](../../scripts/tools/verify_command_session.py) 对真实节点验证，证据见 [guard_after.json](guard_after.json)、[cancel.json](cancel.json) 和 [cross_bin.json](cross_bin.json)。

### 持物与旧语音重置

`pick_hold` 在机器人服务端只接受 quantity=1，在运动前拒绝 0 或多件请求；持物后 HOME 也被拒绝，见 [hold.json](hold.json)。

旧语音节点原先把 Gazebo 世界重置当作任务重置，且使用错误的 Trigger 服务类型。现在提示使用外部 `reset_classify.sh` 并立即停止动作队列；回归测试确认既不调用 Gazebo 重置，也不落入后续 start。此节点的可选依赖改为初始化时加载，便于在无音频依赖的环境审查/测试。

### 文档、查询与依赖

- 语言代理不再把超过 2 秒且刷新失败的场景缓存用于报数；新增生产方法回归测试。`use_voice` 开关控制录音入口。
- 规则解析与 LLM 提示中“两件”的数量示例对齐为 2；文档按服务实际语义说明指定数量不足时会失败。
- 补充 URDF 校验实际需要的 numpy/scipy 依赖，纠正 README 的 MTC 包名及 world/base_link 坐标说明。
- doctor 版本更新为 `2026-10-28.1`，检查当前全部 11 包，保留渲染、图像、时钟、检测和 scene_ready 验证。
- 语音包原有 flake8/pep257 失败已修正，版权检查补齐声明后启用，没有以跳过检查获得通过。

## 本次验证

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| 干净构建及修复后重建 | 11 包完成，原电脑 CMake 绝对路径残留为 0 | [build.log](build.log) |
| 全量自动测试 | colcon 汇总 46 tests，0 errors / failures / skipped | [test_results.log](test_results.log)、[tests.log](tests.log) |
| Bash / Python / RViz 静态检查 | 15 个 shell、24 个 src Python 文件及新增会话工具语法通过，RViz YAML 可解析 | 静态检查与发布清单 |
| 基线场景 / 相机静态链 | 通过，输出写到 log，不覆盖历史校准 JSON | [scene_static.log](scene_static.log)、[camera_static.log](camera_static.log) |
| RGB-D / DDS 在线 | 基线与五块压力场景均 PASS；三路约 14.9–15 Hz，RTF 约 1，场景就绪 | [基线](doctor_online.log)、[11 包新版/五块](doctor_final.log) |
| 失败与取消锁定 | 两个入口均拒绝重试 | [guard_after.json](guard_after.json)、[cancel.json](cancel.json) |
| 跨色放置及后续 HOME | 红块进入蓝盒；原蓝块不动；HOME 被接受；旧批量拒绝重跑 | [cross_bin.json](cross_bin.json) |
| 抓起保持 | 单件蓝块抬升，HOLDING，拒绝后续 HOME | [hold.json](hold.json) |
| 文本代理查询 | 规则解析报数，与当前可见蓝块 1 件一致 | [agent_query.log](agent_query.log) |
| 原三块分拣 | 2/2，约 78.87 s，绿块位移 0.000 mm，HOME 恢复 | [batch.json](batch.json) |
| 五块 seed11 深度压力 | 4/4，约 156.83 s，绿块位移 0.000 mm，HOME 恢复 | [multi.json](multi.json)、[初始相机](multi_initial_camera.png) |

三块场景最终场景/实体中心最大差约 0.463 mm。HOME 容差沿用原验收工具的 0.065 rad，以上结果均属于仿真。

本机默认 GLX renderer 为 Intel Mesa 硬件渲染；装有 NVIDIA 驱动并不表示本次选用 NVIDIA。没有为发布修改显卡驱动或全局系统配置。运行日志保存在忽略目录，报告中只保留选定验收材料。

当前用户 Python 环境中存在 anyio pytest 插件与系统 pytest 的不兼容；测试时用 `PYTHONNOUSERSITE=1` 隔离用户插件，未卸载或修改全局 Python。numpy/scipy 用于校准和仿真探针的版本记录在 environment.json；可选语音/LLM 依赖没有安装或自动下载。

## 验证边界与继续开发

- 本次运动验证均以 `--headless` 执行；RViz 新布局只做静态解析，没有重做 GUI 面板视觉验收。手动操作前按根 README 检查场景与控制器。
- 麦克风、ASR 转写准确率、TTS、外部 DeepSeek/OpenAI 兼容服务未实测；不把规则解析和服务测试说成语音识别验收。
- 保留已知 50 mm 水平方块和理想固定夹持约束。未验收真机 CAN、力控、滑移、设备标定或任意未知物体。
- API Key 只从环境变量读取；原始本地构建日志、模型、编辑器数据库和工作区外备份不进入仓库。
- 本次没有把旧目录中额外的本地诊断草稿盲目合并进来；当前复现入口为根 README、doctor 和文档索引。

交付时已停止本项目测试进程，保留本机可用的构建结果。源码与选定证据提交到原公开仓库 `13646997215/piper_classify_ws` 的 main；具体提交号以 Git 历史为准。

发布检查见 [publish_checks.json](publish_checks.json)，交付关闭状态见 [closeout.json](closeout.json)。
