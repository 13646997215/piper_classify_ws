# 语言驱动机器人演示 · 2026-10-19

本阶段把固定分拣流水线包装成一个可分步展示的语言智能体：

```text
文本 / 语音
  -> 大模型（DeepSeek / 任意 OpenAI 兼容端点）
  -> 受 schema 约束的 CommandSpec
  -> /classify/command
  -> 现有视觉 + MTC 抓放
  -> /classify/status /classify/result 回读
```

大模型只负责理解“拿什么、放到哪”，不会输出关节角、速度或任意坐标。所有运动、碰撞、抓取和落盒检查仍由 C++ 任务节点执行。

## 支持的说法

| 输入示例 | 解析结果 |
|---|---|
| 把红色方块放到蓝色盒子里 | `pick_place`，红块 → 蓝盒 |
| 拿一个蓝色方块 | `pick_hold`，抓起蓝块并保持 |
| 把两个红块都放进红盒 | `pick_place`，数量 2 → 红盒 |
| 拿左边的红块 | `pick_place`/`pick_hold`，selector=`leftmost` |
| 拿最近的红块 | selector=`nearest` |
| 桌上有几个红块 | `count` |
| 扫描 / 看看桌上有什么 | `scan` |
| 回家 | `home` |
| 停止 | 调用 `/classify/cancel` |
| 重置 | 提示运行 `bash scripts/reset_classify.sh` |

绿色是参考物，任何“搬绿块”的请求都会被拒绝。

## 启动

先启动完整仿真（默认不运动）：

```bash
cd ~/piper_classify_ws
bash scripts/reset_classify.sh
```

等 Gazebo/RViz 显示 `VISION_READY` 后，在另一个终端启动语言 agent：

```bash
cd ~/piper_classify_ws
bash scripts/run_agent.sh
```

然后直接输入：

```text
你 > 把红色方块放到蓝色盒子里
```

也可用语音（需要 `faster-whisper`、`sounddevice`）：

```bash
bash scripts/run_agent.sh --ros-args -p use_voice:=true
# 在提示符下输入 voice，录音 4 秒
```

单条命令演示（执行完自动退出，适合录屏/自动化）：

```bash
bash scripts/run_agent.sh --ros-args -p command:="拿一个蓝色方块"
```

TTS 播报（可选，需要 `espeak-ng` / `espeak` / `spd-say`）：

```bash
bash scripts/run_agent.sh --ros-args -p enable_tts:=true
```

## LLM 配置

默认只使用本地规则解析，`use_llm=false`；不会因为终端已有密钥就自动调用云端。使用者自行接入 API，仓库不含真实 Key。以公开 DeepSeek 服务为例：

```bash
cp -n .env.example .env
# 在本地 .env 中填写自己的 Key；.env 被 Git 忽略
set -a; source .env; set +a
bash scripts/run_agent.sh --ros-args -p use_llm:=true
```

也可换成本地 OpenAI 兼容端点，例如 Ollama：

```bash
ollama serve
ollama pull qwen2.5:7b
DEEPSEEK_API_KEY=ollama bash scripts/run_agent.sh --ros-args \
  -p use_llm:=true \
  -p llm_model:=qwen2.5:7b \
  -p llm_base_url:=http://localhost:11434/v1
```

当前节点从 `DEEPSEEK_API_KEY` 读取密钥；也可用 `llm_api_key_env` 指定其他环境变量名，配合 `llm_base_url`、`llm_model` 接入自己的服务。本地端点可用 `DEEPSEEK_API_KEY=ollama` 占位。不要把真实密钥写入源码、公开示例或日志。

## 任务接口

`classify_interfaces/srv/RobotCommand.srv`：

```text
string intent        # pick_place | pick_hold | home
string[] colors      # ["red"] / ["blue"]
int32 quantity       # 0 = 所有新鲜匹配对象；>0 = 指定 N 件，鲜对象不足则失败；pick_hold 仅允许 1
string target_bin    # red | blue | ""（空 = 同色盒）
string selector      # nearest | leftmost | rightmost | first
---
bool accepted
string message
```

任务节点仍会做：

- 新鲜视觉检查与对象选择；
- 料盒槽位预留与容量检查；
- MTC 规划、碰撞检查、接触夹持；
- 实际抬升/落盒/绿块位移核对；
- 失败/取消/持物后锁定，必须重置仿真。

## 当前限制

- 只支持红色、蓝色 50 mm 水平方块；绿色只作为参考。
- `count/scan` 统计的是取料区新鲜对象；已入盒对象由任务结果记录，不再计入可见数量。
- `leftmost/rightmost` 按 `base_link` 的 +Y/−Y 解释。
- `pick_hold` 结束后夹爪保持持物，任务状态为 `HOLDING`，再次操作必须重置。
- `reset` 仍由外部 `reset_classify.sh` 完成；agent 只给出提示，避免在任务进程内重启世界。
- LLM 只做高层解析；未接入开放词汇检测、任意物体抓取或多轮澄清（这些是后续阶段）。
- 没有真实设备 K/D、手眼残差、TCP 或力反馈数据；本演示仍是仿真闭环。

## 新增文件

```text
src/interfaces/classify_interfaces/srv/RobotCommand.srv
src/classify_agent/
  classify_agent/agent_node.py   # 文本/语音、LLM、服务调用、状态播报
  classify_agent/parser.py       # schema 校验 + 确定性兜底解析
  classify_agent/llm.py          # OpenAI 兼容客户端
  launch/agent.launch.py
  test/test_parser.py
scripts/run_agent.sh
```

`classify_task` 增加了 `/classify/command`、任意目标料盒、数量/选择器和 `pick_hold`。原有 `/classify/start` 三块/五块基线入口保留。2026-10-28 发布整理恢复了失败/取消后的锁定，并使已放置后的批处理入口要求重置；成功的语言放置命令仍允许继续提交新指令。

语音默认模型目录为 `~/models/faster-whisper-medium`，可用 `voice_model` 参数覆盖。本仓库不包含模型权重，完整下载方法见根 README。旧 `classify_voice` 使用批量 Trigger 接口，不支持这个 RobotCommand 协议；推荐使用 `classify_agent`。
