# Piper RGB-D 视觉分拣与语言控制仿真

基于 **Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 / MoveIt 2 + MTC 0.1.3**。Piper 腕部 Gemini 335 RGB-D 模型定位 50 mm 方块，执行碰撞检查、接触夹持和入盒核对；支持 RViz 手动操作、三块基线、五块同色多目标场景，以及可选文本/语音代理。

2026-10-28 整理的是队友在原工程上扩展的版本：**11 个 ROS 包**，新增 `classify_agent`、`classify_voice` 和 `/classify/command` 服务。本次干净构建 11 包、46 项检查零失败，三块 2/2 与五块压力场景 4/4 均通过。当前构建、回归与整理清单见 [本次发布记录](docs/release_20261028/README.md)；早期验收记录保留各自日期，不等同于新版本验证。

本项目运行于仿真，没有真机 CAN 执行后端。语音识别需另行安装模型和音频依赖；纯文本规则解析及基础分拣无需 API Key。

## 功能与边界

- **感知**：RGB/Depth 640×480、15 fps，原生深度对齐 RGB，结合图像时刻 TF 定位；跟踪同色目标 ID 和方块 yaw。
- **批量分拣**：`/classify/start` 把红、蓝工件分别放到同色料盒；绿块保留。五块示例为两红、两蓝、一绿。
- **语言任务**：`/classify/command` 支持指定红/蓝颜色、数量、目标盒、最近/最左/最右目标，以及单件抓起保持、返回 HOME。成功放置后可继续提交语言命令。
- **恢复约定**：运行中拒绝新任务；失败、取消或 `HOLDING` 后需完整重置。已放置过对象后，批量入口也要求重置；语言入口可继续处理剩余新鲜目标。
- **交互**：RViz 中规划机械臂/夹爪、查看 MTC 阶段；可选深度噪声、缺失与障碍场景。
- **部署**：启动脚本提供 Fast DDS 每参与者 32 MiB 共享内存配置，尝试连接同一用户的 GNOME/Xwayland 显示会话；显式 RMW、DDS profile 和 DISPLAY 配置由用户控制。

## 系统结构

```text
Gazebo RGB + 原生深度 → classify_camera_sim 对齐 → classify_perception
                                                      │ 检测 + 图像时刻 TF
                                                      ▼
world 固定几何 ───────────────────────────→ classify_scene_bridge
                                                      │ MoveIt PlanningScene
                                                      ▼
文本/语音 → classify_agent → RobotCommand → classify_task / MTC
                                                      │ MoveIt / ros2_control
                                                      ▼
                                              Gazebo 执行与实体核对
```

旧 `classify_voice` 是独立的批量语音入口，只调用开始/取消/场景查询；它不使用 `RobotCommand`。新演示优先使用 `classify_agent`。LLM 只参与高层意图解析，运动目标、碰撞、视觉新鲜度和夹持检查由 C++ 节点负责。

| 包 | 职责 |
| --- | --- |
| `classify_bringup` | 完整启动编排 |
| `classify_description` | URDF/xacro、10 个网格、world、相机与控制器配置 |
| `classify_grasp_sim` | Gazebo 双指接触与固定夹持约束 |
| `classify_camera_sim` | 原生深度对齐 RGB，可选深度压力输入 |
| `classify_perception` | 颜色分割、三维中心与 yaw 估计 |
| `classify_scene_bridge` | 固定设施、视觉 ID、状态和 MoveIt 场景同步 |
| `classify_moveit_config` | 运动学、SRDF、规划、RViz 和手臂/夹爪验证 |
| `classify_interfaces` | 检测、场景、任务 msg 和 `RobotCommand.srv` |
| `classify_task` | MTC 抓放、指令会话、夹持与实体结果检查 |
| `classify_agent` | 文本/可选语音、规则/可选 LLM、状态反馈 |
| `classify_voice` | 保留兼容的独立批量语音节点 |

## 首次下载与构建

先安装并配置 ROS 2 Humble、colcon、rosdep、Gazebo Classic 11、MoveIt 2。MTC 使用以下包，`rosdep` 也会根据依赖声明解析它们：

```bash
sudo apt install ros-humble-moveit-task-constructor-core \
  ros-humble-moveit-task-constructor-visualization

git clone https://github.com/13646997215/piper_classify_ws.git
cd piper_classify_ws
source /opt/ros/humble/setup.bash
# rosdep 尚未初始化时先执行 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 先建立描述包的 ament 索引，供 xacro 查找资源
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to classify_description \
  --symlink-install --executor sequential
bash scripts/tools/build_urdf.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential
source install/local_setup.bash

python3 scripts/tools/check_scene.py
# 使用 log 输出目录，避免覆盖历史验收 JSON
python3 scripts/tools/verify_camera_model.py --output log/camera_static_check
```

不要复制他人机器的 `build/ install/ log/`。生成的 `piper_sim.urdf` 包含当前绝对资源路径；它和 `piper_sim_rviz.urdf` 均不入 Git。移动工作区后，应清除旧构建结果并按上面顺序重建。修改 xacro/相机配置后须重新生成 URDF 并重启仿真。

以下命令从工作区根目录执行。仓库可以放在任意路径；示例中的 `cd` 请使用自己的实际位置。

## 启动、分拣与重置

终端一：

```bash
bash scripts/run_classify.sh
```

默认打开 Gazebo、RViz，**不自动运动**。三个控制器 active、场景 `VISION_READY` 后，终端二加载环境：

```bash
cd /path/to/piper_classify_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source scripts/tools/runtime_env.sh
export ROS_DOMAIN_ID=42

ros2 service call /classify/scene_ready std_srvs/srv/Trigger '{}'
ros2 service call /classify/start std_srvs/srv/Trigger '{}'
ros2 topic echo /classify/result --once
```

服务应答只表示接收情况；最终结果看 `/classify/result`，过程看 `/classify/status`。需要取消、恢复初始物块或关闭时：

```bash
ros2 service call /classify/cancel std_srvs/srv/Trigger '{}'
bash scripts/reset_classify.sh     # 完整停止并重新加载初始 world；默认不运动
bash scripts/stop_classify.sh      # 另一个终端可执行，或启动终端按 Ctrl+C
```

完整重置会重建 Gazebo、任务节点和场景状态。单独调用 Gazebo 的世界重置服务不能代替本项目的重置流程。

五块扩展场景：

```bash
bash scripts/reset_classify.sh --world docs/multi_object_20261002/multi_v3_seed11.world
# 可添加：--sensor-stress --seed 11
```

通用启动参数：`--headless` 关闭窗口，`--auto-start` 自动触发，`--plan-only` 只规划，`--world PATH` 选择场景。每次 reset 都要重新传入扩展参数；不记忆上次 world。压力输入是测试设计，不是 Gemini 实测噪声模型。

## 文本与语音代理

启动仿真后，另开终端运行：

```bash
bash scripts/run_agent.sh --ros-args -p use_llm:=false
```

可输入“桌上有几个红块”“把红色方块放到蓝色盒子里”“拿一个蓝色方块”“回家”“停止”“扫描”；输入 `quit` 退出。`pick_hold` 一次只抓一件，结束后持物，必须重置才能继续。

单条命令示例（会在仿真中执行动作）：

```bash
bash scripts/run_agent.sh --ros-args -p use_llm:=false \
  -p command:="把红色方块放到蓝色盒子里"
```

启用 LLM 时，按需将 `openai` SDK 安装到运行 ROS 节点的同一 Python 环境，并从环境变量读取密钥。默认端点为 DeepSeek，可通过 `llm_model`、`llm_base_url`、`llm_api_key_env` 参数覆盖；调用方式参考 [DeepSeek 官方文档](https://api-docs.deepseek.com/)。未配置密钥时使用规则解析。

```bash
python3 -m pip install --user openai
export DEEPSEEK_API_KEY='<自己的密钥>'
bash scripts/run_agent.sh
```

语音为可选功能，不随仓库分发模型。按 [faster-whisper 官方用法](https://github.com/SYSTRAN/faster-whisper) 下载完整模型，例如较小的 base 模型：

```bash
# 保持 NumPy 1.x，避免破坏 Humble 的 cv_bridge 二进制依赖
python3 -m pip install --user 'numpy<2' faster-whisper sounddevice
python3 - <<'PY'
from pathlib import Path
from faster_whisper.utils import download_model
model_dir = Path.home() / 'models/faster-whisper-base'
download_model('base', output_dir=str(model_dir))
PY
bash scripts/run_agent.sh --ros-args -p use_voice:=true \
  -p voice_model:="$HOME/models/faster-whisper-base"
```

在提示符输入 `voice` 录音 4 秒。默认模型参数仍为 `~/models/faster-whisper-medium`，使用其他模型时要显式覆盖。目录必须包含模型权重，只有 tokenizer/config 不能运行识别；需要可用麦克风及 PortAudio。可添加 `-p enable_tts:=true` 使用已安装的 `espeak-ng` / `espeak` / `spd-say`。

旧入口为 `ros2 run classify_voice voice_command_node`，同样需要语音依赖、模型和 API Key。它的重置动作会提示外部脚本并停止动作队列；不支持按颜色精细选择。麦克风、ASR 准确率、TTS 与付费 LLM 服务的实测边界见发布记录。

## 接口、RViz 与相机

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/classify/start` | `std_srvs/srv/Trigger` | 新鲜完整场景的同色批处理 |
| `/classify/command` | `classify_interfaces/srv/RobotCommand` | 语言任务、选择、异色盒放置、HOME |
| `/classify/cancel` | `std_srvs/srv/Trigger` | 请求停止运动，保留夹爪状态 |
| `/classify/scene_ready` | `std_srvs/srv/Trigger` | 场景检查 |
| `/classify/status`、`/classify/result` | 自定义 msg | 过程、终态与放置结果，TRANSIENT_LOCAL |
| `/classify/scene_status`、`/classify/detections` | 自定义 msg | 场景新鲜度和检测 |
| `/classify/command_text`、`/classify/task_spec` | `std_msgs/msg/String` | 语言输入及解析结果 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB8，640×480 |
| `/camera/native/depth/image_raw` | `sensor_msgs/msg/Image` | 原生 32FC1 深度，米 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | RGB 对齐 16UC1 深度，毫米 |

`RobotCommand`：`intent=pick_place|pick_hold|home`；`colors` 只允许 red/blue；`quantity=0` 表示全部，正数要求指定件数，取料区新鲜对象不足则失败；`pick_hold` 必须为 1。`target_bin` 为空表示同色盒，也可显式 red/blue；`selector=nearest|leftmost|rightmost|first`，左右按 base_link 的 +Y/−Y 解释。放置容量按场景预留槽位，不保证任意跨色组合都能装下。

RViz 的 MotionPlanning 选择 `arm` 或 `gripper`，使用 Current State，先 Plan 再 Execute；夹爪目标为 `open` / `closed`。任务执行时不要同时手动控制同一控制器。查看相机：

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

默认几何使用 **world 坐标**：桌面 z=0.350，红/绿/蓝中心分别为 `(0.40,0,0.375)`、`(0.40,0.14,0.375)`、`(0.40,-0.14,0.375)`，料盒位于 `(0.52,±0.22,0.350)`；base_link 在 world 的 z=0.347，因此基座系坐标需扣除该高度。

## 测试与诊断

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# 避免用户目录里的其他 pytest 插件干扰系统 ROS 测试环境
PYTHONNOUSERSITE=1 colcon test --executor sequential
colcon test-result --verbose
bash doctor.sh
```

`doctor.sh` 检查 renderer、DDS、三路图像、实时率和场景就绪；默认不会停止已有完整仿真。`--headless` 仍需要 Gazebo 相机的渲染上下文。SSH 中 GUI 位于远端桌面；没有可用桌面时，按 [诊断说明](doctor_使用与交付说明.md) 配置显示连接或 Xvfb。运行日志在 `log/`，不入 Git。

独立运动验收工具需要新启动的仿真，且会执行仿真动作。四种 command 用例应分别完整 reset 后运行：

```bash
python3 scripts/tools/verify_task.py --output /tmp/piper_batch_check
python3 scripts/tools/verify_command_session.py --case guard --output /tmp/piper_guard_check
python3 scripts/tools/verify_command_session.py --case cancel --output /tmp/piper_cancel_check
python3 scripts/tools/verify_command_session.py --case cross_bin --output /tmp/piper_cross_check
python3 scripts/tools/verify_command_session.py --case hold --output /tmp/piper_hold_check
```

输出目录必须不存在。`guard` 故意制造缺少目标的失败，检查失败锁定；它不能与正常抓放共用同一任务会话。

## 文件与文档

`src/` 保留 11 包，`scripts/` 为运行/验证入口，`docs/` 保存当前说明与带日期的历史证据。模型下载、编译结果、日志、编辑器索引和 Python 缓存不上传。队友的节点图位于 `docs/images/rosgraph.png`，终端草稿已归档到语言演示目录；旧项目 `piper_classify_ws_old` 不参与构建。

- [完整文档索引](docs/README.md)、[当前交接](docs/05_交接说明.md)、[节点与通信架构](docs/07_系统架构与通信报告.md)、[语言演示](docs/agent_demo_20261019/README.md)
- [本次整理/验收](docs/release_20261028/README.md)、[10 月 10 日 SSH/DDS 修复](docs/remote_deployment_20261010/README.md)
- [基线分拣](docs/task_20261001/README.md)、[同色多块](docs/multi_object_20261002/README.md)、[鲁棒性](docs/robustness_20261002/README.md)、[相机模型](docs/gemini335_20261001/README.md)
- [规划目标](docs/01_规划目标.md)、[开发历史](docs/02_开发进程记录.md)、[早期 GitHub 发布](docs/github_20261002/README.md)

保留代码的 [MIT 许可证](LICENSE) 及各第三方文件原有声明；来源见 [THIRD_PARTY.md](THIRD_PARTY.md)。API Key 只从环境读取，禁止提交 `.env`、凭据或含密钥的运行日志。当前夹持为理想固定约束，Gemini 参数和手眼矩阵不能代替单位设备标定，未验收真机控制、滑移/力反馈或任意未知物体抓取。
