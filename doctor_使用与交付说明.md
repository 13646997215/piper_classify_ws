# Piper 仿真跨电脑诊断与 SSH 渲染说明

适用：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11；配套 `doctor.sh` 版本 **2026-10-28.1**。项目主构建与抓放操作见 [README](README.md) 和 [交接说明](docs/05_交接说明.md)。请同步整个项目中的脚本和 `scripts/config/fastdds_local.xml`，不要只复制 doctor。

当前版本检查 11 个 ROS 包，适用于语言/语音扩展工程。下面的远端实测属于 2026-10-10 的历史修复；2026-10-28 的构建与接口验收见 [本次发布记录](docs/release_20261028/README.md)。

## 远端实测结论

2026-10-10 经免密 SSH 在远端实际排查。最初报告的零图像不能单凭 `DISPLAY` 未设置归因：旧 OGRE 日志已有 NVIDIA 渲染记录。固定同一有效 NVIDIA/GLX 环境后，进行了 DDS 对照：

| 配置 | RGB | 原生深度 | 对齐深度 | 场景 |
| --- | ---: | ---: | ---: | --- |
| Fast DDS 默认内建传输 | 0.08 Hz | 0.33 Hz | 0 Hz | 无同步观测，未就绪 |
| 显式 SHM 32 MiB + UDP | 14.99 Hz | 14.90 Hz | 14.90 Hz | 3 方块，scene_ready=True |

两轮物理实时率均约 1.0。根因是 **Fast DDS 每个参与者的默认共享内存段太小，图像传输失效**，与整个 `/dev/shm` 的剩余容量不同。远端使用 Fast DDS 2.6.12；[2.6 官方文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html) 给出默认段大小 512 KiB，并明确提醒段大小不应小于消息。这里单帧 RGB 约 0.9 MB，原生深度约 1.2 MB。修复配置为每参与者 32 MiB，保留 UDP 传输。

本项目 `--headless` 只将 GUI/RViz 设为 false；图像仍由 `gzserver` 里的相机传感器渲染。Gazebo 11 的 [RenderEngine 官方源码](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/rendering/RenderEngine.cc) 在 `CreateContext()` 中调用 `XOpenDisplay`、建立 GLX 上下文；创建失败时 `Load()` 会禁用渲染。所以没有窗口与不需要显示服务器是两件事。

`nvidia-smi` 能访问 RTX 4060，不能证明 SSH 中的 OpenGL 已连接到它。软件 renderer 也不能直接证明硬件损坏；混合显卡选择、显示会话、容器权限和显卡库均可能影响结果。

报告中 `/dev/shm` 约 16 GB 且基本空闲，没有容量不足证据。44 个 Fast DDS 文件可能属于运行中的节点，不能仅据文件名/数量判定残留。不要把 16 GB 改小成 4 GB，也不要直接通配删除 `/dev/shm/fastrtps_*`。

旧 doctor 曾向 `~/.bashrc` 写入强制纯 UDP 配置。本次已备份并移除远端的 `piper_doctor` 标记块，保留用户其余设置及旧 XML 文件。新版不再写入这些设置。使用新版时先确认开头打印 `2026-10-28.1`。

## 日常启动：SSH 中自动选择环境

远端桌面已以同一用户登录时，项目启动入口会自动查找该用户 GNOME/Xwayland 会话，并通过 GLX 验证；没有显式 DDS 配置时，自动使用项目内的 32 MiB 配置。直接执行：

```bash
cd ~/piper_classify_ws
bash doctor.sh                          # 在线体检；自行启动的仿真测完关闭
bash scripts/reset_classify.sh          # 重置并打开 Gazebo 和 RViz，默认不自动分拣
# Ctrl+C 结束，或在另一个 SSH 终端：
bash scripts/stop_classify.sh
```

GUI 窗口显示在远端电脑的桌面。SSH 终端本身不显示窗口；`--headless` 关闭窗口但保留相机渲染。显式设置的 RMW、DDS profile 或 DISPLAY 会被保留；它们必须有效且与其他节点一致。需要关闭自动选择时，分别设置 `CLASSIFY_DEFAULT_DDS=0`、`CLASSIFY_AUTO_DISPLAY=0`。

在 SSH 中直接使用 `ros2` 命令或 Python 验证工具时，也应加载同一环境：

```bash
cd ~/piper_classify_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source scripts/tools/runtime_env.sh
export ROS_DOMAIN_ID=42
ros2 service call /classify/scene_ready std_srvs/srv/Trigger '{}'
```

## 自动识别未成功时：手动连接远端桌面

以下命令均在**运行仿真的那台远端电脑**执行。不要填写当前协助电脑的 DISPLAY，也不要直接假定远端是 `:0`。

1. 在远端 Ubuntu 桌面中，以 `wujian` 打开一个终端，确认该会话可建立 OpenGL：

```bash
# 缺 glxinfo 时才安装
sudo apt install mesa-utils
glxinfo -B

# 保存该桌面会话的显示地址和授权文件路径；不复制授权 cookie
(
  umask 077
  {
    printf 'export DISPLAY=%q\n' "$DISPLAY"
    if [ -n "${XAUTHORITY:-}" ]; then
      printf 'export XAUTHORITY=%q\n' "$XAUTHORITY"
    else
      printf 'unset XAUTHORITY\n'
    fi
  } > "$HOME/.piper_gazebo_display.env"
)
```

2. 回到远端 SSH / VS Code Remote SSH 终端，使用同一用户加载：

```bash
cd ~/piper_classify_ws
source "$HOME/.piper_gazebo_display.env"
glxinfo -B
```

只有此处 `glxinfo -B` 成功，才继续启动。远端桌面退出/重新登录后，DISPLAY 或 XAUTHORITY 可能改变，应重新生成此文件。`Authorization required` / `unable to open display` 表示连接或授权仍未正确；不要用 `sudo` 启动仿真或 `xhost +` 绕过权限。

3. RTX 笔记本若默认使用其他 renderer，可做 NVIDIA PRIME 对照：

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B
```

如果这条成功且显示 NVIDIA renderer，则在同一 SSH 终端设置：

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

这是 NVIDIA 官方的 [PRIME Render Offload 用法](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/optimus-laptops-and-multi-gpu-desktop-systems.html)。它不会创建缺失的 X 服务器，也不会修复无效的 XAUTHORITY；需先完成上面的显示连接。

4. 用更新后的诊断脚本验证（先替换远端脚本，不要对旧脚本直接使用新参数）：

```bash
bash doctor.sh --shm --stop-existing
```

`--stop-existing` 明确允许先停止本工作区完整仿真；不处理其他工作区。此处 `--shm` 强制使用项目的 SHM 32 MiB + UDP 配置，仅影响本次脚本和其子进程。

## 没有可用桌面：Xvfb 软件渲染对照

此方案验证无桌面时能否恢复图像，**不代表已使用 RTX 或达到正式任务性能要求**。从 SSH 执行：

```bash
sudo apt install xvfb xauth mesa-utils libgl1-mesa-dri
cd ~/piper_classify_ws

env -u __NV_PRIME_RENDER_OFFLOAD -u __GLX_VENDOR_LIBRARY_NAME \
  LIBGL_ALWAYS_SOFTWARE=1 \
  xvfb-run -a -s '-screen 0 1280x1024x24 +extension GLX' glxinfo -B
```

这一步成功后，可用更新后的 doctor：

```bash
env -u __NV_PRIME_RENDER_OFFLOAD -u __GLX_VENDOR_LIBRARY_NAME \
  LIBGL_ALWAYS_SOFTWARE=1 \
  xvfb-run -a -s '-screen 0 1280x1024x24 +extension GLX' \
  bash doctor.sh --shm --stop-existing
```

软件 renderer 常为 llvmpipe；这里属于明确选择的软件对照模式。不要结合 `doctor.sh --keep-sim` 使用此命令，因为 doctor 退出后 `xvfb-run` 会关闭显示服务器，遗留仿真将失去渲染环境。

长时间保留无窗口仿真时，让 `xvfb-run` 包住仿真本身，并保持该终端运行：

```bash
# 停止本项目先前的完整仿真
bash scripts/stop_classify.sh

env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  -u FASTDDS_BUILTIN_TRANSPORTS -u RMW_FASTRTPS_USE_QOS_FROM_XML \
  -u __NV_PRIME_RENDER_OFFLOAD -u __GLX_VENDOR_LIBRARY_NAME \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1 \
  xvfb-run -a -s '-screen 0 1280x1024x24 +extension GLX' \
  bash scripts/run_classify.sh --headless
```

另一个 SSH 终端可用 `bash doctor.sh` 探测已有完整仿真，默认不会停止它；监控端自身没有 DISPLAY 不妨碍订阅图像。长期 GPU 无桌面部署需单独配置能通过 GLX 验证的远端 GPU 显示服务器，本次不自动修改 Xorg 或驱动。

## doctor 参数和副作用

```bash
bash doctor.sh --help
bash doctor.sh --no-live                 # 只体检，保存报告
bash doctor.sh                           # 沿用当前 DDS；已有完整仿真时只探测
bash doctor.sh --shm --stop-existing      # 本次用 SHM 32 MiB + UDP，重新启动验证
bash doctor.sh --udp --stop-existing      # 纯 UDP 对照，不能预先断定会丢包
bash doctor.sh --cyclone --stop-existing  # 需已安装 rmw_cyclonedds_cpp
bash doctor.sh --ws /path/to/piper_classify_ws
bash doctor.sh --keep-sim                 # 保留本次启动的仿真；不要与临时 Xvfb 包装混用
```

新脚本不修改项目源码/配置、不写 `~/.bashrc` 或 `/etc/fstab`、不挂载 `/dev/shm`、不删除共享内存文件、不自动安装软件。显式 DDS 选择只对当前进程及其子进程生效；纯 UDP XML 和探针使用临时文件。`--keep-sim` 时保留其需要的临时 XML，路径会打印。

默认不停止已有仿真。若要改变已有进程的 DDS 配置，必须先停止，或明确指定 `--stop-existing`。正常结束时只终止本次启动的子包装程序，由项目已有的 PID 校验退出逻辑清理对应进程组；复用的仿真不清理。单独用 `run_sim.sh` / 多终端搭建的仿真不属于完整启动入口，应按其停止脚本处理。

其他机器若仍有旧脚本写进 `~/.bashrc` 的 `piper_doctor` 标记块，需备份后核对/撤销。新版不自动编辑用户文件；`--shm` 只在本次运行中覆盖可能影响 Fast DDS 的 profile 环境变量，不能替代整理未来终端的配置。

## 结果如何判断

- `RENDER_UNAVAILABLE`：当前显示/GLX 不可用且没有可复用的完整仿真，因此不会盲目启动相机验证。
- `SKIPPED`：指定 `--no-live` 或前置检查未通过；不是在线验证成功。
- `PASS`：测量窗口内三路图像均有多个样本、属性匹配，含首尾在内的最大无数据间隔 < 0.8 s，且新鲜检测和 scene_ready 服务通过。
- `PASS_WITH_PERF_WARN`：当前场景可用，但图像间隔或样本数没有通过稳定性检查；不能当作完整抓放验收。
- `FAIL`：在线探针失败，或当前图像/检测/scene_ready 不可用。

进程检查失败或在线失败时返回非零退出码。报告里的图像频率是墙钟接收率；配置的 15 fps 是仿真时间更新率。脚本同时测量 `/clock` 变化估计 RTF，暂停、渲染过慢、同步/TF、DDS、QoS 都需要分别排查。

图像订阅使用 BEST_EFFORT，场景状态使用明确的 TRANSIENT_LOCAL 枚举。收到零帧只说明**该探针没有收到样本**；即使发现发布者也不能排除渲染或传输问题。只收到一帧后停流不会再因“相邻帧间隔为零”误判稳定性通过，图像属性错误也会影响功能判定。

## 缓存与 DDS 应如何处理

先查看确切错误，不把所有 Fast DDS 文件当垃圾：

```bash
# 使用失败那一轮的实际日志路径
rg -n -i 'display|GLX|Ogre|render|RTPS_TRANSPORT_SHM|Failed init_port|Native sensor profile/frame mismatch' \
  ~/piper_doctor_launch_20261010_110109.log

df -h /dev/shm
```

没有 `rg` 时使用 `grep -nEi`。确认存在 SHM 初始化错误后，停止相关 ROS 程序再使用发行版自带的官方工具（若已安装）：

```bash
fastdds shm --help
fastdds shm clean
```

[Fast DDS 2.6 官方 CLI 文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastddscli/cli/cli.html) 说明 `shm clean` 用于清理未再使用的共享内存。它与直接通配删除文件不同。无需同时重装驱动、切换 DDS、重挂载内存或重编译；逐项对照才有诊断意义。

## 回传材料与验证边界

若仍失败，提供新版 doctor 报告、同轮启动日志，以及**启动仿真的同一终端**中的 `glxinfo -B` 输出。脚本版本、RMW、DDS 配置路径和 domain 会打印，便于识别脚本差异与环境不一致。

本次已在远端做真实在线对照，不修改驱动、机械臂运动、相机视场/手眼、物块/料盒或视觉算法。诊断不等同于完整抓放验收，后者需运行项目的独立验证工具。详细执行记录见 [远端部署修复](docs/remote_deployment_20261010/README.md)。
