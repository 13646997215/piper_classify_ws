#!/usr/bin/env bash
# ==============================================================================
#  doctor.sh — piper_classify_ws 综合体检 / 在线验证脚本
# ==============================================================================
#  用法:
#    bash doctor.sh                    # 体检 + 在线验证（沿用当前 DDS 环境，不修改系统设置）
#    bash doctor.sh --no-live          # 只做体检，不启动或停止仿真
#    bash doctor.sh --shm              # 本次使用 RGB-D 配置（SHM 32 MiB + UDP）
#    bash doctor.sh --udp              # 本次用 Fast DDS 纯 UDP 作对照试验
#    bash doctor.sh --cyclone          # 改用 Cyclone DDS（Fast DDS 异常时的替代方案）
#    bash doctor.sh --ws /path/to/ws   # 指定工作空间路径
#    bash doctor.sh --stop-existing   # 明确停止本工作区已有完整仿真，再验证
#    bash doctor.sh --keep-sim         # 验证完不关仿真，留给你手动看
#
#  脚本不修改项目、~/.bashrc、/etc/fstab，不删除 /dev/shm 文件。
#  --shm / --udp / --cyclone 只影响本脚本和它启动的子进程。
#  完整报告写入 ~/piper_doctor_<时间戳>.log
# ==============================================================================

# 注意: 这里刻意不使用 set -u / set -e
#   set -u 会让非交互 shell 在遇到未定义变量时直接退出，
#   而 ROS 2 Humble 的 setup.bash 内部大量引用未定义变量（如 AMENT_TRACE_SETUP_FILES）。
#   set -e 会让任意一条"体检用的失败探测"中断整个脚本。
set -o pipefail
export PYTHONDONTWRITEBYTECODE=1

DOCTOR_VERSION="2026-10-28.1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=""; TRANSPORT="inherit"; RUN_LIVE=1; KEEP_SIM=0; STOP_EXISTING=0
while [ $# -gt 0 ]; do
  case "$1" in
    --shm)       TRANSPORT="shm"; shift ;;
    --udp)       TRANSPORT="udp"; shift ;;
    --cyclone)   TRANSPORT="cyclone"; shift ;;
    --no-live)   RUN_LIVE=0; shift ;;
    --keep-sim)  KEEP_SIM=1; shift ;;
    --stop-existing) STOP_EXISTING=1; shift ;;
    --ws)        [ $# -ge 2 ] && [ -n "$2" ] || { echo "--ws 需要路径" >&2; exit 2; }; WS="$2"; shift 2 ;;
    -h|--help)   sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done
if [ -z "$WS" ]; then
  if [ -d "$SCRIPT_DIR/scripts" ]; then WS="$SCRIPT_DIR"; else WS="$HOME/piper_classify_ws"; fi
fi
[ -d "$WS" ] || { echo "工作空间不存在: $WS" >&2; exit 2; }
WS="$(cd "$WS" && pwd -P)"
if [ "$RUN_LIVE" -eq 0 ] && [ "$STOP_EXISTING" -eq 1 ]; then
  echo "--no-live 不停止仿真，不能与 --stop-existing 同用" >&2; exit 2
fi

TS="$(date +%Y%m%d_%H%M%S)_$$"
REPORT="$HOME/piper_doctor_$TS.log"
: > "$REPORT"
WARN=0; FAIL=0; FIXED=""
LAUNCH_PID=""

log()  { printf '%s\n' "$*" | tee -a "$REPORT"; }
sec()  { log ""; log "════════════════════════════════════════════════════════"; log "  $*"; log "════════════════════════════════════════════════════════"; }
ok()   { log "  [ OK  ]  $*"; }
warn() { log "  [WARN]  $*"; WARN=$((WARN+1)); }
bad()  { log "  [FAIL]  $*"; FAIL=$((FAIL+1)); }
info() { log "  [INFO]  $*"; }
fix()  { log "  [FIX ]  $*"; FIXED="${FIXED}
    - $*"; }

TEMP_PROFILE=""; PROBE=""
cleanup() {
  if [ -n "$LAUNCH_PID" ] && [ "$KEEP_SIM" -eq 0 ]; then
    log ""; log "[清理] 停止本次体检启动的仿真..."
    # Only signal our own child wrapper; its EXIT trap checks the recorded launch PID.
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
    ok "本次启动程序已退出；结束状态见启动日志"
  fi
  [ -z "$PROBE" ] || rm -f "$PROBE"
  # A kept simulator still needs the profile for any later participants.
  if [ -n "$TEMP_PROFILE" ] && { [ "$KEEP_SIM" -eq 0 ] || [ -z "$LAUNCH_PID" ]; }; then rm -f "$TEMP_PROFILE"; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

log "╔══════════════════════════════════════════════════════════════╗"
log "║   piper_classify_ws 综合体检 / 在线验证                    ║"
log "╚══════════════════════════════════════════════════════════════╝"
log "脚本版本 : $DOCTOR_VERSION"
log "工作空间 : $WS"
log "报告文件 : $REPORT"
log "DDS 模式 : $TRANSPORT（默认沿用；显式参数仅对本次进程生效）"

###############################################################################
sec "[1/8] 操作系统与 ROS 2 环境"
###############################################################################
if [ -r /etc/os-release ]; then
  . /etc/os-release
  ok "$PRETTY_NAME  内核 $(uname -r)"
else
  warn "读不到 /etc/os-release"
fi
if [ -f /opt/ros/humble/setup.bash ]; then
  ok "ROS 2 Humble 已安装"
else
  bad "缺少 /opt/ros/humble/setup.bash —— 这不是 Ubuntu 22.04 + ROS 2 Humble 环境，脚本无法继续"
  exit 1
fi
command -v gazebo >/dev/null && ok "gazebo: $(gazebo --version 2>&1 | head -1)" || bad "未找到 gazebo"
command -v colcon >/dev/null && ok "colcon 已安装" || warn "未找到 colcon"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash 2>/dev/null
log "ROS_DISTRO=${ROS_DISTRO:-未设置}"
log "RMW 默认实现=${RMW_IMPLEMENTATION:-(未显式设置, 由 ros2 默认决定)}"

###############################################################################
sec "[2/8] 图形栈 / 离屏渲染能力  (Gazebo 相机出图的前提)"
###############################################################################
if [ -f "$WS/scripts/tools/runtime_env.sh" ]; then
  source "$WS/scripts/tools/runtime_env.sh" 2> >(tee -a "$REPORT" >&2)
fi
GL_STATE="unknown"
if [ -z "${DISPLAY:-}" ]; then
  GL_STATE="unavailable"
  warn "DISPLAY 未设置。SSH 中关闭 GUI 不会为 Gazebo Classic 相机创建渲染上下文"
  info "使用远端桌面会话的 DISPLAY/XAUTHORITY 并验证 glxinfo，或参考说明中的 Xvfb 对照试验"
else
  info "DISPLAY=$DISPLAY；XAUTHORITY=${XAUTHORITY:-(默认)}"
fi
GLR=""
if [ "$GL_STATE" != "unavailable" ] && command -v glxinfo >/dev/null; then
  GL_OUTPUT="$(timeout 10s glxinfo -B 2>&1)"
  GL_EXIT=$?
  GLR="$(printf '%s\n' "$GL_OUTPUT" | awk -F: '/OpenGL renderer/{sub(/^ +/,"",$2);print $2;exit}')"
  if [ "$GL_EXIT" -eq 0 ] && [ -n "$GLR" ]; then
    GL_STATE="available"
    case "$GLR" in
      *llvmpipe*|*softpipe*|*swrast*|*Software*)
        warn "OpenGL 渲染器 = $GLR（软件渲染；性能需实测，不能据此断定显卡损坏或未装驱动）" ;;
      *) ok "OpenGL 渲染器 = $GLR" ;;
    esac
  else
    GL_STATE="unavailable"
    warn "当前终端无法建立已验证的 OpenGL/GLX 上下文（glxinfo exit=$GL_EXIT）"
    printf '%s\n' "$GL_OUTPUT" | head -6 | tee -a "$REPORT"
  fi
elif ! command -v glxinfo >/dev/null; then
  warn "未安装 glxinfo；可安装 mesa-utils 后验证，脚本不会自动安装"
fi
[ ! -d /dev/dri ] || info "/dev/dri: $(ls /dev/dri | tr '\n' ' ')"
if command -v nvidia-smi >/dev/null; then
  info "nvidia-smi 只证明驱动接口可访问，不证明本次 GLX 使用该 GPU"
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -2 | tee -a "$REPORT"
fi

###############################################################################
sec "[3/8] DDS 共享内存与文件描述符（仅检查，不自动清理或挂载）"
###############################################################################
SHM_SIZE_KB=$(df -Pk /dev/shm 2>/dev/null | awk 'NR==2{print $2}')
SHM_AVAIL_KB=$(df -Pk /dev/shm 2>/dev/null | awk 'NR==2{print $4}')
SHM_FILES=$(find /dev/shm -maxdepth 1 -name '*fastrtps*' -printf '.\n' 2>/dev/null | wc -l)
if [ -n "$SHM_SIZE_KB" ]; then
  log "  /dev/shm 总量=$((SHM_SIZE_KB/1024)) MB，可用=$((SHM_AVAIL_KB/1024)) MB，Fast DDS 文件=$SHM_FILES"
  if [ "$SHM_AVAIL_KB" -lt 131072 ]; then
    warn "共享内存可用不足 128 MB，请结合实际占用和 DDS 错误判断是否需要处理"
  else
    ok "未发现共享内存可用空间不足的证据"
  fi
  if [ "$SHM_FILES" -gt 0 ]; then
    info "文件可能属于活跃 ROS 进程；数量不能证明残留或损坏，不执行 rm"
    info "确有 SHM 错误时，先停止相关 ROS 进程，再按官方工具 fastdds shm clean 检查清理"
  fi
else
  warn "无法读取 /dev/shm；这影响 SHM 可用性，不足以断定所有 DDS 传输均失败"
fi
log "  文件描述符上限=$(ulimit -n)，硬限制=$(ulimit -Hn)（本次不修改）"

###############################################################################
sec "[4/8] 系统资源"
###############################################################################
log "  CPU 核心数 = $(nproc)   负载 = $(cat /proc/loadavg 2>/dev/null | cut -d' ' -f1-3)"
awk '/MemTotal/{t=$2}/MemAvailable/{a=$2}END{printf "  内存 总量 %.1f GB / 可用 %.1f GB\n", t/1048576, a/1048576}' /proc/meminfo 2>/dev/null
AVAIL_GB=$(awk '/MemAvailable/{printf "%.1f", $2/1048576}' /proc/meminfo 2>/dev/null)
if awk "BEGIN{exit !($AVAIL_GB < 6)}" 2>/dev/null; then
  warn "可用内存 ${AVAIL_GB} GB 偏紧（该项目建议 ≥6 GB）；建议用 --headless 关闭图形界面"
else
  ok "可用内存 ${AVAIL_GB} GB"
fi
df -Pk "$HOME" 2>/dev/null | awk 'NR==2{printf "  家目录磁盘剩余 %.1f GB\n", $4/1048576}'

###############################################################################
sec "[5/8] 工作空间与 URDF 完整性"
###############################################################################
if [ ! -d "$WS" ]; then
  bad "工作空间不存在: $WS  （用 --ws 指定正确路径）"
  exit 1
fi
ok "工作空间存在: $WS"
EXPECT_PKGS="classify_bringup classify_camera_sim classify_description classify_grasp_sim classify_interfaces classify_moveit_config classify_perception classify_scene_bridge classify_task classify_agent classify_voice"
MISSING=""; PKG_TOTAL=0
for p in $EXPECT_PKGS; do
  PKG_TOTAL=$((PKG_TOTAL+1))
  [ -d "$WS/install/$p" ] || MISSING="$MISSING $p"
done
if [ -n "$MISSING" ]; then
  bad "以下包未构建:$MISSING"
  warn "请按 README 顺序构建："
  warn "  source /opt/ros/humble/setup.bash"
  warn "  cd $WS && CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to classify_description --symlink-install --executor sequential"
  warn "  bash scripts/tools/build_urdf.sh"
  warn "  CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential"
else
  ok "$PKG_TOTAL 个 ROS 包全部已构建"
fi
URDF="$WS/src/description/classify_description/urdf/piper_sim.urdf"
if [ ! -f "$URDF" ]; then
  bad "缺少 $URDF"
  warn "该文件由脚本生成且不入 Git，请执行: bash scripts/tools/build_urdf.sh"
else
  ok "piper_sim.urdf 存在 ($(wc -l < "$URDF") 行)"
  MESH_DIR="$WS/src/description/classify_description/meshes"
  if ! grep -Fq "file://$MESH_DIR/" "$URDF"; then
    bad "URDF 的网格路径与当前工作空间不一致（URDF 由脚本生成且含绝对路径，位置变了/未重新生成都会导致网格加载失败）"
    warn "当前工作空间应使用: file://$MESH_DIR/"
    warn "修复: bash scripts/tools/build_urdf.sh"
  else
    ok "网格路径与当前工作空间一致"
  fi
  if grep -q 'filename="package://' "$URDF"; then
    bad "URDF 含 package:// —— Gazebo Classic 不支持，只认 file:// 和 model://，网格会静默加载失败"
    warn "修复: bash scripts/tools/build_urdf.sh"
  fi
  BADMESH=0; NMESH=0
  while IFS= read -r f; do
    NMESH=$((NMESH+1))
    p="${f#file://}"
    [ -f "$p" ] || { BADMESH=$((BADMESH+1)); log "      缺失网格: $p"; }
  done < <(grep -oiE 'filename="[^"]*\.(stl|dae|obj)"' "$URDF" | sed 's/^filename="//;s/"$//')
  if [ "$BADMESH" -gt 0 ]; then bad "$BADMESH/$NMESH 个网格文件不存在"; else ok "$NMESH 个网格文件全部就位"; fi
  if grep -q 'libgazebo_ros2_control.so' "$URDF"; then ok "含 gazebo_ros2_control 插件"; else bad "URDF 缺少 ros2_control 段"; fi
fi

###############################################################################
sec "[6/8] 残留进程与重复启动"
###############################################################################
USE_EXISTING=0
OWN_GROUPS="$(python3 - "$WS" 2> >(tee -a "$REPORT" >&2) <<'PY'
import importlib.util, sys
from pathlib import Path
p = Path(sys.argv[1]) / 'scripts/tools/classify_lifecycle.py'
spec = importlib.util.spec_from_file_location('lifecycle', p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
rows = m.processes()
print(len(m.groups(rows)))
for pid, row in rows.items():
    if m.owned(row) and any(Path(v).name == 'gzserver' for v in row['argv']):
        display = row['env'].get(b'DISPLAY', b'(unset)').decode(errors='replace')
        print(f'  本工作区 gzserver PID={pid}, DISPLAY={display}', file=sys.stderr)
PY
)"
if [[ ! "$OWN_GROUPS" =~ ^[0-9]+$ ]]; then
  bad "无法检查本工作区进程归属，跳过在线验证"
elif [ "$OWN_GROUPS" -gt 0 ]; then
  info "本工作区已有 $OWN_GROUPS 个完整启动进程组；不将其自动认定为残留"
  USE_EXISTING=1
  if [ "$STOP_EXISTING" -eq 1 ]; then
    if [ "$GL_STATE" = "unavailable" ]; then
      bad "重启前未找到可用 GLX，保留已有仿真；先修复显示环境"
    elif bash "$WS/scripts/stop_classify.sh" --quiet; then
      USE_EXISTING=0; fix "按 --stop-existing 停止本工作区完整仿真"
    else
      bad "停止未完成，不启动新的仿真"
    fi
  elif [ "$TRANSPORT" != "inherit" ] && [ "$RUN_LIVE" -eq 1 ]; then
    bad "已有进程不能应用新的 DDS 环境；请先停止，或显式加 --stop-existing"
  fi
else
  ok "未发现本工作区完整启动进程组"
fi

###############################################################################
sec "[7/8] 本次进程的 DDS 配置（不写 ~/.bashrc）"
###############################################################################
if [ "$TRANSPORT" != "inherit" ]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  unset RMW_FASTRTPS_USE_QOS_FROM_XML FASTDDS_BUILTIN_TRANSPORTS
fi
case "$TRANSPORT" in
  inherit) info "沿用当前 RMW 与配置；不声称已启用/验证 SHM" ;;
  shm) export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
       if [ -f "$WS/scripts/config/fastdds_local.xml" ]; then
         export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/scripts/config/fastdds_local.xml"
         info "本次使用项目 RGB-D 配置：每个参与者 SHM 段 32 MiB，并保留 UDP"
       else
         bad "缺少 scripts/config/fastdds_local.xml；请完整同步诊断工具与项目运行环境文件"
       fi ;;
  cyclone)
    if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    else
      bad "缺少 rmw_cyclonedds_cpp；如需此对照，请安装 ros-humble-rmw-cyclonedds-cpp"
    fi ;;
  udp)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    TEMP_PROFILE="$(mktemp /tmp/piper_doctor_udp_XXXXXX.xml)"
    cat > "$TEMP_PROFILE" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor><transport_id>udp_transport</transport_id><type>UDPv4</type></transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_participant" is_default_profile="true">
      <rtps><userTransports><transport_id>udp_transport</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps>
    </participant>
  </profiles>
</dds>
XML
    export FASTRTPS_DEFAULT_PROFILES_FILE="$TEMP_PROFILE"
    info "本次 Fast DDS 纯 UDP 对照：$TEMP_PROFILE；分片不等于必然丢包" ;;
esac
log "  RMW=${RMW_IMPLEMENTATION:-(系统默认)}"
for DDS_VAR in FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML FASTDDS_BUILTIN_TRANSPORTS; do
  [ -z "${!DDS_VAR:-}" ] || log "  $DDS_VAR=${!DDS_VAR}"
done
source "$WS/install/local_setup.bash" 2>/dev/null && ok "工作空间 overlay 已加载" || bad "install/local_setup.bash 加载失败"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GAZEBO_MODEL_DATABASE_URI=""
log "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

###############################################################################
sec "[8/8] 验证"
###############################################################################
PROBE="$(mktemp /tmp/piper_probe_XXXXXX.py)"
cat > "$PROBE" <<'PY'
#!/usr/bin/env python3
"""Measure received ROS samples; do not infer a producer fault from silence."""
import sys
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from classify_interfaces.msg import DetectedBlocks, SceneStatus

WINDOW = float(sys.argv[1])
CONFIG = yaml.safe_load(Path(sys.argv[2]).read_text())
COLOR, DEPTH = CONFIG['color'], CONFIG['depth']
SPEC = [
    ('/camera/color/image_raw', 'rgb8', COLOR['width'], COLOR['height'], 'camera_color_optical_frame'),
    ('/camera/native/depth/image_raw', '32FC1', DEPTH['width'], DEPTH['height'], 'camera_depth_optical_frame'),
    ('/camera/depth/image_raw', '16UC1', COLOR['width'], COLOR['height'], 'camera_color_optical_frame'),
]
REC = {t: [] for t, *_ in SPEC}
META = {}
DET = {'n': 0, 'status': '', 'blocks': 0, 'arrival': float('-inf')}
SCN = {'ready': None, 'detail': ''}
CLOCK = []

def now():
    return time.monotonic()

def sample_summary(samples, started, ended, actual, expected):
    if not samples:
        return 0.0, ended - started, False
    gaps = [samples[0] - started, ended - samples[-1]]
    gaps += [b-a for a, b in zip(samples, samples[1:])]
    maximum = max(gaps)
    valid = len(samples) >= 2 and maximum < 0.8 and actual == expected
    return len(samples) / (ended - started), maximum, valid

class Probe(Node):
    def __init__(self):
        super().__init__('piper_doctor_probe')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, *_ in SPEC:
            self.create_subscription(Image, topic, lambda msg, topic=topic: self.cb(topic, msg), qos)
        self.create_subscription(Clock, '/clock', self.cb_clock, qos)
        self.create_subscription(DetectedBlocks, '/classify/detections', self.cb_det, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SceneStatus, '/classify/scene_status', self.cb_scene, latched)

    def cb(self, topic, msg):
        REC[topic].append(now())
        META[topic] = (msg.encoding, msg.width, msg.height, msg.header.frame_id)

    def cb_clock(self, msg):
        CLOCK.append((now(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    def cb_det(self, msg):
        DET.update(n=DET['n'] + 1, status=msg.status, blocks=len(msg.blocks), arrival=now())

    def cb_scene(self, msg):
        SCN.update(ready=msg.ready, detail=msg.detail)


def main():
    rclpy.init()
    node = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print('  先等待 DDS 发现和图像首帧（最多 5 秒），随后测量 %.1f 秒' % WINDOW, flush=True)
    warmup_end = now() + 5
    while now() < warmup_end and not all(REC.values()):
        executor.spin_once(timeout_sec=0.1)
    for samples in REC.values():
        samples.clear()
    CLOCK.clear()
    started = now()
    while now() - started < WINDOW:
        executor.spin_once(timeout_sec=0.1)
    ended = now()
    duration = ended - started
    perf_ok = True
    all_images = True
    profiles_ok = True
    print('\n  ── 相机接收测量（墙钟 %.1f 秒；配置 %.1f fps）──' % (duration, COLOR['fps']))
    for topic, encoding, width, height, frame in SPEC:
        samples = REC[topic]
        publishers = node.count_publishers(topic)
        if not samples:
            print('  [FAIL] %s: 本探针收到 0 帧；发现的发布者=%d' % (topic, publishers))
            print('         不能仅据零帧区分渲染、插件、DDS 发现/传输或 QoS，需结合日志')
            perf_ok = all_images = False
            continue
        expected = (encoding, width, height, frame)
        rate, max_gap, good = sample_summary(samples, started, ended, META[topic], expected)
        profiles_ok = profiles_ok and META[topic] == expected
        perf_ok = perf_ok and good
        print('  [%s] %s: %.2f Hz，%d 帧，最大无数据间隔 %.3f s，发布者=%d' %
              ('OK' if good else 'WARN', topic, rate, len(samples), max_gap, publishers))
        print('         实测 %s；期望 %s；属性%s' %
              (META[topic], expected, '一致' if META[topic] == expected else '不符'))
    if len(CLOCK) >= 2 and CLOCK[-1][0] > CLOCK[0][0]:
        rtf = (CLOCK[-1][1] - CLOCK[0][1]) / (CLOCK[-1][0] - CLOCK[0][0])
        print('  /clock 实测 RTF≈%.3f；暂停/过慢也可能导致墙钟看门狗失效' % rtf)
    else:
        print('  /clock 未获得足够样本：检查仿真时钟、暂停状态和 DDS 发现')
    print('\n  detections: %d 条，最近 %d 块，%s' % (DET['n'], DET['blocks'], DET['status']))
    print('  scene_status: ready=%s，%s' % (SCN['ready'], SCN['detail']))
    ready_ok = False
    client = node.create_client(Trigger, '/classify/scene_ready')
    if client.wait_for_service(timeout_sec=5):
        future = client.call_async(Trigger.Request())
        deadline = now() + 8
        while not future.done() and now() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if future.done():
            try:
                result = future.result()
                ready_ok = bool(result.success)
                print('  /classify/scene_ready -> success=%s: %s' % (result.success, result.message))
            except Exception as exc:
                print('  [FAIL] scene_ready 响应异常: %s' % exc)
        else:
            print('  [FAIL] scene_ready 调用超时')
    else:
        print('  [FAIL] scene_ready 服务未发现')
    fresh_detection = now() - DET['arrival'] < 0.8
    func_ok = ready_ok and DET['blocks'] > 0 and fresh_detection and all_images and profiles_ok
    verdict = 'PASS' if func_ok and perf_ok else 'PASS_WITH_PERF_WARN' if func_ok else 'FAIL'
    print('  功能判定=%s；稳定性要求：所有图像属性正确且最大无数据间隔 < 0.8 s' % func_ok)
    print('===ONLINE_VERDICT=%s===' % verdict, flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if func_ok else 3

if __name__ == '__main__':
    raise SystemExit(main())
PY

ONLINE_RESULT="SKIPPED"
LAUNCH_LOG=""
if [ "$RUN_LIVE" -eq 0 ]; then
  info "--no-live：只体检，不启动/停止仿真，不修改系统或用户配置"
elif [ "$FAIL" -gt 0 ]; then
  warn "前置检查失败，跳过在线验证"
elif [ "$USE_EXISTING" -eq 0 ] && [ "$GL_STATE" = "unavailable" ]; then
  bad "当前没有可验证的 X/GLX 渲染环境，停止自动启动相机仿真"
  ONLINE_RESULT="RENDER_UNAVAILABLE"
  info "SSH 不继承远端桌面 DISPLAY/XAUTHORITY；不能靠 --headless 或改 DDS 创建渲染上下文"
else
  READY=0
  if [ "$USE_EXISTING" -eq 1 ]; then
    info "仅探测本工作区已有仿真，体检结束不停止它"
    if python3 "$WS/scripts/tools/check_controllers.py" >/dev/null 2>&1; then READY=1; fi
  else
    LAUNCH_LOG="$HOME/piper_doctor_launch_$TS.log"
    log "  启动无 GUI 仿真（相机仍渲染）：bash scripts/run_classify.sh --headless"
    ( cd "$WS" && exec bash scripts/run_classify.sh --headless ) > "$LAUNCH_LOG" 2>&1 &
    LAUNCH_PID=$!
    log "  启动日志: $LAUNCH_LOG"
    for i in $(seq 1 45); do
      sleep 2
      if python3 "$WS/scripts/tools/check_controllers.py" >/dev/null 2>&1; then READY=1; break; fi
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
    done
  fi
  if [ "$READY" -eq 1 ]; then
    ok "三个控制器已 active（只证明控制链路，不证明相机可用）"
    python3 -u "$PROBE" 12 "$WS/src/description/classify_description/config/gemini335.yaml" 2>&1 | tee -a "$REPORT"
    PROBE_EXIT=${PIPESTATUS[0]}
    if [ "$PROBE_EXIT" -eq 0 ] && grep -q '===ONLINE_VERDICT=PASS===' "$REPORT"; then
      ONLINE_RESULT="PASS"; ok "在线检查通过"
    elif [ "$PROBE_EXIT" -eq 0 ] && grep -q '===ONLINE_VERDICT=PASS_WITH_PERF_WARN===' "$REPORT"; then
      ONLINE_RESULT="PASS_WITH_PERF_WARN"; warn "当前功能可用，图像间隔/样本不足以通过稳定性检查"
    else
      ONLINE_RESULT="FAIL"; bad "在线检查未通过（探针 exit=$PROBE_EXIT）"
    fi
  else
    ONLINE_RESULT="FAIL"; bad "本工作区三个控制器未全部 active"
  fi
  if [ -n "$LAUNCH_LOG" ] && [ "$ONLINE_RESULT" != "PASS" ]; then
    log "  启动日志末尾:"; tail -n 25 "$LAUNCH_LOG" | tee -a "$REPORT"
    log "  最近启动日志中的渲染/共享内存错误（若有）:"
    tail -n 500 "$LAUNCH_LOG" | grep -iE "Can't open display|Unable to create X|Rendering will be disabled|GLX|RenderEngine|RTPS_TRANSPORT_SHM|Failed init_port|Native sensor profile/frame mismatch" | head -25 | tee -a "$REPORT"
  fi
fi

sec "汇总"
log "  脚本版本=$DOCTOR_VERSION；在线结果=$ONLINE_RESULT；警告=$WARN；失败=$FAIL"
log "  完整报告: $REPORT"
[ -z "$LAUNCH_LOG" ] || log "  启动日志: $LAUNCH_LOG"
if [ "$ONLINE_RESULT" = "PASS" ]; then
  info "本次在线检查通过；完整抓放验收仍使用项目原有步骤"
elif [ "$RUN_LIVE" -eq 1 ]; then
  info "优先核对远端显示会话：DISPLAY/XAUTHORITY + glxinfo -B；勿盲设 DISPLAY=:0"
  info "约 16 GB 的 /dev/shm 不需要改成 4 GB；Fast DDS 文件数量也不能证明残留"
  info "详细 SSH、PRIME GPU 与 Xvfb 对照步骤见 doctor_使用与交付说明.md"
fi
[ "$FAIL" -eq 0 ] || exit 1
