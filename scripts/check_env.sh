#!/usr/bin/env bash
# ==============================================================================
# S0 · 环境自检脚本
# 只读检查，不修改任何环境
# 用法: bash scripts/check_env.sh
# ==============================================================================
# 注意: 不可使用 `set -u`。ROS 2 Humble 的 setup.bash 会引用未定义变量，
#       开启 nounset 会导致 source 失败（已在 toy_pick_ws 中踩过此坑）。
set -o pipefail

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
PASS=0; FAIL=0; WARN=0

ok()   { printf "  ${GRN}✔${RST} %s\n" "$1"; PASS=$((PASS+1)); }
no()   { printf "  ${RED}✘${RST} %s\n" "$1"; FAIL=$((FAIL+1)); }
wrn()  { printf "  ${YLW}!${RST} %s\n" "$1"; WARN=$((WARN+1)); }
head_() { printf "\n${CYN}== %s ==${RST}\n" "$1"; }

# ---------------------------------------------------------------- 基础
head_ "基础环境"
printf "  OS        : %s\n" "$(. /etc/os-release && echo "$PRETTY_NAME")"
printf "  ROS 2     : %s\n" "${ROS_DISTRO:-<未 source>}"
printf "  工作空间  : %s\n" "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null
printf "  已 source : /opt/ros/humble\n"
printf "  CPU 核数  : %s\n" "$(nproc)"
printf "  可用内存  : %s\n" "$(free -g | awk '/^Mem:/{print $7" GB"}')"
printf "  显卡      : %s\n" "$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo '无 NVIDIA GPU')"
printf "  DISPLAY   : %s\n" "${DISPLAY:-<无>}"

# ---------------------------------------------------------------- 仿真
head_ "仿真器"
if command -v gazebo >/dev/null; then
  printf "  Gazebo    : %s\n" "$(gazebo --version 2>/dev/null | head -1)"
else
  no "未找到 gazebo"
fi
for p in libgazebo_ros_init.so libgazebo_ros_factory.so libgazebo_ros_camera.so libgazebo_ros2_control.so; do
  if [ -f "/opt/ros/humble/lib/$p" ]; then ok "$p"; else no "$p 缺失"; fi
done
if [ -f /opt/ros/humble/lib/libgazebo_ros_depth_camera.so ]; then
  ok "libgazebo_ros_depth_camera.so（独立深度相机插件）"
else
  wrn "无 libgazebo_ros_depth_camera.so —— 可用 <sensor type=\"depth\"> + libgazebo_ros_camera.so 替代（已验证可行）"
fi

# ---------------------------------------------------------------- ros2_control
head_ "ros2_control"
for p in controller_manager ros2_controllers gazebo_ros2_control; do
  [ -d "/opt/ros/humble/share/$p" ] && ok "$p" || no "$p 缺失"
done
for so in libjoint_trajectory_controller.so libjoint_state_broadcaster.so; do
  [ -f "/opt/ros/humble/lib/$so" ] && ok "$so" || no "$so 缺失"
done

# ---------------------------------------------------------------- MoveIt / MTC
head_ "MoveIt / MTC"
for p in moveit_ros_move_group moveit_planners_ompl moveit_simple_controller_manager \
         moveit_kinematics moveit_configs_utils; do
  [ -d "/opt/ros/humble/share/$p" ] && ok "$p" || no "$p 缺失"
done
[ -d /opt/ros/humble/share/moveit_task_constructor_core ] \
  && ok "moveit_task_constructor_core  $(dpkg-query -W -f='${Version}' ros-humble-moveit-task-constructor-core 2>/dev/null | cut -d- -f1)" \
  || no "moveit_task_constructor_core 缺失"
[ -d /opt/ros/humble/share/moveit_task_constructor_visualization ] \
  && ok "moveit_task_constructor_visualization（RViz MTC 面板）" || wrn "无 MTC 可视化插件"
printf "  ${YLW}注意${RST}: MTC 0.1.3 为独立旧版 → ${YLW}仅 C++${RST}，API = plan()/solutions()/execute()\n"
printf "  ${YLW}注意${RST}: Task::execute() 依赖 execute_task_solution action server\n"
printf "        → 本项目改用 solution->toMsg() + MoveGroupInterface::execute() 规避\n"

# ---------------------------------------------------------------- 视觉
head_ "视觉"
[ -d /opt/ros/humble/include/cv_bridge ] && ok "cv_bridge (C++)" || no "cv_bridge 缺失"
pkg-config --exists opencv4 && ok "OpenCV $(pkg-config --modversion opencv4)" || no "OpenCV4 缺失"
for h in "tf2_ros/transform_listener.h" "tf2_eigen/tf2_eigen.hpp" "sensor_msgs/msg/image.hpp"; do
  [ -f "/opt/ros/humble/include/${h%%/*}/${h%%/*}/${h#*/}" ] && ok "$h" || wrn "$h 未找到"
done
[ -d /opt/ros/humble/include/vision_msgs ] && ok "vision_msgs" || wrn "vision_msgs 未安装（计划用自定义 msg，不影响）"

# ---------------------------------------------------------------- 命名冲突
head_ "命名冲突扫描"
N=$(ls /opt/ros/humble/share/ 2>/dev/null | grep -c '^piper' || true)
[ "$N" -eq 0 ] && ok "系统无 piper_* 包，不会抢占 ament index" \
               || wrn "系统存在 $N 个 piper_* 包，可能与本工作空间抢占"
for ws in /home/hanhan/Desktop/toy_pick_ws /home/hanhan/Desktop/raicom_ws; do
  if [ -d "$ws/install" ]; then
    wrn "$(basename "$ws") 存在 install/ → 请勿与本工作空间同时 source"
  else
    ok "$(basename "$ws") 无 install/（未 build，不会冲突）"
  fi
done

# ---------------------------------------------------------------- 上游参考
head_ "上游参考资料（只读，位于 /tmp）"
REF=/tmp/opencode/piper_ref
[ -d "$REF/piper_ros" ] && ok "官方 piper_ros@humble piper_description（参考）" \
                       || wrn "未下载官方参考（S0 步骤：git clone 到 /tmp/opencode/piper_ref）"
if [ -d "$REF/piper_ros/src/piper_description/urdf" ]; then
  C=$(grep -c 'param name="min"' "$REF/piper_ros/src/piper_description/urdf/piper_description_gazebo.xacro" 2>/dev/null || echo 0)
  if [ "$C" -gt 0 ]; then
    wrn "官方 gazebo xacro 含 $C 处 min/max 钳位 → S1 必须删除（否则 joint2 只能到 57°）"
  fi
fi

# ---------------------------------------------------------------- 汇总
printf "\n${CYN}== 汇总 ==${RST}\n"
printf "  通过 %d   警告 %d   失败 %d\n" "$PASS" "$WARN" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
  printf "  ${RED}存在必需组件缺失，请先处理${RST}\n"; exit 1
fi
printf "  ${GRN}环境就绪，可以进入下一阶段${RST}\n"
