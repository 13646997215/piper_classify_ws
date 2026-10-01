> 历史证据：本目录对应 2026-09-30 的旧相机和旧布局。2026-10-01 已更换 Gemini 335 模型并后移物块/料盒，新数据见 [当前相机报告](../gemini335_20261001/README.md)。

> GitHub 发布说明：本报告原始 `.npz` 帧已移入工作区外的本机归档，未随仓库上传；JSON、报告和截图保留。离线回放需恢复原始帧或重新采集。见 [清理记录](../github_20261002/README.md)。

# 机械臂改变姿态后的视觉定位修复 · 2026-09-30

用户现象：Gazebo 物块没有移动，改变机械臂位姿后，RViz 中的物块却偏移。用户先审核诊断方案，随后授权实施。本次只修复视觉与规划场景可信度，当时自动抓放待 S5。之后已完成，见 [最新闭环报告](../task_20261001/README.md)。

## 根因与证据

1. 原检测器用 RGB 回调中的“最近一帧深度”，允许 80 ms 时间差。相机随手臂运动时，像素、深度和 RGB 时刻的 TF 不再对应同一观测。缓存时差记录最大 206 ms，其中超过 80 ms 的旧代码本来会丢弃；不能把全部缓存差都视为被接受的误差。
2. **主要问题是几何估计**：原算法把可见彩色点的最高四分位当作顶面，再取 x/y 中位数。只见侧面、顶面被裁切或透视采样密度变化时，仍会给出错误中心。把 RGB-D 精确配对后，在同一批数据上复算原算法，最大误差仍为 **47.36 mm**，排除了“只修同步即可”的假设。
3. 原桥接直接接受任意有限坐标；随后看不见工件时，RViz 一直保留错误位置，没有标记位置过期。原 TF 已按图像时间查询，并非一直使用最新 TF。

原源码备份：[检测器](detect_blocks_before.cpp)、[桥接器](scene_bridge_before.cpp)。原始 RGB-D、相机内参、图像时刻 TF、实际关节角和检测结果在 [baseline/record.json](baseline/record.json)。

## 实施内容

- `message_filters::ExactTime` 配对 RGB 与深度；内参检查尺寸和 frame，TF 仍使用该对图像的时间戳。
- 用已知桌面高度、50 mm 方块尺寸约束完整顶面：高度窗口 ±7 mm，法向倾斜 ≤8°，平面残差 ≤3 mm，宽/深 44–56 mm，10×10 网格覆盖率 ≥75%，至少 150 个点。用顶面空间边界的中点估计中心，拒绝侧面、严重裁切和薄边框。
- 检查连续图像时刻的相机 TF；超过 15 mm/s 或 0.05 rad/s 视为运动，稳定至少 0.4 s 后恢复估计。
- 首次位置或超过 3 mm 的位移须连续至少 3 帧、跨度至少 0.35 s、一致性在 3 mm 内才写入场景。正常小抖动刷新有效性，不推动已确认物块。
- 运动、看不全或断流时保留上次确认的碰撞体位置，变灰并标注 `LAST_SEEN`，状态显示 `VISION_UNAVAILABLE`。这代表历史位置，不代表当前测量。恢复可靠视觉后恢复颜色与 `VISION_READY`。
- `/classify/scene_status` 发布有效/过期颜色及原因；`/classify/scene_ready` 返回当前可用性。未来任务入口须检查此服务，当前尚无自动抓取消费者。
- 视觉和桥接均有 0.8 s 超时，同时检查仿真时间和单调墙钟，仿真时钟暂停也不会让断流永久有效。
- 新消息字段为 `camera_stable/status` 与几何质量数据。`confidence` 当前是平面网格覆盖率，并非校准过的识别概率。重新编译后应重启旧消息节点。

## 验证结果

| 测试 | 结果 |
|---|---|
| 原算法 + 精确配对数据 | 最大中心误差 47.36 mm |
| 同一组 173 对数据回放到新 C++ 节点 | 243 个可靠中心，最大误差 1.295 mm |
| Gazebo 重新执行多位姿测试，173 对数据 | 569 个可靠中心，最大误差 1.349 mm |
| 同一实时测试的 MoveIt 物块中心 | 最大误差 1.116 mm，运动/视野变化没有带走场景物块 |
| joint1 ±0.16 rad、joint5 −0.18 rad、joint1 +0.75 rad，各返回 | 均执行成功；返回观察姿态重新 ready；不完整视野 ready=false |
| Gazebo 红块真实移至 (0.28, 0.035, 0.375) | 场景中心误差 0.953 mm；绿/蓝未变；红块已恢复 |
| 注入单帧错误坐标、过期消息 | 拒绝更新已确认场景 |
| 暂停仿真时钟并中断检测消息 | ready=false，碰撞体位置保留 |
| RGB/深度时间戳差 1 ns | 不生成检测；断流状态失效 |
| 几何与状态跟踪 C++ 回归测试 | 5 个有效用例通过 |

[精简结果 JSON](results.json)、[新实时记录](fixed_live/record.json)、[位移记录](displacement.json)、[场景防护记录](scene_guards.json)、[同步防护记录](baseline/replay_negative.json)。测试中 Gazebo 位姿只用于误差参照和物块临时移动；产品视觉没有读取 Gazebo 工件真值。运动测试先对 25 个插值状态查 MoveIt 碰撞，再下发受控轨迹，最后恢复原关节位置。这不是完整抓放轨迹验收。

RViz 实际画面：[可靠观测](rviz_ready.png)、[停止视觉后的历史位置](rviz_stale.png)。停视觉后 `/get_planning_scene` 确认三块为灰色、alpha=0.55，`scene_ready=false`，见 [颜色状态](stale_colors.json)。

## 用户画面验收

先停止之前的视觉与 MoveIt 终端，停止旧仿真，再按三个终端依次启动：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/run_sim.sh --gui
```

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/run_perception.sh
```

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/run_moveit.sh --rviz
```

1. 在观察姿态等识别稳定，看到三块、三色标签和 `VISION_READY`。
2. 在 MotionPlanning 中用 Joints 只给 joint1 约 ±9° 的小目标，先 Plan 再 Execute；Gazebo 中三块不动，RViz 的三块也应保留原位。运动时状态会暂不可用。
3. 回到原观察姿态，检测恢复。视野不足时允许某些工件变灰，带 `LAST_SEEN`；这时不应把灰色位置作为新抓取输入。
4. 在视觉终端按 Ctrl+C，约 1 秒后三块应变灰，显示 `VISION_UNAVAILABLE`；重启视觉终端后恢复。

RViz 默认关闭调试图层 `Detected blocks`，显示 `Confirmed blocks and vision status` 和 MotionPlanning 的 Scene Geometry。若使用旧的 RViz 配置，请重新从脚本打开，以免同时叠加未确认标记。

用额外终端查看状态：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ROS_DOMAIN_ID=42 ros2 service call /classify/scene_ready std_srvs/srv/Trigger '{}'
```

## 复测工具

产品节点均为 C++。以下 Python 仅作录制、回放和验收工具，不参与产品定位或运动任务。

```bash
# 构建与 C++ 回归测试
source /opt/ros/humble/setup.bash
colcon build --packages-up-to classify_moveit_config classify_perception --symlink-install
colcon test --packages-select classify_perception classify_scene_bridge
colcon test-result --test-result-base build/classify_perception
colcon test-result --test-result-base build/classify_scene_bridge

source install/local_setup.bash
# 保持 Gazebo、MoveIt、视觉运行；输出目录必须不存在；实际运动后恢复
ROS_DOMAIN_ID=42 python3 scripts/tools/verify_perception.py --output /tmp/piper_new_capture --check
# 真实移红块再恢复；保持三个节点运行
ROS_DOMAIN_ID=42 python3 scripts/tools/verify_scene_guards.py --displacement --output /tmp/piper_displacement.json
# 先停视觉节点，再测试桥接防护；保持 Gazebo 和 MoveIt 运行
ROS_DOMAIN_ID=42 python3 scripts/tools/verify_scene_guards.py --injection --output /tmp/piper_guards.json
```

离线回放使用独立域，两个终端分别运行 `ROS_DOMAIN_ID=43 bash scripts/run_perception.sh` 与 `ROS_DOMAIN_ID=43 python3 scripts/tools/replay_perception.py docs/perception_motion_20260930/baseline`。后者加 `--negative` 可测试时间戳不匹配与暂停时钟下的传感器超时。原算法复算：`python3 scripts/tools/analyze_perception_capture.py docs/perception_motion_20260930/baseline`。

## 适用边界

当前针对桌面取料区内、与世界轴平行、50 mm 方块的水平顶面；要求看见足够完整的顶面，倾斜、旋转任意角度或堆叠物块尚未覆盖。严重遮挡时宁可返回不可用，不凭侧面补猜中心。保留历史碰撞体可避免未知状态下直接删掉障碍，但不能替代重新观察。S5 抓放必须先获取新鲜视觉，再管理物块附着、Gazebo 实际夹持和落盒验证。
