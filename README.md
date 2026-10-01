# Piper 视觉分拣仿真

ROS 2 Humble、Gazebo Classic、MoveIt 2/MTC 0.1.3。腕部 Gemini 335 RGB-D 模型定位 50 mm 红/蓝方块，分别抓放入对应料盒，绿块保留；RViz 支持手动机械臂/夹爪控制和 MTC 阶段查看。

三块基线与五块扩展场景已验收，本阶段已完成收尾。三块基线、五块同色多物体场景及 RViz 手臂/夹爪控制已完成；已有验证记录和 20 tests 无失败。收尾未更改运行代码或配置，仿真已停止。历史校验清单见 [收尾记录](docs/closeout_20261002/README.md)。GitHub 发布已清理编译产物，下载后需要先构建，见下方。

## 首次下载与构建

验证环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、MoveIt 2 和 MTC 0.1.3。需要已安装 ROS 2 Humble、colcon、rosdep；MTC 的核心和 RViz 可视化插件也需安装。历史验证记录对应上述版本。

```bash
git clone https://github.com/13646997215/piper_classify_ws.git
cd piper_classify_ws
source /opt/ros/humble/setup.bash
# rosdep 已初始化时安装 package.xml 中声明的依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 首先建立描述包的 ament 索引，供 xacro 查找资源
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to classify_description --symlink-install --executor sequential
bash scripts/tools/build_urdf.sh
# 单个编译任务，降低内存占用
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential
source install/local_setup.bash
python3 scripts/tools/check_scene.py
python3 scripts/tools/verify_camera_model.py
bash scripts/run_classify.sh
```

`piper_sim.urdf` 和 `piper_sim_rviz.urdf` 由脚本生成，不进入 Git。它们包含当前工作区的绝对资源路径；移动工作区后须清理旧 build/install/log 并按上述顺序重新构建。只修改 xacro 或相机配置时，重新生成 URDF、构建并重启仿真。

## 操作与结果

完整触发、反馈、相机、取消与重跑命令见 [当前交接](docs/05_交接说明.md)。结果、截图、验证日志和限制见 [自动分拣报告](docs/task_20261001/README.md)。设计见 [规划目标](docs/01_规划目标.md)，历史汇报见 [输出记录](docs/4_输出.md)。

重置初始场景（默认不自动分拣）：`bash scripts/reset_classify.sh`。仅关闭：`bash scripts/stop_classify.sh`。重启残留的修复证据见 [重置验证](docs/reset_20261001/README.md)。

2026-10-02 已完成第一阶段难度提升：带 seed 的位置扰动、两处障碍和独立深度压力模式；8 项接受记录、13 tests 无失败。运行复杂场景见 [操作说明](docs/05_交接说明.md)，证据/实机缺项见 [阶段报告](docs/robustness_20261002/README.md)。同色多块阶段已加入，见下方。

当前新增五块场景：两红、两蓝、一绿；独立视觉 ID、方块 yaw 与分开放置槽位已完成实体闭环，最终 GUI 压力回归通过。基线三块 world 不变。

```bash
bash scripts/reset_classify.sh --world docs/multi_object_20261002/multi_v3_seed11.world
```

默认不运动，等 VISION_READY 后调用 `/classify/start`。五块场景约 3 分钟，完成 4/4，绿块不动。新接口中 `current_object`、`placed_objects` 和结果 `objects` 使用视觉 ID；`physical_objects` 只记录夹持后的仿真验证绑定。证据和限制见 [同色多块报告](docs/multi_object_20261002/README.md)。

## 仓库内容

保留九个 ROS 包、脚本、模型网格、场景、配置、历史报告、JSON 验证结果和截图；build/install/log、编辑器索引和 Python 缓存均已清理并加入忽略规则。原始 RGB-D 帧与旧源码压缩快照另存于本机工作区外，未上传；历史 JSON 中的采样文件名仍是原始记录，离线回放需恢复帧数据或重新采集。详见 [发布与清理记录](docs/github_20261002/README.md)。

当前为仿真执行项目，尚无真机 CAN 执行后端。夹持使用理想固定约束，官方标称相机参数和已有手眼矩阵也不能代替单位设备实测。已覆盖的场景及限制见各验收报告。[第三方来源](THIRD_PARTY.md) 与 [MIT 许可证](LICENSE) 一并保留。
