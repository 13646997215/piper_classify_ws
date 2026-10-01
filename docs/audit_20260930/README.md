# 接管审查证据

这些文件记录 **2026-09-30 修复前** 的独立检查结果，不代表后续版本的运行状态。

当前场景修复结果和用户 GUI 验收见 [05_交接说明.md](../05_交接说明.md) 与 [4_输出.md](../4_输出.md) 末尾。下方命令会读取**运行命令时的现行源码**，因此重新运行的数值未必与本目录归档的修复前输出相同。

| 文件 | 内容 |
|---|---|
| `file_manifest.json` | 接管时全部 298 个文件/链接条目的路径、哈希/链接目标、类型与大小 |
| `sdf_probe.cpp` | 用本机 libsdformat9 读取有效几何和传感器的 C++ 探针，不启动 Gazebo |
| `sdf_bins.txt` | 修复前料盒只解析到单块板，位姿偏移为零 |
| `sdf_robot.txt` | 腕部相机实际被保留在固定关节合并后的 link6 上 |
| `world_parsed.sdf` | gz sdf -p 的文本转换结果；包含错误结构也不报错，不能单独作为验收依据 |
| `world_before_rotation.world` | 旋转前的历史备份，用于比对桌板中心和长宽；不是现行 world |
| `kinematics_probe.py` | 从运行时现行 URDF 解析运动学、验证旧 IK 残差、数值求解与计算初始视野的临时审查工具 |
| `kinematics_results.json` | 旧 IK 的真实 180° 姿态误差、世界高度偏差及修正后的数值计算结果 |

复现有效 SDF 几何检查（使用本机已经安装的编译器与库）：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
mkdir -p /tmp/piper_classify_audit
g++ docs/audit_20260930/sdf_probe.cpp \
    -o /tmp/piper_classify_audit/sdf_probe \
    $(pkg-config --cflags --libs sdformat9)
/tmp/piper_classify_audit/sdf_probe \
    src/description/classify_description/worlds/classify.world
/tmp/piper_classify_audit/sdf_probe \
    src/description/classify_description/urdf/piper_sim.urdf
```

运动学审查工具依赖本机现有 numpy、scipy、PyYAML：

```bash
python3 -u docs/audit_20260930/kinematics_probe.py
```

运动学工具读取运行时现行文件，输出到 `/tmp/piper_classify_audit/kinematics_results.json`。修复后相机、基座和 world 已变化，再运行不能逐字节重现修复前归档结果；原始 `kinematics_results.json` 是审查时的快照。多初值数值搜索没有给出全局无解证明，也没有做碰撞检查。报告中的结构错误、坐标变换错误和 180° 残差误报有独立直接证据。
