> 后续 2026-10-02 已完成复杂场景、防护与恢复补测，见 [新阶段报告](../robustness_20261002/README.md)。

# 重启未恢复初始场景 · 2026-10-01

用户反馈停止并启动后仍显示上一轮执行过的场景。实查发现旧 gzserver PID 570946、PPID 960、PGID/SID 570924 仍在运行，命令行加载本工作区 classify.world，环境 ROS_LOG_DIR 指向本工作区 classify_runtime、domain 42；原 ros2 launch 与 launch.pid 已消失。world 文件仍保存三块原始取料位置，没有被执行结果改写。

旧 stop_classify.sh 在 PID 文件缺失或启动进程退出时直接返回，漏掉仍活着的 Gazebo。run 的 EXIT 清理又没有绑定启动身份，启动/停止缺乏互斥；因此不能仅凭窗口关闭或 PID 消失判断世界已经重建。

现改为 classify_lifecycle.py 管理独立进程组：按工作区环境标识恢复归属，即使没有记录也清理残留；发信号前检查同组所有成员的归属；等待全部活成员退出，必要时强制停止。生命周期锁串行化启动和停止，旧 EXIT trap 使用 expected-pid，避免干扰新启动。发现残留时 run 拒绝创建新世界，提示先停止。没有泛用 pkill，不停止其他工作区的仿真。

新增统一重置命令：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/reset_classify.sh
```

默认重新加载初始场景且不自动分拣；夹爪恢复配置中的初始闭合，不是上一轮完成后的全开。只关闭用 stop_classify.sh。当前已重新打开 GUI 供用户检查。

验证：原孤立 gzserver 已清理；首轮启动三块恢复 world 中的位置、IDLE 0/2、没有旧结果，记录 initial_first.json。随后人为移除 launch.pid 再执行 stop，原组 574518 的所有活成员退出；新启动为另一独立组。模拟旧 trap 调用 expected-pid=574518，新 PID 记录保持不变（final_launch_before_old_trap.pid）。再次确认初始位置、VISION_READY、三个颜色新鲜、计数清零、无旧 result，证据 initial_final.json。

初始夹爪角度/臂姿态按 gazebo_initial_positions.yaml 检查；空闲控制器尚未收到轨迹，存在既有重力稳态下垂，臂关节偏差检查容差 0.08 rad，夹爪 1 mm。这不同于任务执行完 HOME 命令后的 0.065 rad 判据，不把初始化小幅下垂称为旧任务残留。

本轮只改生命周期脚本，不改相机、world 几何、MTC 轨迹或夹持节点。bash/Python 语法检查通过，重置测试未触发分拣运动。
